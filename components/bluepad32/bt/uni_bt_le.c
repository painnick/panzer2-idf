/*
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

/*
 * Copyright (C) 2023 Ricardo Quesada
 * Unijoysticle additions based on BlueKitchen's test/example code
 */

/*
 * Execution order (simple HOG path — bypasses flaky BTstack hids_client):
 *  uni_bt_le_on_gap_event_advertising_report() -> hog_connect()
 *  uni_sm_packet_handler()  (pairing / re-encryption)
 *  schedule_hog_setup()
 *    -> probe all GATT primary services (find HID 0x1812 handles)
 *    -> discover characteristics in that range
 *    -> short-read Report Map
 *    -> enable CCC notifications on Report chars
 *    -> uni_hid_device_set_ready()
 */

#include "bt/uni_bt_le.h"

#include <bluetooth_data_types.h>
#include <btstack.h>
#include <btstack_config.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "bt/uni_bt_conn.h"
#include "bt/uni_bt_defines.h"
#include "parser/uni_hid_parser.h"
#include "uni_common.h"
#include "uni_config.h"
#include "uni_hid_device.h"
#include "uni_log.h"
#include "uni_property.h"

static bool is_scanning;
static bool ble_enabled;

// Temporal space for SDP in BLE
static uint8_t hid_descriptor_storage[HID_MAX_DESCRIPTOR_LEN * CONFIG_BLUEPAD32_MAX_DEVICES];
static btstack_packet_callback_registration_t sm_event_callback_registration;

// ---------------------------------------------------------------------------
// Simple HOG host (bypasses BTstack hids_client — hangs on GamePadPlus V3)
// ---------------------------------------------------------------------------
#define HOG_SETUP_DELAY_MS 300
#define HOG_SETUP_MAX_RETRIES 20
#define HOG_POST_PROBE_DELAY_MS 80
#define HOG_REPORT_MAP_TIMEOUT_MS 2000
#define HOG_CCC_TIMEOUT_MS 1500
#define HOG_MAX_NOTIFY_REPORTS 4
#define HOG_SIMPLE_CID 0x9001  // synthetic cid so device lookup still works
// GamePadPlus reports map len=122 successfully; keep reading real map.
#define HOG_TRY_REPORT_MAP_READ 1

// Minimal Android-style Game Pad report map (fallback when device hangs on Report Map read).
// Layout: 16 buttons + X/Y/Z/Rz axes + hat — enough for Bluepad32 Android parser.
static const uint8_t k_fallback_gamepad_report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x10,        //     Usage Maximum (Button 16)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x10,        //     Report Count (16)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x32,        //     Usage (Z)
    0x09, 0x35,        //     Usage (Rz)
    0x15, 0x00,        //     Logical Minimum (0)
    0x26, 0xFF, 0x00,  //     Logical Maximum (255)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x04,        //     Report Count (4)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x09, 0x39,        //     Usage (Hat switch)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x07,        //     Logical Maximum (7)
    0x35, 0x00,        //     Physical Minimum (0)
    0x46, 0x3B, 0x01,  //     Physical Maximum (315)
    0x65, 0x14,        //     Unit (Eng Rot: Angular Pos)
    0x75, 0x04,        //     Report Size (4)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x75, 0x04,        //     Report Size (4)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x03,        //     Input (Const,Var,Abs)  // padding
    0xC0,              //   End Collection
    0xC0,              // End Collection
};

typedef enum {
    HOG_STATE_IDLE = 0,
    HOG_STATE_PROBE_SERVICES,
    HOG_STATE_W4_CHARS,
    HOG_STATE_W4_REPORT_MAP,
    HOG_STATE_W4_CCC,
    HOG_STATE_READY,
} hog_state_t;

static btstack_timer_source_t hog_setup_timer;
static hci_con_handle_t hog_pending_con_handle = HCI_CON_HANDLE_INVALID;
static int hog_setup_retries = 0;

static hog_state_t hog_state = HOG_STATE_IDLE;
static hci_con_handle_t hog_con_handle = HCI_CON_HANDLE_INVALID;
static uint16_t hog_hid_start = 0;
static uint16_t hog_hid_end = 0;
static bool hog_found_hid = false;
static int hog_service_count = 0;

static gatt_client_characteristic_t hog_report_map_char;
static bool hog_have_report_map = false;
static gatt_client_characteristic_t hog_notify_chars[HOG_MAX_NOTIFY_REPORTS];
static gatt_client_notification_t hog_notify_listeners[HOG_MAX_NOTIFY_REPORTS];
static uint8_t hog_notify_report_ids[HOG_MAX_NOTIFY_REPORTS];  // HID Report ID for each notify char
static int hog_notify_count = 0;
static int hog_ccc_index = 0;
static bool hog_simple_active = false;

static btstack_timer_source_t hog_after_probe_timer;
static hci_con_handle_t hog_after_probe_handle = HCI_CON_HANDLE_INVALID;
static btstack_timer_source_t hog_step_timer;
static btstack_timer_source_t hog_report_map_timeout_timer;
static btstack_timer_source_t hog_ccc_timeout_timer;
static bool hog_report_map_got_value = false;
static uint8_t hog_ccc_enable_value[2] = {0x01, 0x00};  // notifications on

static void uni_hids_client_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void uni_device_information_packet_handler(uint8_t packet_type,
                                                  uint16_t channel,
                                                  uint8_t* packet,
                                                  uint16_t size);
static void schedule_hog_setup(hci_con_handle_t con_handle);
static void hog_disconnect(hci_con_handle_t con_handle);
static void hog_reset_session(void);
static void hog_start_char_discovery(hci_con_handle_t con_handle);
static void hog_gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void hog_notification_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void hog_finish_ready(hci_con_handle_t con_handle);
static void hog_enable_next_ccc(void);
static void hog_read_report_map(void);
static void hog_defer_step(void (*handler)(btstack_timer_source_t*), uint32_t delay_ms);
static void hog_step_enable_ccc(btstack_timer_source_t* ts);
static void hog_step_read_map(btstack_timer_source_t* ts);
static void hog_skip_report_map_and_enable_ccc(const char* reason);
static void hog_apply_fallback_descriptor(hci_con_handle_t con_handle);
static void hog_assign_report_ids_from_descriptor(uni_hid_device_t* device);

// Alias used by older call sites in this file
#define schedule_hids_client_connect schedule_hog_setup

/**
 * Connect to remote device but set timer for timeout
 */
static void hog_connect(bd_addr_t addr, bd_addr_type_t addr_type) {
    // Stop scan, otherwise it will be able to connect.
    // Happens in ESP32, but not in libusb
    gap_stop_scan();
    logd("BLE scan -> 0\n");

    gap_connect(addr, addr_type);
}

static void apply_known_vid_pid(uni_hid_device_t* device) {
    if (device->vendor_id != 0 || device->product_id != 0)
        return;
    if (device->name[0] == '\0')
        return;
    if (strstr(device->name, "GamePadPlus") != NULL || strstr(device->name, "GamePad") != NULL) {
        uni_hid_device_set_vendor_id(device, 0x1949);
        uni_hid_device_set_product_id(device, 0x0402);
        logd("Assumed VID/PID 0x1949/0x0402 from name '%s'\n", device->name);
    }
}

static void hog_reset_session(void) {
    int i;
    if (hog_simple_active) {
        for (i = 0; i < hog_notify_count; i++) {
            gatt_client_stop_listening_for_characteristic_value_updates(&hog_notify_listeners[i]);
        }
    }
    btstack_run_loop_remove_timer(&hog_report_map_timeout_timer);
    btstack_run_loop_remove_timer(&hog_ccc_timeout_timer);
    hog_state = HOG_STATE_IDLE;
    hog_con_handle = HCI_CON_HANDLE_INVALID;
    hog_hid_start = 0;
    hog_hid_end = 0;
    hog_found_hid = false;
    hog_service_count = 0;
    hog_have_report_map = false;
    hog_notify_count = 0;
    hog_ccc_index = 0;
    hog_simple_active = false;
    hog_report_map_got_value = false;
    memset(&hog_report_map_char, 0, sizeof(hog_report_map_char));
    memset(hog_notify_chars, 0, sizeof(hog_notify_chars));
    memset(hog_notify_listeners, 0, sizeof(hog_notify_listeners));
    memset(hog_notify_report_ids, 0, sizeof(hog_notify_report_ids));
}

static void resume_scanning_hint(void) {
    if (is_scanning) {
        gap_start_scan();
        logd("BLE scan -> 1\n");
    }
}

static void hog_defer_step(void (*handler)(btstack_timer_source_t*), uint32_t delay_ms) {
    btstack_run_loop_remove_timer(&hog_step_timer);
    btstack_run_loop_set_timer_handler(&hog_step_timer, handler);
    btstack_run_loop_set_timer(&hog_step_timer, delay_ms);
    btstack_run_loop_add_timer(&hog_step_timer);
}

static void hog_finish_ready(hci_con_handle_t con_handle) {
    uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (!device) {
        loge("hog_finish_ready: no device\n");
        hog_disconnect(con_handle);
        return;
    }

    apply_known_vid_pid(device);
    hog_apply_fallback_descriptor(con_handle);  // no-op if map already set
    hog_assign_report_ids_from_descriptor(device);
    device->hids_cid = HOG_SIMPLE_CID;
    hog_simple_active = true;
    hog_state = HOG_STATE_READY;

    logi("Gamepad ready (%s), reports=%d\n", bd_addr_to_str(device->conn.btaddr), hog_notify_count);

    uni_hid_device_guess_controller_type_from_pid_vid(device);
    uni_hid_device_connect(device);
    uni_hid_device_set_ready(device);
    resume_scanning_hint();
}

static void hog_step_enable_ccc(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    hog_enable_next_ccc();
}

static void hog_step_read_map(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    hog_read_report_map();
}

static void hog_apply_fallback_descriptor(hci_con_handle_t con_handle) {
    uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (!device)
        return;
    if (device->hid_descriptor_len > 0)
        return;
    logd("Simple HOG: fallback Report Map (%u bytes)\n", (unsigned)sizeof(k_fallback_gamepad_report_map));
    uni_hid_device_set_hid_descriptor(device, k_fallback_gamepad_report_map, sizeof(k_fallback_gamepad_report_map));
}

static void hog_skip_report_map_and_enable_ccc(const char* reason) {
    btstack_run_loop_remove_timer(&hog_report_map_timeout_timer);
    logd("Simple HOG: %s — skip Report Map, enable CCC\n", reason ? reason : "skip map");
    if (hog_con_handle != HCI_CON_HANDLE_INVALID)
        hog_apply_fallback_descriptor(hog_con_handle);
    hog_ccc_index = 0;
    hog_defer_step(hog_step_enable_ccc, 50);
}

static void hog_report_map_timeout_handler(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    if (hog_state != HOG_STATE_W4_REPORT_MAP)
        return;
    // ShanWan BM-769 often never answers Report Map ATT Read — don't stall forever.
    hog_skip_report_map_and_enable_ccc("Report Map read timed out");
}

static void hog_read_report_map(void) {
    if (hog_con_handle == HCI_CON_HANDLE_INVALID)
        return;

#if !HOG_TRY_REPORT_MAP_READ
    hog_skip_report_map_and_enable_ccc("skipping Report Map read (pad compatibility)");
    return;
#else
    if (!gatt_client_is_ready(hog_con_handle)) {
        logd("Simple HOG: GATT not ready for Report Map, retry\n");
        hog_defer_step(hog_step_read_map, 50);
        return;
    }
    hog_report_map_got_value = false;
    logd("Simple HOG: reading Report Map handle=0x%04x\n", hog_report_map_char.value_handle);
    // Prefer long read so full maps are complete (short read may truncate).
    uint8_t st = gatt_client_read_long_value_of_characteristic_using_value_handle(
        hog_gatt_handler, hog_con_handle, hog_report_map_char.value_handle);
    if (st != ERROR_CODE_SUCCESS) {
        logd("Simple HOG: long Report Map failed 0x%02x, try short read\n", st);
        st = gatt_client_read_value_of_characteristic_using_value_handle(hog_gatt_handler, hog_con_handle,
                                                                        hog_report_map_char.value_handle);
    }
    if (st != ERROR_CODE_SUCCESS) {
        hog_skip_report_map_and_enable_ccc("Report Map read start failed");
        return;
    }
    hog_state = HOG_STATE_W4_REPORT_MAP;
    btstack_run_loop_remove_timer(&hog_report_map_timeout_timer);
    btstack_run_loop_set_timer_handler(&hog_report_map_timeout_timer, &hog_report_map_timeout_handler);
    btstack_run_loop_set_timer(&hog_report_map_timeout_timer, HOG_REPORT_MAP_TIMEOUT_MS);
    btstack_run_loop_add_timer(&hog_report_map_timeout_timer);
#endif
}

// Assign Report IDs from HID descriptor (0x85 tags) in discovery order.
// BTstack hids_client prepends Report ID before parsing; we must do the same.
static void hog_assign_report_ids_from_descriptor(uni_hid_device_t* device) {
    int i, n = 0;
    uint8_t ids[HOG_MAX_NOTIFY_REPORTS];

    if (!device || device->hid_descriptor_len < 2)
        return;

    memset(ids, 0, sizeof(ids));
    for (i = 0; i + 1 < device->hid_descriptor_len && n < HOG_MAX_NOTIFY_REPORTS; i++) {
        // Report ID item: short item tag 0x85 (bTag=1000, bType=global, bSize=1)
        if (device->hid_descriptor[i] == 0x85) {
            ids[n++] = device->hid_descriptor[i + 1];
            i++;  // skip data byte
        }
    }

    if (n == 0) {
        logd("Simple HOG: no Report IDs in descriptor — parse raw\n");
        return;
    }

    for (i = 0; i < hog_notify_count; i++) {
        hog_notify_report_ids[i] = (i < n) ? ids[i] : ids[n - 1];
        logd("Simple HOG: notify[%d] vh=0x%04x report_id=%u\n", i, hog_notify_chars[i].value_handle,
             hog_notify_report_ids[i]);
    }
}

static void hog_notification_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    ARG_UNUSED(channel);
    ARG_UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;
    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION)
        return;

    hci_con_handle_t con_handle = gatt_event_notification_get_handle(packet);
    uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (!device)
        return;

    const uint8_t* report = gatt_event_notification_get_value(packet);
    uint16_t report_len = gatt_event_notification_get_value_length(packet);
    if (report_len == 0)
        return;

    uint16_t value_handle = gatt_event_notification_get_value_handle(packet);
    uint8_t report_id = 0;
    int idx = -1;
    int i;
    for (i = 0; i < hog_notify_count; i++) {
        if (hog_notify_chars[i].value_handle == value_handle) {
            report_id = hog_notify_report_ids[i];
            idx = i;
            break;
        }
    }

    // HOGP notifications omit Report ID; HID parser needs it when the map has 0x85 items.
    uint8_t buf[128];
    const uint8_t* parse_ptr = report;
    uint16_t parse_len = report_len;

    if (report_id != 0 && report_len + 1 <= sizeof(buf)) {
        buf[0] = report_id;
        memcpy(buf + 1, report, report_len);
        parse_ptr = buf;
        parse_len = (uint16_t)(report_len + 1);
    }

    uni_hid_parse_input_report(device, parse_ptr, parse_len);

    // Fallback: synthetic report id if still no activity.
    if (device->controller.klass == UNI_CONTROLLER_CLASS_GAMEPAD && device->controller.gamepad.buttons == 0 &&
        device->controller.gamepad.dpad == 0 && device->controller.gamepad.axis_y == 0 &&
        device->controller.gamepad.axis_ry == 0 && report_id == 0 && report_len + 1 <= sizeof(buf)) {
        buf[0] = (idx >= 0) ? (uint8_t)(idx + 1) : 1;
        memcpy(buf + 1, report, report_len);
        uni_hid_parse_input_report(device, buf, (uint16_t)(report_len + 1));
    }

    uni_hid_device_process_controller(device);
}

static void hog_ccc_timeout_handler(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    if (hog_state != HOG_STATE_W4_CCC)
        return;
    logd("Simple HOG: CCC[%d] timed out — skip\n", hog_ccc_index);
    hog_ccc_index++;
    hog_defer_step(hog_step_enable_ccc, 50);
}

static void hog_enable_next_ccc(void) {
    uint8_t status;
    uint16_t ccc_handle;

    if (hog_con_handle == HCI_CON_HANDLE_INVALID)
        return;

    btstack_run_loop_remove_timer(&hog_ccc_timeout_timer);

    if (hog_ccc_index >= hog_notify_count) {
        hog_finish_ready(hog_con_handle);
        return;
    }

    if (!gatt_client_is_ready(hog_con_handle)) {
        logd("Simple HOG: GATT not ready for CCC[%d], retry\n", hog_ccc_index);
        hog_defer_step(hog_step_enable_ccc, 50);
        return;
    }

    // HID Report CCC is almost always value_handle+1.
    ccc_handle = (uint16_t)(hog_notify_chars[hog_ccc_index].value_handle + 1);
    logd("Simple HOG: CCC[%d] value=0x%04x ccc=0x%04x\n", hog_ccc_index, hog_notify_chars[hog_ccc_index].value_handle,
         ccc_handle);

    gatt_client_listen_for_characteristic_value_updates(&hog_notify_listeners[hog_ccc_index], hog_notification_handler,
                                                        hog_con_handle, &hog_notify_chars[hog_ccc_index]);

    status = gatt_client_write_characteristic_descriptor_using_descriptor_handle(
        hog_gatt_handler, hog_con_handle, ccc_handle, sizeof(hog_ccc_enable_value), hog_ccc_enable_value);
    if (status != ERROR_CODE_SUCCESS) {
        logd("Simple HOG: CCC write failed 0x%02x — skip[%d]\n", status, hog_ccc_index);
        hog_ccc_index++;
        hog_defer_step(hog_step_enable_ccc, 50);
        return;
    }
    hog_state = HOG_STATE_W4_CCC;
    btstack_run_loop_set_timer_handler(&hog_ccc_timeout_timer, &hog_ccc_timeout_handler);
    btstack_run_loop_set_timer(&hog_ccc_timeout_timer, HOG_CCC_TIMEOUT_MS);
    btstack_run_loop_add_timer(&hog_ccc_timeout_timer);
}

static void hog_start_char_discovery(hci_con_handle_t con_handle) {
    gatt_client_service_t service;
    uint8_t status;

    if (!hog_found_hid || hog_hid_start == 0) {
        loge("Simple HOG: no HID handles to discover\n");
        hog_disconnect(con_handle);
        return;
    }

    if (!gatt_client_is_ready(con_handle)) {
        logd("Simple HOG: GATT not ready for char discovery, reschedule\n");
        schedule_hog_setup(con_handle);
        return;
    }

    hog_con_handle = con_handle;
    hog_notify_count = 0;
    hog_ccc_index = 0;
    hog_have_report_map = false;
    memset(&hog_report_map_char, 0, sizeof(hog_report_map_char));

    service.start_group_handle = hog_hid_start;
    service.end_group_handle = hog_hid_end;

    logd("Simple HOG: discover chars HID 0x%04x-0x%04x\n", hog_hid_start, hog_hid_end);
    status = gatt_client_discover_characteristics_for_service(hog_gatt_handler, con_handle, &service);
    if (status != ERROR_CODE_SUCCESS) {
        loge("Simple HOG: char discovery failed 0x%02x\n", status);
        hog_disconnect(con_handle);
        return;
    }
    hog_state = HOG_STATE_W4_CHARS;
}

static void hog_after_probe_timer_handler(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);
    hci_con_handle_t con_handle = hog_after_probe_handle;
    uni_hid_device_t* device;

    hog_after_probe_handle = HCI_CON_HANDLE_INVALID;
    if (con_handle == HCI_CON_HANDLE_INVALID)
        return;

    device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (!device) {
        loge("Simple HOG after probe: no device\n");
        return;
    }
    apply_known_vid_pid(device);

    if (!hog_found_hid) {
        loge("No HID service (0x1812). GamePadPlus V3: use Home+X pairing mode.\n");
        gap_delete_bonding(BD_ADDR_TYPE_LE_PUBLIC, device->conn.btaddr);
        gap_delete_bonding(BD_ADDR_TYPE_LE_RANDOM, device->conn.btaddr);
        hog_disconnect(con_handle);
        return;
    }

    hog_start_char_discovery(con_handle);
}

static void hog_start_after_probe(hci_con_handle_t con_handle) {
    btstack_run_loop_remove_timer(&hog_after_probe_timer);
    hog_after_probe_handle = con_handle;
    btstack_run_loop_set_timer_handler(&hog_after_probe_timer, &hog_after_probe_timer_handler);
    btstack_run_loop_set_timer(&hog_after_probe_timer, HOG_POST_PROBE_DELAY_MS);
    btstack_run_loop_add_timer(&hog_after_probe_timer);
    logd("Simple HOG: HID found, char discovery in %dms\n", HOG_POST_PROBE_DELAY_MS);
}

static void hog_gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    uint8_t event = hci_event_packet_get_type(packet);
    hci_con_handle_t con_handle;
    uint8_t att_status;

    switch (event) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_client_service_t service;
            gatt_event_service_query_result_get_service(packet, &service);
            hog_service_count++;
            if (service.uuid16 != 0) {
                logd("GATT service[%d]: uuid16=0x%04x handles 0x%04x-0x%04x\n", hog_service_count, service.uuid16,
                     service.start_group_handle, service.end_group_handle);
                if (service.uuid16 == ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE || service.uuid16 == 0x1812) {
                    hog_found_hid = true;
                    hog_hid_start = service.start_group_handle;
                    hog_hid_end = service.end_group_handle;
                    logd("  -> HID 0x1812 handles 0x%04x-0x%04x\n", hog_hid_start, hog_hid_end);
                }
            } else {
                logd("GATT service[%d]: uuid128 handles 0x%04x-0x%04x\n", hog_service_count, service.start_group_handle,
                     service.end_group_handle);
            }
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t characteristic;
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            logd("  HID char uuid=0x%04x value=0x%04x props=0x%02x\n", characteristic.uuid16,
                 characteristic.value_handle, characteristic.properties);

            if (characteristic.uuid16 == ORG_BLUETOOTH_CHARACTERISTIC_REPORT_MAP) {
                hog_report_map_char = characteristic;
                hog_have_report_map = true;
            } else if (characteristic.uuid16 == ORG_BLUETOOTH_CHARACTERISTIC_REPORT &&
                       (characteristic.properties & ATT_PROPERTY_NOTIFY) != 0) {
                if (hog_notify_count < HOG_MAX_NOTIFY_REPORTS) {
                    hog_notify_chars[hog_notify_count] = characteristic;
                    hog_notify_report_ids[hog_notify_count] = 0;
                    hog_notify_count++;
                }
            }
            break;
        }

        case GATT_EVENT_LONG_CHARACTERISTIC_VALUE_QUERY_RESULT: {
            if (hog_state != HOG_STATE_W4_REPORT_MAP)
                break;
            con_handle = gatt_event_long_characteristic_value_query_result_get_handle(packet);
            const uint8_t* value = gatt_event_long_characteristic_value_query_result_get_value(packet);
            uint16_t value_len = gatt_event_long_characteristic_value_query_result_get_value_length(packet);
            uint16_t offset = gatt_event_long_characteristic_value_query_result_get_value_offset(packet);
            uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(con_handle);
            logd("Simple HOG: Report Map chunk offset=%u len=%u\n", offset, value_len);
            if (device && value_len > 0) {
                if (offset == 0) {
                    uni_hid_device_set_hid_descriptor(device, value, value_len);
                } else if (device->hid_descriptor_len + value_len <= HID_MAX_DESCRIPTOR_LEN) {
                    memcpy(device->hid_descriptor + device->hid_descriptor_len, value, value_len);
                    device->hid_descriptor_len += value_len;
                }
                hog_report_map_got_value = true;
            }
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT: {
            if (hog_state != HOG_STATE_W4_REPORT_MAP)
                break;
            con_handle = gatt_event_characteristic_value_query_result_get_handle(packet);
            const uint8_t* value = gatt_event_characteristic_value_query_result_get_value(packet);
            uint16_t value_len = gatt_event_characteristic_value_query_result_get_value_length(packet);
            uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(con_handle);
            logd("Simple HOG: Report Map short len=%u\n", value_len);
            if (device && value_len > 0) {
                uni_hid_device_set_hid_descriptor(device, value, value_len);
                hog_report_map_got_value = true;
            }
            break;
        }

        case GATT_EVENT_QUERY_COMPLETE:
            con_handle = gatt_event_query_complete_get_handle(packet);
            att_status = gatt_event_query_complete_get_att_status(packet);

            if (hog_state == HOG_STATE_PROBE_SERVICES) {
                logd("GATT probe done: %d services hid=%d\n", hog_service_count, hog_found_hid ? 1 : 0);
                hog_start_after_probe(con_handle);
                break;
            }

            if (hog_state == HOG_STATE_W4_CHARS) {
                logd("Simple HOG: chars done status=0x%02x map=%d notify=%d\n", att_status, hog_have_report_map ? 1 : 0,
                     hog_notify_count);
                if (att_status != ATT_ERROR_SUCCESS) {
                    loge("Simple HOG: char discovery failed\n");
                    hog_disconnect(con_handle);
                    break;
                }
                if (hog_have_report_map) {
                    hog_defer_step(hog_step_read_map, 50);
                } else if (hog_notify_count > 0) {
                    hog_ccc_index = 0;
                    hog_defer_step(hog_step_enable_ccc, 50);
                } else {
                    loge("Simple HOG: no HID reports found\n");
                    hog_disconnect(con_handle);
                }
                break;
            }

            if (hog_state == HOG_STATE_W4_REPORT_MAP) {
                btstack_run_loop_remove_timer(&hog_report_map_timeout_timer);
                logd("Simple HOG: Report Map done status=0x%02x\n", att_status);
                if (!hog_report_map_got_value)
                    hog_apply_fallback_descriptor(con_handle);
                if (hog_notify_count == 0) {
                    hog_finish_ready(con_handle);
                } else {
                    hog_ccc_index = 0;
                    hog_defer_step(hog_step_enable_ccc, 50);
                }
                break;
            }

            if (hog_state == HOG_STATE_W4_CCC) {
                btstack_run_loop_remove_timer(&hog_ccc_timeout_timer);
                logd("Simple HOG: CCC[%d] done status=0x%02x\n", hog_ccc_index, att_status);
                hog_ccc_index++;
                hog_defer_step(hog_step_enable_ccc, 50);
                break;
            }
            break;

        default:
            break;
    }
}

static bool try_start_gatt_probe(hci_con_handle_t con_handle) {
    uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (!device) {
        loge("try_start_gatt_probe: no device for con_handle=%#x\n", con_handle);
        return true;
    }

    if (!gatt_client_is_ready(con_handle)) {
        logd("GATT probe deferred (retry=%d)\n", hog_setup_retries);
        return false;
    }

    if (hog_state != HOG_STATE_IDLE && hog_state != HOG_STATE_PROBE_SERVICES) {
        logd("Simple HOG already in progress (state=%d)\n", hog_state);
        return true;
    }

    hog_reset_session();
    apply_known_vid_pid(device);
    hog_con_handle = con_handle;
    hog_state = HOG_STATE_PROBE_SERVICES;

    logd("Probing GATT services, con_handle=%#x\n", con_handle);
    uint8_t status = gatt_client_discover_primary_services(hog_gatt_handler, con_handle);
    if (status != ERROR_CODE_SUCCESS) {
        loge("GATT probe start failed status=0x%02x\n", status);
        hog_state = HOG_STATE_IDLE;
        return false;
    }
    return true;
}

static void hog_setup_timer_handler(btstack_timer_source_t* ts) {
    ARG_UNUSED(ts);

    if (hog_pending_con_handle == HCI_CON_HANDLE_INVALID)
        return;

    uni_hid_device_t* device = uni_hid_device_get_instance_for_connection_handle(hog_pending_con_handle);
    if (device && hog_simple_active && hog_state == HOG_STATE_READY) {
        hog_pending_con_handle = HCI_CON_HANDLE_INVALID;
        hog_setup_retries = 0;
        return;
    }

    if (try_start_gatt_probe(hog_pending_con_handle)) {
        hog_pending_con_handle = HCI_CON_HANDLE_INVALID;
        hog_setup_retries = 0;
        return;
    }

    hog_setup_retries++;
    if (hog_setup_retries >= HOG_SETUP_MAX_RETRIES) {
        loge("Simple HOG setup gave up after %d retries\n", hog_setup_retries);
        hog_disconnect(hog_pending_con_handle);
        hog_pending_con_handle = HCI_CON_HANDLE_INVALID;
        hog_setup_retries = 0;
        return;
    }

    btstack_run_loop_set_timer(&hog_setup_timer, HOG_SETUP_DELAY_MS);
    btstack_run_loop_add_timer(&hog_setup_timer);
}

static void schedule_hog_setup(hci_con_handle_t con_handle) {
    btstack_run_loop_remove_timer(&hog_setup_timer);

    hog_pending_con_handle = con_handle;
    hog_setup_retries = 0;
    btstack_run_loop_set_timer_handler(&hog_setup_timer, &hog_setup_timer_handler);
    btstack_run_loop_set_timer(&hog_setup_timer, HOG_SETUP_DELAY_MS);
    btstack_run_loop_add_timer(&hog_setup_timer);
    logd("Scheduled HOG setup (delay %dms)\n", HOG_SETUP_DELAY_MS);
}

static void hog_disconnect(hci_con_handle_t con_handle) {
    // MUST not call uni_hid_device_disconnect(), called from it.
    uni_hid_device_t* device;

    if (hog_pending_con_handle == con_handle) {
        btstack_run_loop_remove_timer(&hog_setup_timer);
        hog_pending_con_handle = HCI_CON_HANDLE_INVALID;
        hog_setup_retries = 0;
    }
    if (hog_after_probe_handle == con_handle) {
        btstack_run_loop_remove_timer(&hog_after_probe_timer);
        hog_after_probe_handle = HCI_CON_HANDLE_INVALID;
    }

    btstack_run_loop_remove_timer(&hog_step_timer);

    if (hog_con_handle == con_handle || hog_simple_active) {
        hog_reset_session();
    }

    device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (device) {
        // Only call hids_client_disconnect if legacy path was used
        if (device->hids_cid != 0 && device->hids_cid != 0xffff && device->hids_cid != HOG_SIMPLE_CID) {
            uint8_t status = hids_client_disconnect(device->hids_cid);
            if (status != ERROR_CODE_SUCCESS) {
                loge("Failed to disconnect HIDS client for hids_cid=%d, status=%d\n", device->hids_cid, status);
            }
        }
        device->hids_cid = 0xffff;
    }

    if (gap_get_connection_type(con_handle) != GAP_CONNECTION_INVALID)
        gap_disconnect(con_handle);

    resume_scanning_hint();
}

static void get_advertisement_data(const uint8_t* adv_data, uint8_t adv_size, uint16_t* appearance, char* name) {
    ad_context_t context;

    for (ad_iterator_init(&context, adv_size, (uint8_t*)adv_data); ad_iterator_has_more(&context);
         ad_iterator_next(&context)) {
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t size = ad_iterator_get_data_len(&context);
        const uint8_t* data = ad_iterator_get_data(&context);

        int i;
        // Assigned Numbers GAP

        switch (data_type) {
            case BLUETOOTH_DATA_TYPE_FLAGS:
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_16_BIT_SERVICE_SOLICITATION_UUIDS:
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_32_BIT_SERVICE_SOLICITATION_UUIDS:
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_128_BIT_SERVICE_SOLICITATION_UUIDS:
                break;
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                for (i = 0; i < size; i++) {
                    name[i] = data[i];
                }
                name[size] = 0;
                break;
            case BLUETOOTH_DATA_TYPE_TX_POWER_LEVEL:
                break;
            case BLUETOOTH_DATA_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE:
                break;
            case BLUETOOTH_DATA_TYPE_SERVICE_DATA:
                break;
            case BLUETOOTH_DATA_TYPE_PUBLIC_TARGET_ADDRESS:
            case BLUETOOTH_DATA_TYPE_RANDOM_TARGET_ADDRESS:
                break;
            case BLUETOOTH_DATA_TYPE_APPEARANCE:
                // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
                *appearance = little_endian_read_16(data, 0);
                break;
            case BLUETOOTH_DATA_TYPE_ADVERTISING_INTERVAL:
                break;
            case BLUETOOTH_DATA_TYPE_3D_INFORMATION_DATA:
                break;
            case BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA:  // Manufacturer Specific Data
                break;
            case BLUETOOTH_DATA_TYPE_CLASS_OF_DEVICE:
                logi("class of device: %#x\n", little_endian_read_16(data, 0));
                break;
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_HASH_C:
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_RANDOMIZER_R:
            case BLUETOOTH_DATA_TYPE_DEVICE_ID:
                logi("device id: %#x\n", little_endian_read_16(data, 0));
                break;
            case BLUETOOTH_DATA_TYPE_LE_BLUETOOTH_DEVICE_ADDRESS:
            case BLUETOOTH_DATA_TYPE_MESH_BEACON:
            case BLUETOOTH_DATA_TYPE_MESH_MESSAGE:
                // Safely ignore these messages
                break;
            case BLUETOOTH_DATA_TYPE_SECURITY_MANAGER_OUT_OF_BAND_FLAGS:
                // fall-through
            default:
                logi("Advertising Data Type 0x%2x not handled yet\n", data_type);
                break;
        }
    }
}

static void adv_event_get_data(const uint8_t* packet, uint16_t* appearance, char* name) {
    const uint8_t* ad_data;
    uint16_t ad_len;

    ad_data = gap_event_advertising_report_get_data(packet);
    ad_len = gap_event_advertising_report_get_data_length(packet);

    // if (!ad_data_contains_uuid16(ad_len, ad_data, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE))
    get_advertisement_data(ad_data, ad_len, appearance, name);
}

static void parse_report(const uint8_t* packet, uint16_t size) {
    uint16_t service_index;
    uint16_t hids_cid;
    uni_hid_device_t* device;
    const uint8_t* descriptor_data;
    uint16_t descriptor_len;
    const uint8_t* report_data;
    uint16_t report_len;

    ARG_UNUSED(size);

    service_index = gattservice_subevent_hid_report_get_service_index(packet);
    hids_cid = gattservice_subevent_hid_report_get_hids_cid(packet);
    device = uni_hid_device_get_instance_for_hids_cid(hids_cid);

    if (!device) {
        loge("BLE parser report: Invalid device for hids_cid=%d\n", hids_cid);
        return;
    }

    // FIXME: Copying the HID descriptor should be done at setup time since some device, like Xbox requires it
    // to set the correct parser.
    // But not clear how to get the "service_index" from setup
    if (device->hid_descriptor_len == 0) {
        descriptor_data = hids_client_descriptor_storage_get_descriptor_data(hids_cid, service_index);
        descriptor_len = hids_client_descriptor_storage_get_descriptor_len(hids_cid, service_index);

        uni_hid_device_set_hid_descriptor(device, descriptor_data, descriptor_len);
    }
    report_data = gattservice_subevent_hid_report_get_report(packet);
    report_len = gattservice_subevent_hid_report_get_report_len(packet);

    uni_hid_parse_input_report(device, report_data, report_len);
    uni_hid_device_process_controller(device);
}

static void uni_hids_client_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    uint8_t status;
    uint16_t hids_cid;
    uni_hid_device_t* device;
    uint8_t event_type;
    hid_protocol_mode_t protocol_mode;

    ARG_UNUSED(packet_type);
    ARG_UNUSED(channel);
    ARG_UNUSED(size);

#if 0
    // FIXME: Bug in BTStack??? This comparison fails because packet_type is HCI_EVENT_GATTSERVICE_META
    if (packet_type != HCI_EVENT_PACKET) {
        loge("uni_hids_client_packet_handler: unsupported packet type: %#x\n", packet_type);
        return;
    }
#endif

    event_type = hci_event_packet_get_type(packet);
    if (event_type != HCI_EVENT_GATTSERVICE_META) {
        loge("uni_hids_client_packet_handler: unsupported event type: %#x\n", event_type);
        return;
    }

    switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
        case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED:
            status = gattservice_subevent_hid_service_connected_get_status(packet);
            logi("GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED, status=0x%02x\n", status);
            switch (status) {
                case ERROR_CODE_SUCCESS:
                    protocol_mode = gattservice_subevent_hid_service_connected_get_protocol_mode(packet);
                    logi("HID service client connected, found %d services, protocol_mode=%d\n",
                         gattservice_subevent_hid_service_connected_get_num_instances(packet), protocol_mode);

                    // XXX TODO: store device as bonded
                    hids_cid = gattservice_subevent_hid_service_connected_get_hids_cid(packet);
                    device = uni_hid_device_get_instance_for_hids_cid(hids_cid);
                    if (!device) {
                        loge("Hids Cid: Could not find valid device for hids_cid=%d\n", hids_cid);
                        break;
                    }
#if 0
                    status = hids_client_enable_notifications(hids_cid);
                    if (status != ERROR_CODE_SUCCESS)
                        logi("Failed to enable client notifications for hids_cid=%d, status=%#x\n", hids_cid, status);
                    else
                        logi("Client notifications enabled for for hids_cid=%d\n", hids_cid);
#endif

                    uni_hid_device_guess_controller_type_from_pid_vid(device);
                    uni_hid_device_connect(device);
                    uni_hid_device_set_ready(device);

                    // Optional: fill VID/PID after HID is ready (non-blocking for input).
                    if (device->vendor_id == 0 || device->product_id == 0) {
                        uint8_t di_status = device_information_service_client_query(
                            device->conn.handle, uni_device_information_packet_handler);
                        if (di_status != ERROR_CODE_SUCCESS) {
                            logi("Post-HID DIS query skipped, status=%#x\n", di_status);
                        }
                    }

                    resume_scanning_hint();
                    break;
                default:
                    loge("HID service client connection failed, err 0x%02x.\n", status);
                    hids_cid = gattservice_subevent_hid_service_connected_get_hids_cid(packet);
                    device = uni_hid_device_get_instance_for_hids_cid(hids_cid);
                    if (device) {
                        // Stale bonding often leaves HIDS discovery stuck — force re-pair next time.
                        gap_delete_bonding(BD_ADDR_TYPE_LE_PUBLIC, device->conn.btaddr);
                        gap_delete_bonding(BD_ADDR_TYPE_LE_RANDOM, device->conn.btaddr);
                        hog_disconnect(device->conn.handle);
                    }
                    break;
            }
            break;

        case GATTSERVICE_SUBEVENT_HID_REPORT:
            logd("GATTSERVICE_SUBEVENT_HID_REPORT\n");
            parse_report(packet, size);
            break;
        case GATTSERVICE_SUBEVENT_HID_INFORMATION:
            logi(
                "Hid Information: service index %d, USB HID 0x%02X, country code %d, remote wake %d, normally "
                "connectable %d\n",
                gattservice_subevent_hid_information_get_service_index(packet),
                gattservice_subevent_hid_information_get_base_usb_hid_version(packet),
                gattservice_subevent_hid_information_get_country_code(packet),
                gattservice_subevent_hid_information_get_remote_wake(packet),
                gattservice_subevent_hid_information_get_normally_connectable(packet));
            break;

        case GATTSERVICE_SUBEVENT_HID_PROTOCOL_MODE:
            logi("Protocol Mode: service index %d, mode 0x%02X (Boot mode: 0x%02X, Report mode 0x%02X)\n",
                 gattservice_subevent_hid_protocol_mode_get_service_index(packet),
                 gattservice_subevent_hid_protocol_mode_get_protocol_mode(packet), HID_PROTOCOL_MODE_BOOT,
                 HID_PROTOCOL_MODE_REPORT);
            break;

        case GATTSERVICE_SUBEVENT_HID_SERVICE_REPORTS_NOTIFICATION:
            if (gattservice_subevent_hid_service_reports_notification_get_configuration(packet) == 0) {
                logi("Reports disabled\n");
            } else {
                logi("Reports enabled\n");
            }
            break;
#if 0  // Does not compile on Pico SDK 1.5.1. Enable it only if needed.
        case GATTSERVICE_SUBEVENT_HID_REPORT_WRITTEN:
            // Called when a client a hid report was written.
            // E.g.: "set rumble" was sent to the gamepad.
            // TODO: Inform the device that it is ready to write another hid report?
            break;
#endif
        default:
            logi("Unsupported gatt client event: 0x%02x\n", hci_event_gattservice_meta_get_subevent_code(packet));
            break;
    }
}

static void uni_device_information_packet_handler(uint8_t packet_type,
                                                  uint16_t channel,
                                                  uint8_t* packet,
                                                  uint16_t size) {
    uint8_t code;
    uint8_t status;
    uint8_t att_status;
    hci_con_handle_t con_handle;
    uni_hid_device_t* device;
    uint8_t event_type;

    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        loge("uni_device_information_packet_handler: unsupported packet type: %#x\n", packet_type);
        return;
    }

    event_type = hci_event_packet_get_type(packet);
    if (event_type != HCI_EVENT_GATTSERVICE_META) {
        loge("uni_device_information_packet_handler: unsupported event type: %#x\n", event_type);
        return;
    }

    code = hci_event_gattservice_meta_get_subevent_code(packet);
    switch (code) {
        case GATTSERVICE_SUBEVENT_SCAN_PARAMETERS_SERVICE_CONNECTED:
            logi("PnP ID: vendor source ID 0x%02X, vendor ID 0x%02X, product ID 0x%02X, product version 0x%02X\n",
                 gattservice_subevent_device_information_pnp_id_get_vendor_source_id(packet),
                 gattservice_subevent_device_information_pnp_id_get_vendor_id(packet),
                 gattservice_subevent_device_information_pnp_id_get_product_id(packet),
                 gattservice_subevent_device_information_pnp_id_get_product_version(packet));
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_DONE:
            status = gattservice_subevent_device_information_done_get_att_status(packet);
            con_handle = gattservice_subevent_device_information_done_get_con_handle(packet);
            switch (status) {
                case ERROR_CODE_SUCCESS:
                    logi("Device Information service found\n");
                    device = uni_hid_device_get_instance_for_connection_handle(con_handle);
                    if (!device) {
                        loge("Invalid device for in GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_DONE");
                        break;
                    }
                    // Post-HID DIS path only: do not start HID again from here.
                    if (device->hids_cid == 0 || device->hids_cid == 0xffff) {
                        schedule_hids_client_connect(con_handle);
                    }
                    break;
                default:
                    logi("Device Information service client connection failed, error=%#x.\n", status);
                    device = uni_hid_device_get_instance_for_connection_handle(con_handle);
                    if (device && (device->hids_cid == 0 || device->hids_cid == 0xffff)) {
                        schedule_hids_client_connect(con_handle);
                    }
                    break;
            }
            break;
        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_MANUFACTURER_NAME:
            att_status = gattservice_subevent_device_information_manufacturer_name_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("Manufacturer Name read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Manufacturer Name: %s\n",
                     gattservice_subevent_device_information_manufacturer_name_get_value(packet));
            }
            break;
        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_MODEL_NUMBER:
            att_status = gattservice_subevent_device_information_model_number_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("Model Number read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Model Number:     %s\n", gattservice_subevent_device_information_model_number_get_value(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_SERIAL_NUMBER:
            att_status = gattservice_subevent_device_information_serial_number_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("Serial Number read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Serial Number:    %s\n", gattservice_subevent_device_information_serial_number_get_value(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_HARDWARE_REVISION:
            att_status = gattservice_subevent_device_information_hardware_revision_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("Hardware Revision read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Hardware Revision: %s\n",
                     gattservice_subevent_device_information_hardware_revision_get_value(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_FIRMWARE_REVISION:
            att_status = gattservice_subevent_device_information_firmware_revision_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("Firmware Revision read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Firmware Revision: %s\n",
                     gattservice_subevent_device_information_firmware_revision_get_value(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_SOFTWARE_REVISION:
            att_status = gattservice_subevent_device_information_software_revision_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("Software Revision read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Software Revision: %s\n",
                     gattservice_subevent_device_information_software_revision_get_value(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_SYSTEM_ID:
            att_status = gattservice_subevent_device_information_system_id_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("System ID read failed, ATT Error 0x%02x\n", att_status);
            } else {
                uint32_t manufacturer_identifier_low =
                    gattservice_subevent_device_information_system_id_get_manufacturer_id_low(packet);
                uint8_t manufacturer_identifier_high =
                    gattservice_subevent_device_information_system_id_get_manufacturer_id_high(packet);

                logi("Manufacturer ID:  0x%02x%08x\n", manufacturer_identifier_high, manufacturer_identifier_low);
                logi("Organizationally Unique ID:  0x%06x\n",
                     gattservice_subevent_device_information_system_id_get_organizationally_unique_id(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_IEEE_REGULATORY_CERTIFICATION:
            att_status = gattservice_subevent_device_information_ieee_regulatory_certification_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("IEEE Regulatory Certification read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("value_a:          0x%04x\n",
                     gattservice_subevent_device_information_ieee_regulatory_certification_get_value_a(packet));
                logi("value_b:          0x%04x\n",
                     gattservice_subevent_device_information_ieee_regulatory_certification_get_value_b(packet));
            }
            break;

        case GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_PNP_ID:
            con_handle = gattservice_subevent_device_information_pnp_id_get_con_handle(packet);
            device = uni_hid_device_get_instance_for_connection_handle(con_handle);
            if (!device) {
                loge("Invalid device for in GATTSERVICE_SUBEVENT_DEVICE_INFORMATION_PNP_ID");
                break;
            }

            att_status = gattservice_subevent_device_information_pnp_id_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                logi("PNP ID read failed, ATT Error 0x%02x\n", att_status);
            } else {
                logi("Vendor Source ID: 0x%02x\n",
                     gattservice_subevent_device_information_pnp_id_get_vendor_source_id(packet));
                logi("Vendor  ID:       0x%04x\n",
                     gattservice_subevent_device_information_pnp_id_get_vendor_id(packet));
                logi("Product ID:       0x%04x\n",
                     gattservice_subevent_device_information_pnp_id_get_product_id(packet));
                logi("Product Version:  0x%04x\n",
                     gattservice_subevent_device_information_pnp_id_get_product_version(packet));
            }
            uni_hid_device_set_vendor_id(device, gattservice_subevent_device_information_pnp_id_get_vendor_id(packet));
            uni_hid_device_set_product_id(device,
                                          gattservice_subevent_device_information_pnp_id_get_product_id(packet));

            break;

        default:
            logi("Unknown gattservice meta subevent code: %#x\n", code);
            break;
    }
}

/* HCI packet handler
 *
 * text The SM packet handler receives Security Manager Events required for
 * pairing. It also receives events generated during Identity Resolving see
 * Listing SMPacketHandler.
 */
static void uni_sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    bd_addr_t addr;
    uni_hid_device_t* device;
    uint8_t status;
    uint8_t type;
    hci_con_handle_t con_handle = UNI_BT_CONN_HANDLE_INVALID;
    bool request_device_information_query = false;

    ARG_UNUSED(channel);
    ARG_UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) {
        loge("uni_sm_packet_handler: unsupported packet type: %#x\n", packet_type);
        return;
    }

    type = hci_event_packet_get_type(packet);
    switch (type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            logd("Just works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            logd("Confirming numeric comparison: %" PRIu32 "\n",
                 sm_event_numeric_comparison_request_get_passkey(packet));
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            logd("Display Passkey: %" PRIu32 "\n", sm_event_passkey_display_number_get_passkey(packet));
            break;
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            logd("Identity resolving started\n");
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            sm_event_identity_created_get_address(packet, addr);
            logd("Identity resolving failed for %s\n", bd_addr_to_str(addr));
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
            sm_event_identity_resolving_succeeded_get_identity_address(packet, addr);
            logd("Identity resolved: %s\n", bd_addr_to_str(addr));
            break;
        case SM_EVENT_PAIRING_STARTED:
            logd("Pairing started\n");
            break;
        case SM_EVENT_IDENTITY_CREATED:
            sm_event_identity_created_get_identity_address(packet, addr);
            logd("Identity created: %s\n", bd_addr_to_str(addr));
            break;
        case SM_EVENT_REENCRYPTION_STARTED:
            sm_event_reencryption_complete_get_address(packet, addr);
            logd("Re-encryption started for %s\n", bd_addr_to_str(addr));
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            con_handle = sm_event_reencryption_complete_get_handle(packet);
            switch (sm_event_reencryption_complete_get_status(packet)) {
                case ERROR_CODE_SUCCESS:
                    logd("Re-encryption complete\n");
                    request_device_information_query = true;
                    break;
                case ERROR_CODE_CONNECTION_TIMEOUT:
                    loge("Re-encryption failed: timeout\n");
                    hog_disconnect(con_handle);
                    break;
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    loge("Re-encryption failed: disconnected\n");
                    hog_disconnect(con_handle);
                    break;
                case ERROR_CODE_PIN_OR_KEY_MISSING:
                    logi("Bonding missing — re-pairing\n");
                    sm_event_reencryption_complete_get_address(packet, addr);
                    type = sm_event_reencryption_started_get_addr_type(packet);
                    gap_delete_bonding(type, addr);
                    sm_request_pairing(sm_event_reencryption_complete_get_handle(packet));
                    break;
                default:
                    break;
            }
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            sm_event_pairing_complete_get_address(packet, addr);
            device = uni_hid_device_get_instance_for_address(addr);
            con_handle = sm_event_pairing_complete_get_handle(packet);
            if (!device) {
                loge("Pairing complete: invalid device %s\n", bd_addr_to_str(addr));
                hog_disconnect(con_handle);
                break;
            }

            status = sm_event_pairing_complete_get_status(packet);
            switch (status) {
                case ERROR_CODE_SUCCESS:
                    logd("Pairing complete\n");
                    request_device_information_query = true;
                    break;
                case ERROR_CODE_CONNECTION_TIMEOUT:
                    loge("Pairing failed: timeout\n");
                    break;
                case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                    loge("Pairing failed: disconnected\n");
                    break;
                case ERROR_CODE_AUTHENTICATION_FAILURE:
                    loge("Pairing failed: auth reason=%u\n", sm_event_pairing_complete_get_reason(packet));
                    break;
                default:
                    loge("Pairing failed: status=0x%02x\n", status);
                    break;
            }

            // TODO: Double check
            // Do not disconnect. Sometimes it appears as "failure" although
            // the connection as Ok (???)
            // hog_disconnect(device->conn.handle);
            break;

        default:
            loge("Unknown SM packet type: %#x\n", type);
            break;
    }

    if (request_device_information_query) {
        if (con_handle == UNI_BT_CONN_HANDLE_INVALID) {
            // Should not happen.
            loge("Error: Invalid conn_handle: %d\n", con_handle);
            return;
        }
        // Prefer HID discovery first. Cheap BLE pads (ShanWan/GamePadPlus) can stall
        // after a long Device Information Service walk, never finishing HID setup.
        // VID/PID are optional; unknown pads fall back to the Android HID parser.
        logd("Starting HOG setup after pairing\n");
        schedule_hog_setup(con_handle);

        // Optional background DIS for VID/PID logging only — do not block HID on it.
        // Re-enable if you need accurate VID/PID before HID and the pad supports it:
        // status = device_information_service_client_query(con_handle, uni_device_information_packet_handler);
        (void)status;
    }
}

void uni_bt_le_on_hci_event_le_meta(const uint8_t* packet, uint16_t size) {
    uni_hid_device_t* device;
    hci_con_handle_t con_handle;
    bd_addr_t event_addr;
    uint8_t subevent;

    ARG_UNUSED(size);

    subevent = hci_event_le_meta_get_subevent_code(packet);

    switch (subevent) {
        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
            hci_subevent_le_connection_complete_get_peer_address(packet, event_addr);
            device = uni_hid_device_get_instance_for_address(event_addr);
            if (!device) {
                loge("uni_bt_le_on_connection_complete: Device not found for addr: %s\n", bd_addr_to_str(event_addr));
                break;
            }
            con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            logd("Using con_handle: %#x\n", con_handle);

            uni_hid_device_set_connection_handle(device, con_handle);
            sm_request_pairing(con_handle);

            // Resume scanning
            // gap_start_scan();
            break;

        case HCI_SUBEVENT_LE_ADVERTISING_REPORT:
            // Safely ignore it, we handle the GAP advertising report instead
            break;

        default:
            logd("Unsupported LE_META sub-event: %#x\n", subevent);
            break;
    }
}

void uni_bt_le_on_hci_event_encryption_change(const uint8_t* packet, uint16_t size) {
    uni_hid_device_t* device;
    hci_con_handle_t con_handle;

    ARG_UNUSED(size);

    // Might be called from BR/EDR connections.
    // Only handle BLE in this function.
    con_handle = hci_event_encryption_change_get_connection_handle(packet);
    if (gap_get_connection_type(con_handle) != GAP_CONNECTION_LE)
        return;

    device = uni_hid_device_get_instance_for_connection_handle(con_handle);
    if (!device) {
        loge("uni_bt_le_on_encryption_change: Device not found for connection handle: 0x%04x\n", con_handle);
        return;
    }
    // This event is also triggered by Classic, and might crash the stack.
    // Real case: Connect a Wii, disconnect it, and try re-connection
    if (device->conn.protocol != UNI_BT_CONN_PROTOCOL_BLE)
        // Abort on non BLE connections
        return;

    logd("Connection encrypted: %u\n", hci_event_encryption_change_get_encryption_enabled(packet));
    if (hci_event_encryption_change_get_encryption_enabled(packet) == 0) {
        loge("Encryption failed — disconnect\n");
        hog_disconnect(con_handle);
    }
}

void uni_bt_le_on_gap_event_advertising_report(const uint8_t* packet, uint16_t size) {
    bd_addr_t addr;
    bd_addr_type_t addr_type;
    uint16_t appearance;
    uint16_t cod;
    uint8_t rssi;
    char name[64];

    appearance = 0;
    name[0] = 0;

    ARG_UNUSED(size);

    gap_event_advertising_report_get_address(packet, addr);
    if (uni_hid_device_get_instance_for_address(addr)) {
        // Ignore, address already found
        return;
    }

    adv_event_get_data(packet, &appearance, name);

    if (appearance != UNI_BT_HID_APPEARANCE_GAMEPAD && appearance != UNI_BT_HID_APPEARANCE_JOYSTICK &&
        appearance != UNI_BT_HID_APPEARANCE_MOUSE && appearance != UNI_BT_HID_APPEARANCE_KEYBOARD) {
        // Don't log it. There too many devices advertising themselves.
        if (appearance != 0 || strlen(name) != 0)
            logd("Not a HID controller, appearance: %#x, name =%s\n", appearance, name);
        return;
    }

    switch (appearance) {
        case UNI_BT_HID_APPEARANCE_MOUSE:
            cod = UNI_BT_COD_MAJOR_PERIPHERAL | UNI_BT_COD_MINOR_MICE;
            break;
        case UNI_BT_HID_APPEARANCE_JOYSTICK:
            cod = UNI_BT_COD_MAJOR_PERIPHERAL | UNI_BT_COD_MINOR_JOYSTICK;
            break;
        case UNI_BT_HID_APPEARANCE_GAMEPAD:
            cod = UNI_BT_COD_MAJOR_PERIPHERAL | UNI_BT_COD_MINOR_GAMEPAD;
            break;
        case UNI_BT_HID_APPEARANCE_KEYBOARD:
            cod = UNI_BT_COD_MAJOR_PERIPHERAL | UNI_BT_COD_MINOR_KEYBOARD;
            break;
        default:
            cod = 0;
            break;
    }

    addr_type = gap_event_advertising_report_get_address_type(packet);
    rssi = gap_event_advertising_report_get_rssi(packet);

    logd("Device found: %s name='%s' appearance=0x%x rssi=%u\n", bd_addr_to_str(addr), name, appearance, rssi);

    if (uni_hid_device_on_device_discovered(addr, name, cod, rssi) != UNI_ERROR_SUCCESS)
        return;

    uni_hid_device_t* d = uni_hid_device_create(addr);
    if (!d) {
        loge("Error: no more available device slots\n");
        return;
    }

    // FIXME: Using CODs to make it compatible with legacy BR/EDR code.
    uni_hid_device_set_cod(d, cod);
    uni_hid_device_set_name(d, name);
    uni_bt_conn_set_protocol(&d->conn, UNI_BT_CONN_PROTOCOL_BLE);
    uni_bt_conn_set_state(&d->conn, UNI_BT_CONN_STATE_DEVICE_DISCOVERED);
    d->conn.rssi = rssi;

    hog_connect(addr, addr_type);
}

void uni_bt_le_on_hci_disconnection_complete(uint16_t channel, const uint8_t* packet, uint16_t size) {
    ARG_UNUSED(channel);
    ARG_UNUSED(packet);
    ARG_UNUSED(size);

    resume_scanning_hint();
}

void uni_bt_le_list_bonded_keys(void) {
    bd_addr_t entry_address;
    int i;

    if (!ble_enabled)
        return;

    logi("Bluetooth LE keys:\n");

    for (i = 0; i < le_device_db_max_count(); i++) {
        int entry_address_type = (int)BD_ADDR_TYPE_UNKNOWN;
        le_device_db_info(i, &entry_address_type, entry_address, NULL);

        // skip unused entries
        if (entry_address_type == (int)BD_ADDR_TYPE_UNKNOWN)
            continue;

        logi("%s - type %u\n", bd_addr_to_str(entry_address), (int)entry_address_type);
    }
    logi(".\n");
}

void uni_bt_le_delete_bonded_keys(void) {
    bd_addr_t entry_address;
    int i;

    if (!ble_enabled)
        return;

    logi("Deleting stored BLE link keys:\n");

    for (i = 0; i < le_device_db_max_count(); i++) {
        int entry_address_type = (int)BD_ADDR_TYPE_UNKNOWN;
        le_device_db_info(i, &entry_address_type, entry_address, NULL);

        // skip unused entries
        if (entry_address_type == (int)BD_ADDR_TYPE_UNKNOWN)
            continue;

        logi("%s - type %u\n", bd_addr_to_str(entry_address), (int)entry_address_type);
        gap_delete_bonding((bd_addr_type_t)entry_address_type, entry_address);
    }
    logi(".\n");
}

void uni_bt_le_setup(void) {
    // register for events from Security Manager
    sm_event_callback_registration.callback = &uni_sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    // Setup LE device db
    le_device_db_init();

    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    // TL;DR:
    // Enable Secure connection, disable bonding

    // Legacy paring, Just Works in ESP32
    // - Stadia: Ok
    // - MS mouse: Ok
    // - Xbox 3 buttons: flaky, fails to connect or connects
    // - Xbox 2 buttons: flaky, fails to connect or connects
    // sm_set_authentication_requirements(0);

    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);

    // Secure connection + NO bonding in ESP32:
    // - Stadia: Ok
    // - MS mouse: Ok
    // - Xbox 3 buttons: Ok
    // - Xbox 2 buttons: fails to connect
    // sm_set_secure_connections_only_mode(true);
    // gap_set_secure_connections_only_mode(true);
    // sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION);

    // Secure connection + bonding in ESP32:
    // - Stadia: Ok
    // - MS mouse: Ok... but disconnects after 10 seconds
    // - Xbox 3 buttons: fails to connect
    // - Xbox 2 buttons: fails to connect
    // sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);

    // libusb works with mostly any configuration

    gatt_client_init();
    hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
    // FIXME: this is an empty function and PicoW toolchain is removing empty function (?)
    // scan_parameters_service_client_init();
    device_information_service_client_init();

    gap_set_scan_parameters(0 /* type: passive */, 48 /* interval */, 48 /* window */);
}

void uni_bt_le_scan_start(void) {
    if (!ble_enabled)
        return;

    gap_start_scan();
    logd("BLE scan -> 1\n");
    is_scanning = true;
}

void uni_bt_le_scan_stop(void) {
    if (!ble_enabled)
        return;

    gap_stop_scan();
    logd("BLE scan -> 0\n");
    is_scanning = false;
}

void uni_bt_le_disconnect(uni_hid_device_t* d) {
    // if (gap_get_connection_type(conn->handle) == GAP_CONNECTION_INVALID)
    //     return;
    hog_disconnect(d->conn.handle);
}

void uni_bt_le_set_enabled(bool enabled) {
    // Called from different Task. Don't call BTstack functions.
    uni_property_value_t val;

    val.u8 = enabled;
    uni_property_set(UNI_PROPERTY_IDX_BLE_ENABLED, val);

    ble_enabled = enabled;
}

bool uni_bt_le_is_enabled() {
    // Expensive call. Avoid calling it from this same file.
    // Called from "uni_bt_setup"
    uni_property_value_t val;

    val = uni_property_get(UNI_PROPERTY_IDX_BLE_ENABLED);

    ble_enabled = val.u8;

    return ble_enabled;
}
