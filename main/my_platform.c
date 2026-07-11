// Bluepad32 커스텀 플랫폼 - RC 탱크 게임패드 입력 처리
// panzer4-idf my_flatform.c 기반, ESP32-C3용

#include <string.h>
#include "uni.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char* TAG = "platform";

//
// 공유 상태: BT 콜백 ↔ 제어 태스크 간 데이터 교환
//
typedef struct {
    SemaphoreHandle_t mutex;

    int32_t axis_y;
    int32_t axis_ry;
    uint16_t buttons;
    uint8_t dpad;
    uint8_t misc_buttons;

    bool connected;
    bool new_connection;
} gamepad_state_t;

static gamepad_state_t g_gamepad = {
    .mutex = NULL,
    .connected = false,
    .new_connection = false,
};

typedef struct {
    uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

//
// 플랫폼 콜백
//
void gamepad_state_init(void) {
    g_gamepad.mutex = xSemaphoreCreateMutex();
    configASSERT(g_gamepad.mutex);
}

static void my_platform_init(int argc, const char** argv) {
    (void)argc;
    (void)argv;
    logd("platform: init()\n");
}

static void my_platform_on_init_complete(void) {
    logd("platform: on_init_complete()\n");
    // Do NOT delete bond keys on every boot. That forces re-pairing and can
    // leave cheap BLE pads (GamePadPlus V3 / ShanWan BM-769) stuck mid-connect.
    // To factory-reset bonds, call uni_bt_del_keys_unsafe() once from console
    // or a dedicated user action, not here.
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name,
                                                     uint16_t cod, uint8_t rssi) {
    (void)addr;
    (void)rssi;
    logd("platform: discovered: %s cod=0x%04x\n", name ? name : "?", cod);

    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        logd("platform: ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }
    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logd("platform: device connected\n");
    (void)d;
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logd("platform: device disconnected\n");
    (void)d;

    if (xSemaphoreTake(g_gamepad.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_gamepad.connected = false;
        g_gamepad.new_connection = false;
        g_gamepad.buttons = 0;
        g_gamepad.dpad = 0;
        g_gamepad.axis_y = 0;
        g_gamepad.axis_ry = 0;
        g_gamepad.misc_buttons = 0;
        xSemaphoreGive(g_gamepad.mutex);
    }
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logd("platform: device ready\n");

    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    if (xSemaphoreTake(g_gamepad.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // 연결 직후 잔여/노이즈 입력으로 터렛·모터가 움직이지 않도록 클리어
        g_gamepad.axis_y = 0;
        g_gamepad.axis_ry = 0;
        g_gamepad.buttons = 0;
        g_gamepad.dpad = 0;
        g_gamepad.misc_buttons = 0;
        g_gamepad.connected = true;
        g_gamepad.new_connection = true;
        xSemaphoreGive(g_gamepad.mutex);
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    const uni_gamepad_t* gp = &ctl->gamepad;

    if (xSemaphoreTake(g_gamepad.mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
        g_gamepad.axis_y = gp->axis_y;
        g_gamepad.axis_ry = gp->axis_ry;
        g_gamepad.buttons = gp->buttons;
        g_gamepad.dpad = gp->dpad;
        g_gamepad.misc_buttons = gp->misc_buttons;
        xSemaphoreGive(g_gamepad.mutex);
    }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    (void)idx;
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
            uni_hid_device_t* d = data;
            if (d == NULL) {
                loge("platform: invalid NULL device in oob_event\n");
                return;
            }
            my_platform_instance_t* ins = get_my_platform_instance(d);
            ins->gamepad_seat = (ins->gamepad_seat == GAMEPAD_SEAT_A)
                                    ? GAMEPAD_SEAT_B
                                    : GAMEPAD_SEAT_A;
            break;
        }
        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logd("platform: bluetooth enabled: %d\n", (bool)(data));
            break;
        default:
            logd("platform: oob_event: %d\n", event);
            break;
    }
}

struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "panzer2",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };
    return &plat;
}

//
// 제어 태스크에서 호출하는 상태 조회 함수
//
bool gamepad_is_connected(void) {
    bool ret = false;
    if (xSemaphoreTake(g_gamepad.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ret = g_gamepad.connected;
        xSemaphoreGive(g_gamepad.mutex);
    }
    return ret;
}

bool gamepad_read_new_connection(void) {
    bool ret = false;
    if (xSemaphoreTake(g_gamepad.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ret = g_gamepad.new_connection;
        g_gamepad.new_connection = false;
        xSemaphoreGive(g_gamepad.mutex);
    }
    return ret;
}

bool gamepad_read(int32_t* axis_y, int32_t* axis_ry, uint16_t* buttons,
                  uint8_t* dpad, uint8_t* misc_buttons) {
    bool ret = false;
    if (xSemaphoreTake(g_gamepad.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (g_gamepad.connected) {
            *axis_y = g_gamepad.axis_y;
            *axis_ry = g_gamepad.axis_ry;
            *buttons = g_gamepad.buttons;
            *dpad = g_gamepad.dpad;
            *misc_buttons = g_gamepad.misc_buttons;
            ret = true;
        }
        xSemaphoreGive(g_gamepad.mutex);
    }
    return ret;
}
