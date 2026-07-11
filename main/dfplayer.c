/**
 * @file dfplayer.c
 * @brief DFPlayer Mini UART 제어 (panzer4-idf rctank_dfplayer 기반)
 */
#include "dfplayer.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "dfplayer";

/* 핀 설정 (ESP32-C3)
 * 주의: GPIO20 = 기본 UART0 RX (콘솔). DFPlayer RX로 쓰면 시리얼 로그가 끊길 수 있음.
 * GPIO21 = 기본 UART0 TX. 둘 다 피한다.
 */
#define PIN_DFPLAYER_TX  10
#define PIN_DFPLAYER_RX  9

#define UART_NUM         UART_NUM_1
#define UART_BAUD        9600
#define UART_BUF_SIZE    256

/* DFPlayer 패킷 상수 */
#define DFPLAYER_SB          0x7E
#define DFPLAYER_VER         0xFF
#define DFPLAYER_LEN         0x06
#define DFPLAYER_NO_FEEDBACK 0x00
#define DFPLAYER_EB          0xEF

/* 명령 코드 */
#define DFPLAYER_CMD_PLAY           0x03
#define DFPLAYER_CMD_VOLUME         0x06
#define DFPLAYER_CMD_PLAYBACK_MODE  0x08
#define DFPLAYER_CMD_STOP           0x16

#define DFPLAYER_STACK_SIZE 10

typedef struct {
    uint8_t start_byte;
    uint8_t version;
    uint8_t length;
    uint8_t command_value;
    uint8_t feedback_value;
    uint8_t param_msb;
    uint8_t param_lsb;
    uint8_t checksum_msb;
    uint8_t checksum_lsb;
    uint8_t end_byte;
} dfplayer_stack_t;

static void dfplayer_find_checksum(dfplayer_stack_t* stack) {
    uint16_t sum = (uint16_t)(stack->version + stack->length + stack->command_value
                              + stack->feedback_value + stack->param_msb + stack->param_lsb);
    uint16_t checksum = (uint16_t)((~sum) + 1);
    stack->checksum_msb = (uint8_t)(checksum >> 8);
    stack->checksum_lsb = (uint8_t)(checksum & 0xFF);
}

static esp_err_t dfplayer_send_stack(const dfplayer_stack_t* stack) {
    uint8_t buf[DFPLAYER_STACK_SIZE] = {
        stack->start_byte, stack->version, stack->length,
        stack->command_value, stack->feedback_value,
        stack->param_msb, stack->param_lsb,
        stack->checksum_msb, stack->checksum_lsb, stack->end_byte,
    };
    int n = uart_write_bytes(UART_NUM, buf, DFPLAYER_STACK_SIZE);
    return (n == DFPLAYER_STACK_SIZE) ? ESP_OK : ESP_FAIL;
}

static esp_err_t dfplayer_send_cmd(uint8_t cmd, uint8_t param_msb, uint8_t param_lsb) {
    dfplayer_stack_t stack = {
        .start_byte = DFPLAYER_SB,
        .version = DFPLAYER_VER,
        .length = DFPLAYER_LEN,
        .command_value = cmd,
        .feedback_value = DFPLAYER_NO_FEEDBACK,
        .param_msb = param_msb,
        .param_lsb = param_lsb,
        .end_byte = DFPLAYER_EB,
    };
    dfplayer_find_checksum(&stack);
    return dfplayer_send_stack(&stack);
}

static bool dfplayer_verify_checksum(const dfplayer_stack_t* stack) {
    uint16_t sum = (uint16_t)(stack->version + stack->length + stack->command_value
                              + stack->feedback_value + stack->param_msb + stack->param_lsb);
    uint16_t checksum = (uint16_t)((~sum) + 1);
    return (stack->checksum_msb == (uint8_t)(checksum >> 8)) &&
           (stack->checksum_lsb == (uint8_t)(checksum & 0xFF));
}

/**
 * RX 태스크: DFPlayer 응답 수신, 트랙 완료 시 대기음 재생
 */
static void dfplayer_task(void* arg) {
    (void)arg;
    uint8_t data[128];
    uint8_t pkt_buf[DFPLAYER_STACK_SIZE];
    int pkt_idx = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), pdMS_TO_TICKS(50));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t b = data[i];
                if (pkt_idx == 0) {
                    if (b == DFPLAYER_SB) {
                        pkt_buf[pkt_idx++] = b;
                    }
                } else {
                    pkt_buf[pkt_idx++] = b;
                    if (pkt_idx >= DFPLAYER_STACK_SIZE) {
                        if (pkt_buf[9] == DFPLAYER_EB) {
                            dfplayer_stack_t* s = (dfplayer_stack_t*)pkt_buf;
                            if (dfplayer_verify_checksum(s)) {
                                /* 0x3D = Track Finished */
                                if (s->command_value == 0x3D) {
                                    dfplayer_play_loop(DFPLAYER_TRACK_IDLE);
                                }
                            }
                        }
                        pkt_idx = 0;
                    }
                }
            }
        }
    }
}

esp_err_t dfplayer_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) return ret;
    ret = uart_set_pin(UART_NUM, PIN_DFPLAYER_TX, PIN_DFPLAYER_RX, -1, -1);
    if (ret != ESP_OK) return ret;
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(200));

    xTaskCreate(dfplayer_task, "dfplayer_rx", 2048, NULL, 5, NULL);
    ESP_LOGD(TAG, "DFPlayer 초기화 완료");
    return ESP_OK;
}

esp_err_t dfplayer_play(uint8_t track) {
    if (track < 1) return ESP_ERR_INVALID_ARG;
    return dfplayer_send_cmd(DFPLAYER_CMD_PLAY, 0, track);
}

esp_err_t dfplayer_play_loop(uint8_t track) {
    if (track < 1) return ESP_ERR_INVALID_ARG;
    return dfplayer_send_cmd(DFPLAYER_CMD_PLAYBACK_MODE, 0, track);
}

esp_err_t dfplayer_set_volume(uint8_t vol) {
    if (vol > 30) vol = 30;
    return dfplayer_send_cmd(DFPLAYER_CMD_VOLUME, 0, vol);
}

esp_err_t dfplayer_stop(void) {
    return dfplayer_send_cmd(DFPLAYER_CMD_STOP, 0, 0);
}
