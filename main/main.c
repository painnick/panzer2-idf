// RC 탱크 - ESP-IDF v5.5.2
// Original: M3Stuart_ESP32C3 (Arduino/PlatformIO)
//
// Bluepad32 게임패드로 DRV8833 모터 트랙 + SG90 서보 터렛 + DFPlayer 효과음 제어

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "btstack_port_esp32.h"
#include "dfplayer.h"
#include "btstack_run_loop.h"
#include "uni.h"

// my_platform.c
struct uni_platform* get_my_platform(void);
void gamepad_state_init(void);

static const char* TAG = "RC_TANK";

// ============================================================================
// 핀 정의
// ============================================================================
#define PIN_LEFT_IN1        4   // DRV8833 좌측 트랙 IN1
#define PIN_LEFT_IN2        3   // DRV8833 좌측 트랙 IN2
#define PIN_RIGHT_IN1       0   // DRV8833 우측 트랙 IN1
#define PIN_RIGHT_IN2       5   // DRV8833 우측 트랙 IN2
#define PIN_CANNON_LED      1   // 포신 LED
#define PIN_MG_LED          6   // 게틀링(기관총) LED
#define PIN_TURRET_SERVO    7   // 터렛 SG90 서보

// ============================================================================
// LEDC 채널/타이머 할당
// ============================================================================
#define LEDC_MOTOR_FREQ     5000
#define LEDC_MOTOR_RES      LEDC_TIMER_8_BIT
#define LEDC_SERVO_FREQ     50
#define LEDC_SERVO_RES      LEDC_TIMER_14_BIT

#define LEDC_CH_LEFT_IN1    LEDC_CHANNEL_0
#define LEDC_CH_LEFT_IN2    LEDC_CHANNEL_1
#define LEDC_CH_RIGHT_IN1   LEDC_CHANNEL_2
#define LEDC_CH_RIGHT_IN2   LEDC_CHANNEL_3
#define LEDC_CH_SERVO       LEDC_CHANNEL_4

#define LEDC_TIMER_MOTOR    LEDC_TIMER_0
#define LEDC_TIMER_SERVO    LEDC_TIMER_1

// ============================================================================
// NVS 키
// ============================================================================
#define NVS_NAMESPACE       "rc_tank"
#define NVS_KEY_VOLUME      "volume"

// ============================================================================
// 타이밍 상수 (ms)
// ============================================================================
#define LOOP_INTERVAL_MS        10
#define IDLE_SOUND_INTERVAL_MS  13000
#define VOLUME_CHANGE_INTERVAL  100
#define BUTTON_SWAP_HOLD_MS     3000
#define EEPROM_RESET_HOLD_MS    3000
#define CANNON_LED_DURATION     200
#define MACHINE_GUN_DURATION    500   // panzer4 MG_FIRE_MS
#define MG_LED_BLINK_MS         75    // panzer4 게틀링 LED 깜빡임 주기
#define TURRET_STEP_INTERVAL_MS 120   // 터렛 1° 이동 간격 (ms) — 아주 느리게
#define TURRET_IDLE_DISCONNECT_MS 3000 // 터렛 무입력 시 서보 연결 해제 (ms)
#define GAMEPAD_CONNECT_GRACE_MS  500 // 연결 직후 입력 무시 (노이즈/잔여 D-Pad 방지)
#define RECOIL_DELAY_MS         250   // LED·효과음 후 반동 시작 지연 (ms)
#define RECOIL_BACK_DURATION    40    // 포 발사 시 후진 시간 (ms)
#define RECOIL_SETTLE_DURATION  40    // 후진 후 정지 안정화 (ms)

// ============================================================================
// 모터 설정 (panzer4: 램프 가속/감속)
// ============================================================================
// 기동 최소 속도 (0~512). 너무 낮으면 고주파 윙 소리만 나고 바퀴가 안 돔.
// duty ≈ START * 255 / 512 → 400 ≈ 78%
#define MOTOR_START_SPEED   400
#define MOTOR_MAX_SPEED     512
#define LEDC_MOTOR_DUTY_MAX 255   // 8-bit LEDC
// 내부 속도 = 실제 속도 * TRACK_SPEED_SCALE (10ms마다 step)
// 기동(START)→max 약 1초, max→정지 약 0.4초
#define TRACK_SPEED_SCALE   10
#define TRACK_ACCEL_STEP    12    // 10ms당 가속 (×10 스케일)
#define TRACK_DECEL_STEP    100   // 10ms당 감속 (×10 스케일)
// 스틱 전진 = axis_y 음수 → 후진 반동은 양수 최대 속도
#define RECOIL_BACK_SPEED   MOTOR_MAX_SPEED

// ============================================================================
// 외부 함수 선언 (my_platform.c)
// ============================================================================
extern bool gamepad_is_connected(void);
extern bool gamepad_read_new_connection(void);
extern bool gamepad_read(int32_t* axis_y, int32_t* axis_ry, uint16_t* buttons,
                         uint8_t* dpad, uint8_t* misc_buttons);

// ============================================================================
// 전역 상태
// ============================================================================

// NVS
static nvs_handle_t g_nvs_handle;

// DFPlayer
static int g_current_volume = 20;
static int g_temp_volume = 20;

// 트랙 모터 (목표 속도 vs 램프 현재 속도)
static int g_target_left_speed = 0;
static int g_target_right_speed = 0;
static int g_current_left_x10 = 0;
static int g_current_right_x10 = 0;

// 터렛 서보
static int g_turret_angle = 90;
static int64_t g_turret_last_step_ms = 0;
static int64_t g_turret_last_input_ms = 0;
static bool g_turret_attached = false;

// 게임패드 연결 직후 입력 무시 시각
static int64_t g_input_ignore_until_ms = 0;

// 포신 발사
static bool g_cannon_firing = false;
static int64_t g_cannon_start_time = 0;

// 리코일 (pending: LED/효과음 대기 → active: 후진/안정화)
static bool g_recoil_pending = false;
static bool g_recoil_active = false;
static int64_t g_recoil_start_time = 0;

// 기관총(게틀링)
static bool g_machinegun_firing = false;
static int64_t g_machinegun_start_time = 0;
static bool g_mg_led_on = false;
static int64_t g_mg_led_last_toggle = 0;

// 효과음
static int64_t g_last_idle_sound_time = 0;

// 볼륨 조절
static bool g_l1_pressed = false;
static bool g_r1_pressed = false;
static int64_t g_l1_last_change = 0;
static int64_t g_r1_last_change = 0;
static bool g_volume_changed = false;

// Start 버튼 (터렛 중앙) 엣지 감지
static bool g_start_pressed = false;

// 현재 시간 (us)
static inline int64_t now_us(void) {
    return esp_timer_get_time();
}

static inline int64_t now_ms(void) {
    return now_us() / 1000;
}

// ============================================================================
// NVS 볼륨 관리
// ============================================================================
static void load_volume_from_nvs(void) {
    int32_t stored = -1;
    esp_err_t err = nvs_get_i32(g_nvs_handle, NVS_KEY_VOLUME, &stored);
    if (err != ESP_OK || stored < 11 || stored > 30) {
        stored = 20;
        ESP_LOGI(TAG, "NVS 볼륨 없음, 기본값 사용: %d", stored);
    } else {
        ESP_LOGI(TAG, "NVS 볼륨 로드: %d", stored);
    }
    g_current_volume = (int)stored;
    g_temp_volume = (int)stored;
}

static void save_volume_to_nvs(int vol) {
    int32_t clamped = vol;
    if (clamped < 11) clamped = 11;
    if (clamped > 30) clamped = 30;
    nvs_set_i32(g_nvs_handle, NVS_KEY_VOLUME, clamped);
    nvs_commit(g_nvs_handle);
    ESP_LOGI(TAG, "NVS 볼륨 저장: %d", clamped);
}

// ============================================================================
// LEDC 초기화
// ============================================================================

// 채널별 GPIO 매핑 테이블
typedef struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
} motor_pin_map_t;

static const motor_pin_map_t g_motor_pins[] = {
    { .gpio = PIN_LEFT_IN1,  .channel = LEDC_CH_LEFT_IN1  },
    { .gpio = PIN_LEFT_IN2,  .channel = LEDC_CH_LEFT_IN2  },
    { .gpio = PIN_RIGHT_IN1, .channel = LEDC_CH_RIGHT_IN1 },
    { .gpio = PIN_RIGHT_IN2, .channel = LEDC_CH_RIGHT_IN2 },
};

static void init_ledc(void) {
    // 모터 타이머: 5kHz, 8-bit
    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_MOTOR_RES,
        .timer_num = LEDC_TIMER_MOTOR,
        .freq_hz = LEDC_MOTOR_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&motor_timer);

    // 모터 채널 초기화
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = g_motor_pins[i].channel,
            .timer_sel = LEDC_TIMER_MOTOR,
            .gpio_num = g_motor_pins[i].gpio,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&ch);
    }

    // 서보 타이머: 50Hz, 14-bit (채널은 입력 시 attach)
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_SERVO_RES,
        .timer_num = LEDC_TIMER_SERVO,
        .freq_hz = LEDC_SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);
}

// ============================================================================
// 모터 제어 (panzer4식 가속/감속 램프)
// ============================================================================
static void apply_motor_pwm(ledc_channel_t ch_in1, ledc_channel_t ch_in2, int speed) {
    int abs_sp = abs(speed);
    if (abs_sp == 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in2);
        return;
    }

    if (abs_sp > MOTOR_MAX_SPEED) abs_sp = MOTOR_MAX_SPEED;
    // 저속 구간은 기동 토크 확보 (윙 소리만 나는 구간 회피)
    if (abs_sp < MOTOR_START_SPEED) abs_sp = MOTOR_START_SPEED;

    uint32_t duty = (uint32_t)abs_sp * LEDC_MOTOR_DUTY_MAX / MOTOR_MAX_SPEED;
    if (duty > LEDC_MOTOR_DUTY_MAX) duty = LEDC_MOTOR_DUTY_MAX;

    if (speed > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in2);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in2, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in2);
    }
}

// panzer4: 0 기동 시 START 점프, 가속/감속 step, 정지 시 START 이하 → 0
static void update_track_speed_x10(int* current_x10, int target) {
    int cur = *current_x10;
    int target_x10 = target * TRACK_SPEED_SCALE;
    int min_x10 = MOTOR_START_SPEED * TRACK_SPEED_SCALE;

    if (cur == target_x10) {
        return;
    }

    if (cur == 0 && target_x10 != 0) {
        int sign = (target_x10 > 0) ? 1 : -1;
        int start = sign * min_x10;
        int target_abs = (target_x10 > 0) ? target_x10 : -target_x10;
        if (target_abs <= min_x10) {
            *current_x10 = target_x10;
            return;
        }
        cur = start;
        *current_x10 = cur;
    }

    int diff = target_x10 - cur;
    bool is_decelerating = false;
    if (cur > 0) {
        if (target_x10 < cur) is_decelerating = true;
    } else if (cur < 0) {
        if (target_x10 > cur) is_decelerating = true;
    }

    int step = is_decelerating ? TRACK_DECEL_STEP : TRACK_ACCEL_STEP;
    if (diff > 0) {
        *current_x10 = (diff > step) ? (cur + step) : target_x10;
    } else {
        *current_x10 = (-diff > step) ? (cur - step) : target_x10;
    }

    if (target_x10 == 0) {
        int cur_abs = (*current_x10 > 0) ? *current_x10 : -*current_x10;
        if (cur_abs < min_x10) {
            *current_x10 = 0;
        }
    }
}

// 스틱 값 → 목표 속도. 0이 아니면 최소 MOTOR_START_SPEED 보장.
static int clamp_track_target(int v) {
    if (v > MOTOR_MAX_SPEED) v = MOTOR_MAX_SPEED;
    if (v < -MOTOR_MAX_SPEED) v = -MOTOR_MAX_SPEED;
    if (v != 0 && abs(v) < MOTOR_START_SPEED) {
        v = (v > 0) ? MOTOR_START_SPEED : -MOTOR_START_SPEED;
    }
    return v;
}

static void set_track_targets(int left, int right) {
    g_target_left_speed = clamp_track_target(left);
    g_target_right_speed = clamp_track_target(right);
}

// 리코일·비상 정지 등 램프 우회
static void set_track_immediate(int left, int right) {
    if (left > MOTOR_MAX_SPEED) left = MOTOR_MAX_SPEED;
    if (left < -MOTOR_MAX_SPEED) left = -MOTOR_MAX_SPEED;
    if (right > MOTOR_MAX_SPEED) right = MOTOR_MAX_SPEED;
    if (right < -MOTOR_MAX_SPEED) right = -MOTOR_MAX_SPEED;
    g_target_left_speed = left;
    g_target_right_speed = right;
    g_current_left_x10 = left * TRACK_SPEED_SCALE;
    g_current_right_x10 = right * TRACK_SPEED_SCALE;
    apply_motor_pwm(LEDC_CH_LEFT_IN1, LEDC_CH_LEFT_IN2, left);
    apply_motor_pwm(LEDC_CH_RIGHT_IN1, LEDC_CH_RIGHT_IN2, right);
}

static void process_motor_ramp(void) {
    if (g_recoil_active) return;

    update_track_speed_x10(&g_current_left_x10, g_target_left_speed);
    update_track_speed_x10(&g_current_right_x10, g_target_right_speed);
    apply_motor_pwm(LEDC_CH_LEFT_IN1, LEDC_CH_LEFT_IN2,
                    g_current_left_x10 / TRACK_SPEED_SCALE);
    apply_motor_pwm(LEDC_CH_RIGHT_IN1, LEDC_CH_RIGHT_IN2,
                    g_current_right_x10 / TRACK_SPEED_SCALE);
}

// ============================================================================
// 서보 제어 (무입력 시 연결 해제 → 버즈/전류 감소)
// ============================================================================
static void turret_apply_pwm(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // 0.5ms(0도) ~ 2.5ms(180도) in 14-bit (0~16383) at 50Hz(20ms)
    // 0.5ms = 409, 2.5ms = 2048
    uint32_t duty = 409 + (uint32_t)((int32_t)angle * (2048 - 409) / 180);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_SERVO, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CH_SERVO);
}

static void turret_attach(void) {
    if (g_turret_attached) return;

    ledc_channel_config_t servo_ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CH_SERVO,
        .timer_sel = LEDC_TIMER_SERVO,
        .gpio_num = PIN_TURRET_SERVO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&servo_ch);
    g_turret_attached = true;
    turret_apply_pwm(g_turret_angle);
    ESP_LOGD(TAG, "터렛 서보 연결");
}

static void turret_detach(void) {
    if (!g_turret_attached) return;

    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CH_SERVO, 0);
    gpio_reset_pin(PIN_TURRET_SERVO);
    g_turret_attached = false;
    ESP_LOGD(TAG, "터렛 서보 연결 해제");
}

static void set_turret_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    g_turret_angle = angle;

    if (!g_turret_attached) {
        turret_attach();
    } else {
        turret_apply_pwm(g_turret_angle);
    }
}

static void process_turret_idle(void) {
    if (!g_turret_attached) return;
    if (now_ms() - g_turret_last_input_ms >= TURRET_IDLE_DISCONNECT_MS) {
        turret_detach();
    }
}

// ============================================================================
// 게임패드 처리
// ============================================================================
static void process_gamepad(int32_t axis_y, int32_t axis_ry,
                            uint16_t buttons, uint8_t dpad, uint8_t misc_buttons) {
    // 연결 직후 grace: 잔여 D-Pad/버튼으로 터렛·모터가 움직이지 않게
    if (now_ms() < g_input_ignore_until_ms) {
        if (!g_recoil_active) {
            set_track_targets(0, 0);
        }
        return;
    }

    // 데드존
    int left_y = (abs(axis_y) < 50) ? 0 : (int)axis_y;
    int right_y = (abs(axis_ry) < 50) ? 0 : (int)axis_ry;

    // 목표 속도만 설정 — 실제 PWM은 process_motor_ramp()가 가속/감속
    if (!g_recoil_active) {
        set_track_targets(left_y, right_y);
    }

    // D-PAD 좌우: 터렛 회전
    // - 각도 변경 시에만 attach (연결 직후 PWM 재인가로 위치가 튀는 것 방지)
    // - 이미 attach된 상태에서 홀드 중이면 idle 타이머만 갱신
    if ((dpad & DPAD_LEFT) || (dpad & DPAD_RIGHT)) {
        int64_t now = now_ms();
        if (g_turret_attached) {
            g_turret_last_input_ms = now;
        }
        if (now - g_turret_last_step_ms >= TURRET_STEP_INTERVAL_MS) {
            if (dpad & DPAD_LEFT) {
                if (g_turret_angle > 0) {
                    g_turret_last_step_ms = now;
                    g_turret_last_input_ms = now;
                    set_turret_angle(g_turret_angle - 1);
                }
            } else if (dpad & DPAD_RIGHT) {
                if (g_turret_angle < 180) {
                    g_turret_last_step_ms = now;
                    g_turret_last_input_ms = now;
                    set_turret_angle(g_turret_angle + 1);
                }
            }
        }
    }

    // Start: 터렛 서보 중앙(90°)
    if (misc_buttons & MISC_BUTTON_START) {
        if (!g_start_pressed) {
            g_start_pressed = true;
            g_turret_last_input_ms = now_ms();
            set_turret_angle(90);
        }
    } else {
        g_start_pressed = false;
    }

    // B 버튼: LED·효과음 먼저, 반동은 RECOIL_DELAY_MS 후 process_recoil()
    if ((buttons & BUTTON_B) && !g_cannon_firing && !g_machinegun_firing
            && !g_recoil_pending && !g_recoil_active) {
        g_cannon_firing = true;
        g_cannon_start_time = now_ms();
        gpio_set_level(PIN_CANNON_LED, 1);
        dfplayer_play(DFPLAYER_TRACK_CANNON);

        g_recoil_pending = true;
        g_recoil_start_time = now_ms();
    }

    // A 버튼: 기관총(게틀링) 발사 — LED 깜빡임 + 효과음
    if ((buttons & BUTTON_A) && !g_machinegun_firing && !g_cannon_firing) {
        g_machinegun_firing = true;
        g_machinegun_start_time = now_ms();
        g_mg_led_on = true;
        g_mg_led_last_toggle = now_ms();
        gpio_set_level(PIN_MG_LED, 1);
        dfplayer_play(DFPLAYER_TRACK_MACHINEGUN);
    }

    // L1: 볼륨 감소
    if (buttons & BUTTON_SHOULDER_L) {
        if (!g_l1_pressed) {
            g_l1_pressed = true;
            g_temp_volume = g_current_volume;
            g_l1_last_change = now_ms();
        }
        if (g_temp_volume > 11 && (now_ms() - g_l1_last_change >= VOLUME_CHANGE_INTERVAL)) {
            g_temp_volume--;
            g_l1_last_change = now_ms();
        }
    } else {
        if (g_l1_pressed) {
            g_l1_pressed = false;
            if (g_temp_volume != g_current_volume) {
                g_current_volume = g_temp_volume;
                dfplayer_set_volume(g_current_volume);
                g_volume_changed = true;
            }
        }
    }

    // R1: 볼륨 증가
    if (buttons & BUTTON_SHOULDER_R) {
        if (!g_r1_pressed) {
            g_r1_pressed = true;
            g_temp_volume = g_current_volume;
            g_r1_last_change = now_ms();
        }
        if (g_temp_volume < 30 && (now_ms() - g_r1_last_change >= VOLUME_CHANGE_INTERVAL)) {
            g_temp_volume++;
            g_r1_last_change = now_ms();
        }
    } else {
        if (g_r1_pressed) {
            g_r1_pressed = false;
            if (g_temp_volume != g_current_volume) {
                g_current_volume = g_temp_volume;
                dfplayer_set_volume(g_current_volume);
                g_volume_changed = true;
            }
        }
    }

    // 볼륨 변경 시 NVS 저장
    if (g_volume_changed) {
        save_volume_to_nvs(g_current_volume);
        g_volume_changed = false;
    }
}

// ============================================================================
// 타이머 기반 처리
// ============================================================================
static void process_cannon_firing(void) {
    if (!g_cannon_firing) return;
    if (now_ms() - g_cannon_start_time >= CANNON_LED_DURATION) {
        g_cannon_firing = false;
        gpio_set_level(PIN_CANNON_LED, 0);
    }
}

// panzer4: MG_LED_BLINK_MS 주기로 토글, MG_FIRE_MS 후 소등
static void process_machinegun_firing(void) {
    if (!g_machinegun_firing) return;

    int64_t now = now_ms();
    if (now - g_machinegun_start_time >= MACHINE_GUN_DURATION) {
        g_machinegun_firing = false;
        g_mg_led_on = false;
        gpio_set_level(PIN_MG_LED, 0);
        return;
    }

    if (now - g_mg_led_last_toggle >= MG_LED_BLINK_MS) {
        g_mg_led_last_toggle = now;
        g_mg_led_on = !g_mg_led_on;
        gpio_set_level(PIN_MG_LED, g_mg_led_on ? 1 : 0);
    }
}

static void process_recoil(void) {
    int64_t now = now_ms();

    // LED·효과음 재생 후 반동 시작
    if (g_recoil_pending) {
        if (now - g_recoil_start_time < RECOIL_DELAY_MS) {
            return;
        }
        g_recoil_pending = false;
        g_recoil_active = true;
        g_recoil_start_time = now;
        set_track_immediate(RECOIL_BACK_SPEED, RECOIL_BACK_SPEED);
        return;
    }

    if (!g_recoil_active) return;
    int64_t elapsed = now - g_recoil_start_time;

    if (elapsed < RECOIL_BACK_DURATION) {
        set_track_immediate(RECOIL_BACK_SPEED, RECOIL_BACK_SPEED);
        return;
    }
    if (elapsed < RECOIL_BACK_DURATION + RECOIL_SETTLE_DURATION) {
        set_track_immediate(0, 0);
        return;
    }
    g_recoil_active = false;
}

static void process_idle_sound(void) {
    if (gamepad_is_connected()) return;
    if (g_cannon_firing || g_machinegun_firing) return;

    if (now_ms() - g_last_idle_sound_time >= IDLE_SOUND_INTERVAL_MS) {
        dfplayer_play_loop(DFPLAYER_TRACK_IDLE);
        g_last_idle_sound_time = now_ms();
    }
}

// ============================================================================
// 제어 태스크 (FreeRTOS)
// ============================================================================
static void control_task(void* arg) {
    (void)arg;

    int32_t axis_y, axis_ry;
    uint16_t buttons;
    uint8_t dpad, misc_buttons;

    bool prev_connected = false;

    while (1) {
        bool cur_connected = gamepad_is_connected();

        // 연결 직후: 터렛 서보 연결·중앙 + 입력 유예 + 효과음
        if (gamepad_read_new_connection()) {
            int64_t now = now_ms();
            g_input_ignore_until_ms = now + GAMEPAD_CONNECT_GRACE_MS;
            g_turret_last_step_ms = now; // 유예 직후 즉시 1° 스텝 방지
            g_turret_last_input_ms = now;
            set_track_targets(0, 0);
            set_turret_angle(90); // 패드 연결 시 서보 attach + 중앙

            dfplayer_stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            dfplayer_set_volume(g_current_volume);
            dfplayer_play(DFPLAYER_TRACK_CONNECTED);
            ESP_LOGI(TAG, "게임패드 연결됨");
        }

        // 연결 해제 직후: 정리 (서보 포함)
        if (prev_connected && !cur_connected) {
            dfplayer_set_volume(15); // initialVolume
            dfplayer_play_loop(DFPLAYER_TRACK_IDLE);
            g_last_idle_sound_time = now_ms();
            set_track_immediate(0, 0);
            turret_detach();
            g_machinegun_firing = false;
            g_mg_led_on = false;
            gpio_set_level(PIN_MG_LED, 0);
            gpio_set_level(PIN_CANNON_LED, 0);
            ESP_LOGI(TAG, "게임패드 연결 해제됨");
        }
        prev_connected = cur_connected;

        // 게임패드 데이터 처리
        if (cur_connected) {
            if (gamepad_read(&axis_y, &axis_ry, &buttons, &dpad, &misc_buttons)) {
                process_gamepad(axis_y, axis_ry, buttons, dpad, misc_buttons);
            }
        }

        process_cannon_firing();
        process_machinegun_firing();
        process_recoil();
        process_motor_ramp();
        process_turret_idle();
        process_idle_sound();

        vTaskDelay(pdMS_TO_TICKS(LOOP_INTERVAL_MS));
    }
}

// ============================================================================
// Bluepad32 메인 (BTstack 루프 - 절대 리턴 안 함)
// ============================================================================
static void bt_main_task(void* arg) {
    (void)arg;

    btstack_init();
    uni_platform_set_custom(get_my_platform());
    uni_init(0, NULL);
    btstack_run_loop_execute();

    // 여기까지 오면 안 됨
    vTaskDelete(NULL);
}

// ============================================================================
// 앱 메인
// ============================================================================
void app_main(void) {
    // 모니터 연결 직후에도 바로 보이도록 즉시 로그
    // 캐패시터 충전 대기
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "RC Tank 초기화 시작");

    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_open(NVS_NAMESPACE, NVS_READWRITE, &g_nvs_handle);
    load_volume_from_nvs();

    // GPIO 초기화
    gpio_reset_pin(PIN_LEFT_IN1);
    gpio_reset_pin(PIN_LEFT_IN2);
    gpio_reset_pin(PIN_RIGHT_IN1);
    gpio_reset_pin(PIN_RIGHT_IN2);
    gpio_reset_pin(PIN_CANNON_LED);
    gpio_reset_pin(PIN_MG_LED);
    // 터렛 서보는 게임패드 연결 시 attach (부팅 시 PWM/출력 설정 안 함)
    gpio_reset_pin(PIN_TURRET_SERVO);

    gpio_set_direction(PIN_LEFT_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LEFT_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RIGHT_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RIGHT_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_CANNON_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_MG_LED, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_LEFT_IN1, 0);
    gpio_set_level(PIN_LEFT_IN2, 0);
    gpio_set_level(PIN_RIGHT_IN1, 0);
    gpio_set_level(PIN_RIGHT_IN2, 0);
    gpio_set_level(PIN_CANNON_LED, 0);
    gpio_set_level(PIN_MG_LED, 0);

    // LEDC 초기화 (서보 채널은 패드 연결 시 turret_attach)
    init_ledc();

    // DFPlayer 초기화 (실패해도 탱크/BT는 계속)
    if (dfplayer_init() != ESP_OK) {
        ESP_LOGW(TAG, "DFPlayer init failed — continuing without sound");
    } else {
        dfplayer_set_volume(g_current_volume);
        vTaskDelay(pdMS_TO_TICKS(200));
        dfplayer_play_loop(DFPLAYER_TRACK_IDLE);
        g_last_idle_sound_time = now_ms();
    }

    ESP_LOGI(TAG, "초기화 완료 — BTstack 시작");

    // 게임패드 상태 뮤프스 생성 (BTstack보다 먼저)
    gamepad_state_init();

    // 제어 태스크 (BTstack보다 낮은 우선순위 — ESP32-C3 단일 코어에서 BLE 안정성)
    xTaskCreate(control_task, "tank_ctrl", 4096, NULL, 4, NULL);

    // BTstack 메인 태스크 (높은 우선순위, 코어 0)
    xTaskCreatePinnedToCore(bt_main_task, "bt_main", 8192, NULL, 10, NULL, 0);
}
