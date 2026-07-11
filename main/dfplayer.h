/**
 * @file dfplayer.h
 * @brief DFPlayer Mini UART 제어
 */
#ifndef DFPLAYER_H
#define DFPLAYER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DFPLAYER_TRACK_IDLE      1  /* 0001.mp3 대기 반복 */
#define DFPLAYER_TRACK_CANNON    2  /* 0002.mp3 포신 발사 */
#define DFPLAYER_TRACK_MACHINEGUN 3 /* 0003.mp3 기관총 */
#define DFPLAYER_TRACK_CONNECTED 4  /* 0004.mp3 게임패드 연결 */

esp_err_t dfplayer_init(void);
esp_err_t dfplayer_play(uint8_t track);
esp_err_t dfplayer_play_loop(uint8_t track);
esp_err_t dfplayer_set_volume(uint8_t vol);
esp_err_t dfplayer_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* DFPLAYER_H */
