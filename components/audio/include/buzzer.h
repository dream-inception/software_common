// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */


// Play mode enumeration
typedef enum {
    PLAY_MODE_INVALID = 0, // Invalid play mode
    PLAY_MODE_CONTINUOUS,  // Continuous play
    PLAY_MODE_REPEAT_ONE,  // Repeat single
    PLAY_MODE_SHUFFLE,     // Shuffle play
    
    PLAY_MODE_BEEP,        // 
} buzzer_play_mode_t;

// Song enumeration
typedef enum {
    SONG_CASTLE_IN_THE_SKY,
    SONG_HAPPY_BIRTHDAY,
    SONG_PAINTER,
    SONG_TWO_TIGERS,
    SONG_TWINKLE_TWINKLE,
    SONG_JINGLE_BELLS,
    SONG_LITTLE_BEE,
    SONG_NUMBER_SONG,
    SONG_LONDON_BRIDGE,
    SONG_MARY_LAMB,
    SONG_NUM_MAX,
} buzzer_song_t;

// Notification sound enumeration
typedef enum {
    BEEP_STARTUP,
    BEEP_SHUTDOWN,
    BEEP_MESSAGE,
    BEEP_TIP,
    BEEP_CONFIRM,
    BEEP_WARNING,
    BEEP_SUCCESS,
    BEEP_ERROR,
} buzzer_beep_t;


typedef struct {
    gpio_num_t gpio_num;
    uint8_t volume;

    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
} buzzer_config_t;

#define BUZZER_CONFIG_DEFAULT() { \
        .gpio_num = GPIO_NUM_NC, \
        .volume = 50, \
        .speed_mode = LEDC_LOW_SPEED_MODE, \
        .duty_resolution = LEDC_TIMER_13_BIT, \
        .timer_num = LEDC_TIMER_2, \
        .freq_hz = 5000, \
    }

esp_err_t buzzer_set_volume(uint8_t volume);

esp_err_t buzzer_play_song(buzzer_song_t song, buzzer_play_mode_t play_mode);
esp_err_t buzzer_pause_song(bool enable);

esp_err_t buzzer_play_beep(buzzer_beep_t beep);

esp_err_t buzzer_init(const buzzer_config_t *config);
esp_err_t buzzer_deinit(void);

#ifdef __cplusplus
}
#endif /**< _cplusplus */
