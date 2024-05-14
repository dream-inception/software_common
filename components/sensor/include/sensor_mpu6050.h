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
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

#define SENSOR_MPU6050_CONFIG_DEFAULT()   { \
        .i2c_scl = GPIO_NUM_NC, \
        .i2c_sda = GPIO_NUM_NC, \
        .acce_fs = ACCE_FS_4G, \
        .gyro_fs = GYRO_FS_500DPS, \
    }

typedef struct {
    gpio_num_t i2c_scl;
    gpio_num_t i2c_sda;
    mpu6050_acce_fs_t acce_fs;
    mpu6050_gyro_fs_t gyro_fs;
} sensor_mpu6050_config_t;


esp_err_t sensor_mpu6050_init(const sensor_mpu6050_config_t *config);

esp_err_t sensor_mpu6050_deinit(void);

esp_err_t sensor_mpu6050_read(mpu6050_acce_value_t *acce, mpu6050_gyro_value_t *gyro);



#ifdef __cplusplus
}
#endif /**< _cplusplus */
