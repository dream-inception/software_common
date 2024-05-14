// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include "esp_log.h"

#include "driver/i2c.h"
#include "i2c_bus.h"
#include "sensor_mpu6050.h"

#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */

static i2c_bus_handle_t g_i2c_bus = NULL;
static mpu6050_handle_t g_mpu6050_handle = NULL;
static const char *TAG = "sensor_mpu6050";

esp_err_t sensor_mpu6050_read(mpu6050_acce_value_t *acce, mpu6050_gyro_value_t *gyro)
{
    mpu6050_get_acce(g_mpu6050_handle, acce);
    mpu6050_get_gyro(g_mpu6050_handle, gyro);

    // ESP_LOGI(TAG, "acce_x: %.2f, acce_y: %.2f, acce_z: %.2f", acce->acce_x, acce->acce_y, acce->acce_z);
    // ESP_LOGI(TAG, "gyro_x: %.2f, gyro_y: %.2f, gyro_z: %.2f", gyro->gyro_x, gyro->gyro_y, gyro->gyro_z);

    return ESP_OK;
}

esp_err_t sensor_mpu6050_init(const sensor_mpu6050_config_t *config)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->i2c_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = config->i2c_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    g_i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    g_mpu6050_handle = mpu6050_create(g_i2c_bus, MPU6050_I2C_ADDRESS);

    uint8_t mpu6050_deviceid;
    mpu6050_get_deviceid(g_mpu6050_handle, &mpu6050_deviceid);
    ESP_LOGI(TAG, "g_mpu6050_handle device ID: 0x%02x", mpu6050_deviceid);


    mpu6050_set_acce_fs(g_mpu6050_handle, config->acce_fs);
    mpu6050_set_gyro_fs(g_mpu6050_handle, config->gyro_fs);

    mpu6050_wake_up(g_mpu6050_handle);

    return ESP_OK;
}

esp_err_t sensor_mpu6050_deinit()
{
    mpu6050_delete(&g_mpu6050_handle);
    i2c_bus_delete(&g_i2c_bus);

    return ESP_OK;
}