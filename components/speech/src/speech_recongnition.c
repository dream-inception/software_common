/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "spi_flash_mmap.h"
#include "driver/i2s_std.h"
#include "xtensa/core-macros.h"
#include "esp_partition.h"
#include "esp_log.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "dl_lib_coefgetter_if.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"
#include "esp_process_sdkconfig.h"

#include "speech_recognition.h"

static const char *TAG = "speech_recognition";

int *buffer = NULL;
int size = 0;
static esp_mn_iface_t *multinet = NULL;
const esp_wn_iface_t *wakenet = NULL;
static model_iface_data_t *model_wn_data = NULL;
static model_iface_data_t *model_mn_data = NULL;
static i2s_chan_handle_t rx_chan;        // I2S rx channel handler

esp_err_t speech_mic_init(const speech_mic_config_t *config)
{
    i2s_chan_config_t rx_chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 3,
        .dma_frame_num = 300,
        .auto_clear = true,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = config->mic_gpio.i2s_bclk,
            .ws   = config->mic_gpio.i2s_ws,
            .dout = I2S_GPIO_UNUSED,
            .din  = config->mic_gpio.i2s_din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    i2s_channel_enable(rx_chan);

    return ESP_OK;
}

esp_err_t speech_mic_record(int **audio_data, size_t *audio_len)
{
    esp_err_t ret = ESP_OK;
    *audio_data = 0;

    ret = i2s_channel_read(rx_chan, buffer, size * 2 * sizeof(int), audio_len, portMAX_DELAY);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s read failed, ret: %s", esp_err_to_name(ret));
        return ret;
    }

    // ESP_LOGW(TAG, "audio_len: %d, size: %d", *audio_len, size);

    // for (int x = 0; x < size * 2 / 4; x++) {
    //     int s1 = ((buffer[x * 4] + buffer[x * 4 + 1]) >> 13) & 0x0000FFFF;
    //     int s2 = ((buffer[x * 4 + 2] + buffer[x * 4 + 3]) << 3) & 0xFFFF0000;
    //     buffer[x] = s1 | s2;
    // }

    *audio_data = buffer;

    return ESP_OK;
}

esp_err_t speech_recognition_init(void)
{
    srmodel_list_t *models = esp_srmodel_init("model");
    const char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL); // select WakeNet model
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE); // select MultiNet model

    wakenet = esp_wn_handle_from_name(wn_name);
    multinet = esp_mn_handle_from_name(mn_name);
    model_wn_data = wakenet->create(wn_name, DET_MODE_90);
    int wn_num = wakenet->get_word_num(model_wn_data);
    float wn_threshold = 0;
    int wn_sample_rate = wakenet->get_samp_rate(model_wn_data);
    int audio_wn_chunksize = wakenet->get_samp_chunksize(model_wn_data);
    ESP_LOGI(TAG, "keywords_num = %d, threshold = %f, sample_rate = %d, chunksize = %d, sizeof_uint16 = %d", wn_num, wn_threshold, wn_sample_rate, audio_wn_chunksize, sizeof(int16_t));

    model_mn_data = multinet->create(mn_name, 6000);

    int audio_mn_chunksize = multinet->get_samp_chunksize(model_mn_data);
    int mn_num = multinet->get_samp_chunknum(model_mn_data);
    int mn_sample_rate = multinet->get_samp_rate(model_mn_data);
    ESP_LOGI(TAG, "keywords_num = %d , sample_rate = %d, chunksize = %d, sizeof_uint16 = %d", mn_num,  mn_sample_rate, audio_mn_chunksize, sizeof(int16_t));

    size = audio_wn_chunksize;

    if (audio_mn_chunksize > audio_wn_chunksize) {
        size = audio_mn_chunksize;
    }

    buffer = (int *)malloc(size * 2 * sizeof(int));

    return ESP_OK;
}

esp_err_t speech_recognition_wakenet(const int16_t *audio_data, size_t audio_len, bool *wakenet_state)
{
    wakenet_state_t r = wakenet->detect(model_wn_data, (int16_t *)audio_data);
    *wakenet_state = false;

    if (r == WAKENET_DETECTED) {
        ESP_LOGI(TAG, "%s DETECTED", wakenet->get_word_name(model_wn_data, r));
        *wakenet_state = true;
    }

    return ESP_OK;
}

esp_err_t speech_recognition_command(const int16_t *audio_data, size_t audio_len, int *commands_id)
{
    esp_mn_state_t mn_state = multinet->detect(model_mn_data, (int16_t *)audio_data);

    *commands_id = -1;

    if (mn_state == ESP_MN_STATE_DETECTED) {
        esp_mn_results_t *mn_result = multinet->get_results(model_mn_data);
        ESP_LOGI(TAG, "MN test successfully, Commands ID: %d", mn_result->phrase_id[0]);
        *commands_id =  mn_result->phrase_id[0];
        return ESP_OK;

    } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
        esp_mn_results_t *mn_result = multinet->get_results(model_mn_data);
        // ESP_LOGI(TAG, "timeout, string:%s", mn_result->string);

        return ESP_ERR_TIMEOUT;
    } else {
        return ESP_ERR_NOT_FINISHED;
    }
}
