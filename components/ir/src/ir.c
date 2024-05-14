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

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir.h"
#include "ir_nec_encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       41
#define EXAMPLE_IR_RX_GPIO_NUM       42
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250

static const char *TAG = "example";

// #define IR_NEC_ENCODE

/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);

    if (!valid_leading_code) {
        return false;
    }

    cur++;

    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }

        cur++;
    }

    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }

        cur++;
    }

    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    printf("NEC frame start---\r\n");
    printf("recv size: %d, uint16_t ir_data[][2] = {\n", symbol_num);

    for (size_t i = 0; i < symbol_num; i++) {
        // printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
        //        rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);

        printf("{%d,%d},", rmt_nec_symbols[i].duration0, rmt_nec_symbols[i].duration1);
    }

    printf("};\n");

    printf("---NEC frame end: ");

    // decode RMT symbols
    switch (symbol_num) {
        case 34: // NEC normal frame
            if (nec_parse_frame(rmt_nec_symbols)) {
                printf("Address=%04X, Command=%04X\r\n\r\n", s_nec_code_address, s_nec_code_command);
            }

            break;

        case 2: // NEC repeat frame
            if (nec_parse_frame_repeat(rmt_nec_symbols)) {
                printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code_address, s_nec_code_command);
            }

            break;

        default:
            printf("Unknown NEC frame\r\n\r\n");
            break;
    }
}

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}


rmt_channel_handle_t rx_channel = NULL;
rmt_channel_handle_t tx_channel = NULL;
rmt_encoder_handle_t nec_encoder = NULL;

QueueHandle_t receive_queue = NULL;

esp_err_t ir_init(const ir_config_t *config)
{
    if (config->rx == GPIO_NUM_NC && config->tx == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "IR not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "config rx: %d, tx: %d", config->rx, config->tx);

    if (config->rx != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "create RMT RX channel");
        rmt_rx_channel_config_t rx_channel_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = IR_RESOLUTION_HZ,
            .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
            .gpio_num = config->rx,
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

        ESP_LOGI(TAG, "register RX done callback");

        receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
        assert(receive_queue);
        rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = example_rmt_rx_done_callback,
        };
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));
    }

    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = config->tx,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    ESP_LOGI(TAG, "modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));


    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_ir_raw_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG, "enable RMT TX");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));

    if (config->rx != GPIO_NUM_NC) {
        ESP_ERROR_CHECK(rmt_enable(rx_channel));
    }

    return ESP_OK;
}

esp_err_t ir_recv(rmt_symbol_word_t **data, size_t *size)
{
    ESP_LOGI(TAG, "waiting for IR command");
    // the following timing requirement is based on NEC protocol
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    };
    rmt_symbol_word_t *raw_symbols = malloc(sizeof(rmt_symbol_word_t) * 64);
    rmt_rx_done_event_data_t rx_data;

    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(rmt_symbol_word_t) * 64, &receive_config));
    xQueueReceive(receive_queue, &rx_data, portMAX_DELAY);

    example_parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);

    *size = rx_data.num_symbols;
    *data = malloc(sizeof(rmt_symbol_word_t) * rx_data.num_symbols);
    memcpy(*data, rx_data.received_symbols, sizeof(rmt_symbol_word_t) * rx_data.num_symbols);

    free(raw_symbols);

    return ESP_OK;
}

esp_err_t ir_send(const rmt_symbol_word_t *data, size_t size)
{
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, data, size, &transmit_config));
    rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);

    return ESP_OK;
}

esp_err_t ir_deinit(void)
{
    if (rx_channel == NULL || tx_channel == NULL) {
        ESP_LOGE(TAG, "IR not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disabling IR");

    rmt_disable(rx_channel);
    rmt_del_channel(rx_channel);

    rmt_disable(tx_channel);
    rmt_del_channel(tx_channel);

    return ESP_OK;
}

#define EXAMPLE_IR_TX_GPIO_NUM       41
#define EXAMPLE_IR_RX_GPIO_NUM       42
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream
void test_ir(void)
{
    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM,
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    ESP_LOGI(TAG, "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    };

    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    ESP_LOGI(TAG, "modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    // this example won't send NEC frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };

    rmt_encoder_handle_t nec_encoder = NULL;
#ifdef IR_NEC_ENCODE
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));
#else
    ESP_ERROR_CHECK(rmt_new_ir_raw_encoder(&nec_encoder_cfg, &nec_encoder));
#endif

    ESP_LOGI(TAG, "enable RMT TX and RX channels");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;
    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));

    while (1) {
        // wait for RX done signal
        if (xQueueReceive(receive_queue, &rx_data, portMAX_DELAY) == pdPASS) {
            // parse the receive symbols and print the result
            example_parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        }

#ifdef IR_NEC_ENCODE
        else {
            // timeout, transmit predefined IR NEC packets
            const ir_nec_scan_code_t scan_code = {
                .address = 0x0440,
                .command = 0x3003,
            };
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
        }

#else
        else {
            // uint16_t ir_data[][2] = {{9013,4515},{564,567},{538,570},{561,568},{537,594},{537,569},{561,570},{537,1700},{562,569},{536,570},{562,569},{537,1701},{561,568},{537,595},{537,568},{562,570},{536,569},{563,1700},{536,1701},{562,569},{536,571},{561,568},{536,596},{536,569},{536,596},{536,569},{562,569},{536,570},{561,570},{536,1703},{560,1700},{536,596},{536,569},{536,0},};
            uint16_t ir_data[][2] = {{1, 1}, {2, 2}, {3, 3}};
            rmt_symbol_word_t raw_symbols[64];

            printf("uint16_t send_ir_data[][2] = {\n");

            for (int i = 0; i < sizeof(ir_data) / sizeof(ir_data[0]); i++) {
                raw_symbols[i].level0 = 1;
                raw_symbols[i].level1 = 0;
                raw_symbols[i].duration0 = ir_data[i][0];
                raw_symbols[i].duration1 = ir_data[i][1];

                printf("{%d,%d},", raw_symbols[i].duration0, raw_symbols[i].duration1);
            }

            printf("};\n");

            ESP_LOGI(TAG, "transmit raw symbols");

            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, raw_symbols, sizeof(raw_symbols) / sizeof(rmt_symbol_word_t), &transmit_config));
            rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);
        }

#endif
    }
}
