/* Wi-Fi CSI console Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_console.h"

#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "hal/uart_ll.h"

#include "commands.h"

#include "storage.h"
#include "wifi.h"
#include "ir.h"
#include "led_pwm.h"
#include "led_ws2812.h"
#include "sensor_mpu6050.h"
#include "sensor_vibration.h"
#include "touchpad_button.h"
#include "touchpad_sensor.h"
#include "speech_recognition.h"


static const char *TAG = "app_main";

static struct {
    struct arg_lit *init;
    struct arg_int *tx_gpio;
    struct arg_int *rx_gpio;
    struct arg_str *send;
    struct arg_lit *recv;
    struct arg_end *end;
} ir_args;

static void ir_recv_task(void *arg)
{
    rmt_symbol_word_t *data;
    size_t size;

    while (1) {
        esp_err_t err = ir_recv(&data, &size);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Receive IR data failed: %d", err);
            return;
        }

        // printf("IR recv size: %d, data: {", size);
        // for (int i = 0; i < size; i++) {
        //     printf("{%d, %d},", data[i].duration0, data[i].duration1);
        // }
        // printf("\b}\n");
        free(data);
    }

    vTaskDelete(NULL);
}

static int ir_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &ir_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, ir_args.end, argv[0]);
        return 1;
    }

    if (ir_args.init->count) {
        ESP_LOGI(TAG, "Initialize IR");
        ir_config_t config = IR_CONFIG_DEFAULT();

        if (ir_args.tx_gpio->count) {
            config.tx = ir_args.tx_gpio->ival[0];
        }

        if (ir_args.rx_gpio->count) {
            config.rx = ir_args.rx_gpio->ival[0];
        }

        ESP_ERROR_CHECK(ir_init(&config));
    }

    if (ir_args.send->count) {
        // ESP_LOGI(TAG, "Send IR data: %s", ir_args.send->sval[0]);
        rmt_symbol_word_t *data = (rmt_symbol_word_t *) malloc(sizeof(rmt_symbol_word_t) * 256);

        int count = 0;
        const char *ptr = ir_args.send->sval[0];

        for (; count < 128 && *ptr;) {
            int x, y;

            if (sscanf(ptr, "{%d, %d}", &x, &y) == 2) {
                data[count].level0 = 1;
                data[count].duration0 = x;
                data[count].level1 = 0;
                data[count].duration1 = y;
                count++;

                while (*ptr && *ptr != '}') {
                    ptr++;
                }

                if (*ptr) {
                    ptr++;
                }

                if (*ptr && *ptr == ',') {
                    ptr++;
                }
            } else {
                ptr++;
            }
        }

        printf("IR send, size: %d, data: {", count);

        for (int i = 0; i < count; i++) {
            printf("{%d, %d},", data[i].duration0, data[i].duration1);
        }

        printf("\b}\n");

        ESP_ERROR_CHECK(ir_send(data, count));
        free(data);
    }

    if (ir_args.recv->count) {
        ESP_LOGI(TAG, "Receive IR data");
        xTaskCreate(ir_recv_task, "ir_recv_task", 1024 * 4, NULL, 5, NULL);
    }

    return 0;
}

static void cmd_register_ir(void)
{
    ir_args.init = arg_lit0("i", "init", "Initialize IR");
    ir_args.tx_gpio = arg_int0("t", "tx_gpio", "<tx_gpio>", "IR TX GPIO");
    ir_args.rx_gpio = arg_int0("r", "rx_gpio", "<rx_gpio>", "IR RX GPIO");
    ir_args.send = arg_str0("s", "send", "<data>", "Send IR data");
    ir_args.recv = arg_lit0("v", "recv", "Receive IR data");
    ir_args.end = arg_end(4);

    const esp_console_cmd_t ir_cmd = {
        .command = "ir",
        .help = "IR command",
        .hint = NULL,
        .func = &ir_cmd_handler,
        .argtable = &ir_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&ir_cmd));
}

static struct {
    struct arg_lit *init;
    struct arg_lit *create;
    struct arg_lit *delete;
    struct arg_lit *set;
    struct arg_lit *blink_start;
    struct arg_lit *blink_stop;
    struct arg_int *gpio;
    struct arg_int *duty;
    struct arg_int *period;
    struct arg_int *fade_ms;
    struct arg_end *end;
} led_pwm_args;

static esp_err_t led_pwm_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &led_pwm_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, led_pwm_args.end, argv[0]);
        return 1;
    }

    gpio_num_t gpio = GPIO_NUM_NC;
    uint8_t duty = 0;
    uint32_t period_ms = 1000;
    uint32_t fade_ms = 500;

    if (led_pwm_args.gpio->count) {
        gpio = led_pwm_args.gpio->ival[0];
    }

    if (led_pwm_args.duty->count) {
        duty = led_pwm_args.duty->ival[0];
    }

    if (led_pwm_args.period->count) {
        period_ms = led_pwm_args.period->ival[0];
    }

    if (led_pwm_args.fade_ms->count) {
        fade_ms = led_pwm_args.fade_ms->ival[0];
    }

    if (led_pwm_args.init->count) {
        ESP_LOGI(TAG, "Initialize LED PWM");
        led_pwm_config_t config = LED_PWM_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(led_pwm_init(&config));
    }

    if (led_pwm_args.create->count) {
        ESP_LOGI(TAG, "Create LED PWM GPIO: %d", gpio);
        ESP_ERROR_CHECK(led_pwm_create(gpio));
    }

    if (led_pwm_args.delete->count) {
        ESP_LOGI(TAG, "Delete LED PWM");
        ESP_ERROR_CHECK(led_pwm_delete(gpio));
    }

    if (led_pwm_args.set->count) {
        ESP_LOGI(TAG, "Set LED PWM duty: %d, fade: %d ms", duty, fade_ms);
        ESP_ERROR_CHECK(led_pwm_set(gpio, duty, fade_ms));
    }

    if (led_pwm_args.blink_start->count) {
        ESP_LOGI(TAG, "Start LED PWM blink, duty: %d, period: %d ms", duty, period_ms);
        ESP_ERROR_CHECK(led_pwm_blink_start(gpio, duty, period_ms));
    }

    if (led_pwm_args.blink_stop->count) {
        ESP_LOGI(TAG, "Stop LED PWM blink");
        ESP_ERROR_CHECK(led_pwm_blink_stop(gpio));
    }

    return 0;
}

static void cmd_register_led_pwm(void)
{
    led_pwm_args.init = arg_lit0(NULL, "init", "Initialize LED PWM");
    led_pwm_args.create = arg_lit0(NULL, "create", "Create LED PWM");
    led_pwm_args.delete = arg_lit0(NULL, "delete", "Delete LED PWM");
    led_pwm_args.set = arg_lit0(NULL, "set", "Set LED PWM");
    led_pwm_args.blink_start = arg_lit0(NULL, "blink_start", "Start LED PWM blink");
    led_pwm_args.blink_stop = arg_lit0(NULL, "blink_stop", "Stop LED PWM blink");
    led_pwm_args.gpio = arg_int0(NULL, "gpio", "<gpio>", "LED PWM GPIO");
    led_pwm_args.duty = arg_int0(NULL, "duty", "<duty>", "LED PWM duty");
    led_pwm_args.period = arg_int0(NULL, "period", "<ms>", "LED PWM blink period");
    led_pwm_args.fade_ms = arg_int0(NULL, "fade_ms", "<ms>", "LED PWM fade ms");
    led_pwm_args.end = arg_end(4);

    const esp_console_cmd_t led_pwm_cmd = {
        .command = "led_pwm",
        .help = "LED PWM command",
        .hint = NULL,
        .func = &led_pwm_cmd_handler,
        .argtable = &led_pwm_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&led_pwm_cmd));
}


static struct {
    struct arg_lit *init;
    struct arg_lit *create;
    struct arg_lit *set;
    struct arg_lit *blink_start;
    struct arg_int *gpio;
    struct arg_int *num;
    struct arg_int *red;
    struct arg_int *green;
    struct arg_int *blue;
    struct arg_int *period;
    struct arg_int *fade;
    struct arg_end *end;
} led_ws2812_args;

static esp_err_t led_ws2812_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &led_ws2812_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, led_ws2812_args.end, argv[0]);
        return 1;
    }

    gpio_num_t gpio = GPIO_NUM_NC;
    uint32_t num = 1;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    if (led_ws2812_args.gpio->count) {
        gpio = led_ws2812_args.gpio->ival[0];
    }

    if (led_ws2812_args.num->count) {
        num = led_ws2812_args.num->ival[0];
    }

    if (led_ws2812_args.red->count) {
        red = led_ws2812_args.red->ival[0];
    }

    if (led_ws2812_args.green->count) {
        green = led_ws2812_args.green->ival[0];
    }

    if (led_ws2812_args.blue->count) {
        blue = led_ws2812_args.blue->ival[0];
    }

    if (led_ws2812_args.init->count) {
        ESP_LOGI(TAG, "Initialize LED WS2812");
        led_ws2812_config_t config = LED_WS2812_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(led_ws2812_init(&config));
    }

    if (led_ws2812_args.create->count) {
        ESP_LOGI(TAG, "Create LED WS2812, gpio: %d, num: %d", gpio, num);
        led_ws2812_create(gpio, num);
    }

    if (led_ws2812_args.set->count) {
        uint32_t fade = 0;

        if (led_ws2812_args.fade->count) {
            fade = led_ws2812_args.fade->ival[0];
        }

        ESP_LOGI(TAG, "Set LED WS2812, red: %d, green: %d, blue: %d, fade: %d ms", red, green, blue, fade);
        led_ws2812_set_rgb(gpio, num, red, green, blue, fade);
    }

    if (led_ws2812_args.blink_start->count) {
        uint32_t period_ms = 500;

        if (led_ws2812_args.period->count) {
            period_ms = led_ws2812_args.period->ival[0];
        }

        ESP_LOGI(TAG, "Start LED WS2812 blink, red: %d, green: %d, blue: %d, period: %d ms", red, green, blue, period_ms);
        ESP_ERROR_CHECK(led_ws2812_blink_start(gpio, num, red, green, blue, period_ms));
    }

    return 0;
}

static void cmd_register_led_ws2812(void)
{
    led_ws2812_args.init = arg_lit0(NULL, "init", "Initialize LED WS2812");
    led_ws2812_args.create = arg_lit0(NULL, "create", "Create LED WS2812");
    led_ws2812_args.set = arg_lit0(NULL, "set", "Set LED WS2812");
    led_ws2812_args.blink_start = arg_lit0(NULL, "blink_start", "Start LED WS2812 blink");
    led_ws2812_args.gpio = arg_int0(NULL, "gpio", "<gpio>", "LED WS2812 GPIO");
    led_ws2812_args.num = arg_int0(NULL, "num", "<num>", "LED WS2812 number");
    led_ws2812_args.red = arg_int0(NULL, "red", "<red>", "LED WS2812 red");
    led_ws2812_args.green = arg_int0(NULL, "green", "<green>", "LED WS2812 green");
    led_ws2812_args.blue = arg_int0(NULL, "blue", "<blue>", "LED WS2812 blue");
    led_ws2812_args.period = arg_int0(NULL, "period", "<ms>", "LED WS2812 blink period");
    led_ws2812_args.fade = arg_int0(NULL, "fade", "<ms>", "LED WS2812 set fade");
    led_ws2812_args.end = arg_end(4);

    const esp_console_cmd_t led_ws2812_cmd = {
        .command = "led_ws2812",
        .help = "LED WS2812 command",
        .hint = NULL,
        .func = &led_ws2812_cmd_handler,
        .argtable = &led_ws2812_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&led_ws2812_cmd));
}


static struct {
    struct arg_lit *init;
    struct arg_int *sda;
    struct arg_int *scl;
    struct arg_int *read;
    struct arg_end *end;
} sensor_mpu6050_args;

static void sensor_mpu6050_read_task(void *arg)
{
    uint32_t delay_ms = (uint32_t) arg;

    mpu6050_acce_value_t acce_last = {0};
    mpu6050_gyro_value_t gyro_last = {0};

    while (1) {
        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(sensor_mpu6050_read(&acce, &gyro));

        if (acce.acce_x == acce_last.acce_x && acce.acce_y == acce_last.acce_y && acce.acce_z == acce_last.acce_z &&
                gyro.gyro_x == gyro_last.gyro_x && gyro.gyro_y == gyro_last.gyro_y && gyro.gyro_z == gyro_last.gyro_z) {
            continue;
        }

        acce_last = acce;
        gyro_last = gyro;

        ESP_LOGI(TAG, "MPU6050 acce: {x: %f, y: %f, z: %f}, gyro: {x: %f, y: %f, z: %f}",
                 acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    }
}

static esp_err_t sensor_mpu6050_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &sensor_mpu6050_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, sensor_mpu6050_args.end, argv[0]);
        return 1;
    }

    if (sensor_mpu6050_args.init->count) {
        ESP_LOGI(TAG, "Initialize MPU6050");
        sensor_mpu6050_config_t config = SENSOR_MPU6050_CONFIG_DEFAULT();

        if (sensor_mpu6050_args.sda->count) {
            config.i2c_sda = sensor_mpu6050_args.sda->ival[0];
        }

        if (sensor_mpu6050_args.scl->count) {
            config.i2c_scl = sensor_mpu6050_args.scl->ival[0];
        }

        ESP_ERROR_CHECK(sensor_mpu6050_init(&config));
    }

    if (sensor_mpu6050_args.read->count) {
        ESP_LOGI(TAG, "Read MPU6050");
        uint32_t delay_ms = sensor_mpu6050_args.read->ival[0];
        xTaskCreate(sensor_mpu6050_read_task, "sensor_mpu6050_read_task", 1024 * 4, (void *)delay_ms, 5, NULL);
    }

    return 0;
}

static void cmd_register_sensor_mpu6050(void)
{
    sensor_mpu6050_args.init = arg_lit0(NULL, "init", "Initialize MPU6050");
    sensor_mpu6050_args.sda = arg_int0(NULL, "sda", "<gpio_sda>", "MPU6050 SDA");
    sensor_mpu6050_args.scl = arg_int0(NULL, "scl", "<gpio_scl>", "MPU6050 SCL");
    sensor_mpu6050_args.read = arg_int0(NULL, "read", "<ms>", "Read MPU6050");
    sensor_mpu6050_args.end = arg_end(4);

    const esp_console_cmd_t sensor_mpu6050_cmd = {
        .command = "sensor_mpu6050",
        .help = "MPU6050 command",
        .hint = NULL,
        .func = &sensor_mpu6050_cmd_handler,
        .argtable = &sensor_mpu6050_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&sensor_mpu6050_cmd));
}

static struct {
    struct arg_lit *init;
    struct arg_int *gpio;
    struct arg_int *read;
    struct arg_end *end;
} sensor_vibration_args;

static void sensor_vibration_read_task(void *arg)
{
    uint32_t delay_ms = (uint32_t)arg;

    while (1) {
        uint32_t count;
        ESP_ERROR_CHECK(sensor_vibration_read(&count));

        if (count > 0) {
            ESP_LOGI(TAG, "Vibration Sensor count: %d", count);
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static esp_err_t sensor_vibration_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &sensor_vibration_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, sensor_vibration_args.end, argv[0]);
        return 1;
    }

    if (sensor_vibration_args.init->count) {
        ESP_LOGI(TAG, "Initialize Vibration Sensor");
        sensor_vibration_config_t config = SENSOR_VIBRATION_CONFIG_DEFAULT();

        if (sensor_vibration_args.gpio->count) {
            config.gpio_num = sensor_vibration_args.gpio->ival[0];
        }

        ESP_ERROR_CHECK(sensor_vibration_init(&config));
    }

    if (sensor_vibration_args.read->count) {
        uint32_t delay_ms = sensor_vibration_args.read->ival[0];
        ESP_LOGI(TAG, "Read Vibration Sensor, delay: %d", delay_ms);
        xTaskCreate(sensor_vibration_read_task, "sensor_vibration_read_task", 1024 * 4, (void *)delay_ms, 5, NULL);
    }

    return 0;
}

static void cmd_register_sensor_vibration(void)
{
    sensor_vibration_args.init = arg_lit0(NULL, "init", "Initialize Vibration Sensor");
    sensor_vibration_args.gpio = arg_int0(NULL, "gpio", "<gpio>", "Vibration Sensor GPIO");
    sensor_vibration_args.read = arg_int0(NULL, "read", "<ms>", "Read Vibration Sensor");
    sensor_vibration_args.end = arg_end(4);

    const esp_console_cmd_t sensor_vibration_cmd = {
        .command = "sensor_vibration",
        .help = "Vibration Sensor command",
        .hint = NULL,
        .func = &sensor_vibration_cmd_handler,
        .argtable = &sensor_vibration_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&sensor_vibration_cmd));
}

static struct {
    struct arg_lit *init;
    struct arg_lit *create;
    struct arg_int *gpio;
    struct arg_str *sensitivity;
    struct arg_end *end;
} touchpad_button_args;

static void touchpad_button_cb(gpio_num_t gpio, touchpad_button_event_t event)
{
    switch (event) {
        case TOUCHPAD_BUTTON_EVT_ON_PRESS:
            ESP_LOGI(TAG, "Touch Button[%d] Press", gpio);
            break;

        case TOUCHPAD_BUTTON_EVT_ON_RELEASE:
            ESP_LOGI(TAG, "Touch Button[%d] Release", gpio);
            break;

        case TOUCHPAD_BUTTON_EVT_ON_LONGPRESS:
            ESP_LOGI(TAG, "Touch Button[%d] LongPress", gpio);
            break;

        default:
            break;
    }
}

static esp_err_t touchpad_button_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &touchpad_button_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, touchpad_button_args.end, argv[0]);
        return 1;
    }

    if (touchpad_button_args.init->count) {
        ESP_LOGI(TAG, "Initialize Touch Button");
        touchpad_button_config_t config = TOUCHPAD_BUTTON_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(touchpad_button_init(&config));
    }

    if (touchpad_button_args.create->count) {
        ESP_LOGI(TAG, "Create Touch Button");

        gpio_num_t gpio = GPIO_NUM_NC;
        float sensitivity = 0.1;

        if (touchpad_button_args.gpio->count) {
            gpio = touchpad_button_args.gpio->ival[0];
        }

        if (touchpad_button_args.sensitivity->count) {
            sensitivity = atof(touchpad_button_args.sensitivity->sval[0]);
        }

        ESP_ERROR_CHECK(touchpad_button_create(gpio, sensitivity, touchpad_button_cb));
    }

    return 0;
}

static void cmd_register_touchpad_button(void)
{
    touchpad_button_args.init = arg_lit0(NULL, "init", "Initialize Touch Button");
    touchpad_button_args.create = arg_lit0(NULL, "create", "Create Touch Button");
    touchpad_button_args.gpio = arg_int0(NULL, "gpio", "<gpio>", "Touch Button GPIO");
    touchpad_button_args.sensitivity = arg_str0(NULL, "sensitivity", "<sensitivity>", "Touch Button sensitivity");
    touchpad_button_args.end = arg_end(4);

    const esp_console_cmd_t touchpad_button_cmd = {
        .command = "touchpad_button",
        .help = "Touch Button command",
        .hint = NULL,
        .func = &touchpad_button_cmd_handler,
        .argtable = &touchpad_button_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&touchpad_button_cmd));
}

static struct {
    struct arg_lit *init;
    struct arg_lit *create;
    struct arg_int *gpio;
    struct arg_lit *read;
    struct arg_end *end;
} touchpad_sensor_args;

static void touchpad_sensor_read_task(void *arg)
{
    gpio_num_t gpio_num = (gpio_num_t)arg;
    uint32_t value_last = 0;

    while (1) {
        uint32_t value;
        ESP_ERROR_CHECK(touchpad_sensor_read(gpio_num, &value));

        if (value > value_last * 1.1 || value < value_last * 0.9) {
            value_last = value;
            ESP_LOGI(TAG, "Touch Sensor read: %d, value: %d", gpio_num, value);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static esp_err_t touchpad_sensor_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &touchpad_sensor_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, touchpad_sensor_args.end, argv[0]);
        return 1;
    }

    gpio_num_t gpio = GPIO_NUM_NC;

    if (touchpad_sensor_args.init->count) {
        ESP_LOGI(TAG, "Initialize Touch Sensor");
        touchpad_sensor_config_t config = TOUCHPAD_SENSOR_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(touchpad_sensor_init(&config));
    }

    if (touchpad_sensor_args.gpio->count) {
        gpio = touchpad_sensor_args.gpio->ival[0];
    }

    if (touchpad_sensor_args.create->count) {
        ESP_LOGI(TAG, "Create Touch Sensor");
        ESP_ERROR_CHECK(touchpad_sensor_create(gpio));
    }

    if (touchpad_sensor_args.read->count) {
        ESP_LOGI(TAG, "Read Touch Sensor");
        xTaskCreate(touchpad_sensor_read_task, "touchpad_sensor_read_task", 1024 * 4, (void *)gpio, 5, NULL);
    }

    return 0;
}

static void cmd_register_touchpad_sensor(void)
{
    touchpad_sensor_args.init = arg_lit0(NULL, "init", "Initialize Touch Sensor");
    touchpad_sensor_args.create = arg_lit0(NULL, "create", "Create Touch Sensor");
    touchpad_sensor_args.gpio = arg_int0(NULL, "gpio", "<gpio>", "Touch Sensor GPIO");
    touchpad_sensor_args.read = arg_lit0(NULL, "read", "Read Touch Sensor");
    touchpad_sensor_args.end = arg_end(4);

    const esp_console_cmd_t touchpad_sensor_cmd = {
        .command = "touchpad_sensor",
        .help = "Touch Sensor command",
        .hint = NULL,
        .func = &touchpad_sensor_cmd_handler,
        .argtable = &touchpad_sensor_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&touchpad_sensor_cmd));
}


#define VOICE_COMMANDS_MAX 10
static const char *g_commands[VOICE_COMMANDS_MAX] = {
    "da kai dian deng",
    "guan bi dian deng",
};

static void voice_control_task(void *arg)
{
    int *audio_data = NULL;
    size_t audio_len = 0;

    while (1) {
        esp_err_t ret = speech_mic_record(&audio_data, &audio_len);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ear mic record failed, ret: %s", esp_err_to_name(ret));
            break;
        }


        for (int x = 0; x < audio_len / 16; x++) {
            int s1 = ((audio_data[x * 4] + audio_data[x * 4 + 1]) >> 13) & 0x0000FFFF;
            int s2 = ((audio_data[x * 4 + 2] + audio_data[x * 4 + 3]) << 3) & 0xFFFF0000;
            audio_data[x] = s1 | s2;
        }


        bool wakenet_state;
        ret = speech_recognition_wakenet((const int16_t *)audio_data, audio_len, &wakenet_state);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ear recognition wakenet failed, ret: %s", esp_err_to_name(ret));
            break;
        }

        if (!wakenet_state) {
            continue;
        }

        while (true) {
            ret = speech_mic_record(&audio_data, &audio_len);

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ear mic record failed, ret: %s", esp_err_to_name(ret));
                break;
            }

            int commands_id;
            ret = speech_recognition_command((const int16_t *)audio_data, audio_len, &commands_id);

            if (ret == ESP_ERR_TIMEOUT) {
                break;
            } else if (ret == ESP_ERR_NOT_FINISHED) {
                continue;
            }

            ESP_LOGI(TAG, "ear recognition command: %s", g_commands[commands_id]);
        }
    }

    vTaskDelete(NULL);
}

static struct {
    struct arg_lit *init;
    struct arg_int *gpio_bclk;
    struct arg_int *gpio_ws;
    struct arg_int *gpio_din;
    struct arg_str *commands;
    struct arg_end *end;
} voice_control_args;

static esp_err_t voice_control_cmd_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &voice_control_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, voice_control_args.end, argv[0]);
        return 1;
    }

    if (voice_control_args.init->count) {
        ESP_LOGI(TAG, "Initialize Voice Control");

        speech_mic_config_t config = SPEECH_MIC_CONFIG_DEFAULT();

        if (voice_control_args.gpio_bclk->count) {
            config.mic_gpio.i2s_bclk = voice_control_args.gpio_bclk->ival[0];
        }

        if (voice_control_args.gpio_ws->count) {
            config.mic_gpio.i2s_ws = voice_control_args.gpio_ws->ival[0];
        }

        if (voice_control_args.gpio_din->count) {
            config.mic_gpio.i2s_din = voice_control_args.gpio_din->ival[0];
        }

        ESP_ERROR_CHECK(speech_mic_init(&config));
        ESP_ERROR_CHECK(speech_recognition_init());

        esp_mn_commands_clear();
        for (int i = 0; g_commands[i] && i < VOICE_COMMANDS_MAX; i++) {
            esp_mn_commands_add(i, (char *)g_commands[i]);
        }
        esp_mn_commands_update();

        xTaskCreate(voice_control_task, "voice_control_task", 1024 * 4, NULL, 5, NULL);
    }

    if (voice_control_args.commands->count) {
        for (int i = 0; i < VOICE_COMMANDS_MAX; i++) {
            if (!g_commands[i]) {
                g_commands[i] = voice_control_args.commands->sval[0];
                esp_mn_commands_add(i, (char *)g_commands[i]);
                esp_mn_commands_update();
                break;
            }
        }
    }

    return 0;
}

static void cmd_register_voice_control(void)
{
    voice_control_args.init = arg_lit0(NULL, "init", "Initialize Voice Control");
    voice_control_args.gpio_bclk = arg_int0(NULL, "gpio_bclk", "<gpio>", "Voice Control BCLK GPIO");
    voice_control_args.gpio_ws = arg_int0(NULL, "gpio_ws", "<gpio>", "Voice Control WS GPIO");
    voice_control_args.gpio_din = arg_int0(NULL, "gpio_din", "<gpio>", "Voice Control DIN GPIO");
    voice_control_args.commands = arg_str0(NULL, "commands", "<text>", "Voice Control commands text");
    voice_control_args.end = arg_end(4);

    const esp_console_cmd_t voice_control_cmd = {
        .command = "voice_control",
        .help = "Voice Control command",
        .hint = NULL,
        .func = &voice_control_cmd_handler,
        .argtable = &voice_control_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&voice_control_cmd));
}

void app_main(void)
{
    storage_init();

    ESP_LOGW(TAG, "This is a Wi-Fi CSI console example, please use 'help' command to get more information");

    /**
     * @brief Register serial command
     */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = ">";
    repl_config.max_cmdline_length = 1024;

    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));

    /**
     * @brief Register system command
     * 
     * @test help
     *       version
     *       restart
     *       heap
     *       log -t 'TAG' -l INFO
     */
    cmd_register_system();
    
    /**
     * @brief Construct a new cmd register wifi config object
     * 
     * @test wifi_config --init --ssid "ssid" --password "password"
     */
    cmd_register_wifi_config();

    /**
     * @brief Construct a new cmd register ir object
     * 
     * @test ir --init --tx_gpio 47 --rx_gpio 48
     *       ir --send "{9013,4515},{564,567},{538,570},{561,568},{537,594},{537,569},{561,570},{537,1700},{562,569},{536,570},{562,569},{537,1701},{561,568},{537,595},{537,568},{562,570},{536,569},{563,1700},{536,1701},{562,569},{536,571},{561,568},{536,596},{536,569},{536,596},{536,569},{562,569},{536,570},{561,570},{536,1703},{560,1700},{536,596},{536,569},{536,0}"
     */
    cmd_register_ir();

    /**
     * @brief Construct a new cmd register led pwm object
     *
     * @test led_pwm --init --create --gpio 13
     *       led_pwm --set --gpio 13 --duty 10 --fade_ms 500
     *       led_pwm --blink_start --gpio 13 --duty 10 --period 500
     */
    cmd_register_led_pwm();

    /**
     * @brief Construct a new cmd register led ws2812 object
     *
     * @test led_ws2812 --init --create --gpio 9 --num 6
     *       led_ws2812 --set --gpio 9 --num 6 --red 32 --green 32 --blue 32 --fade 500
     *       led_ws2812 --blink_start --gpio 9 --num 6 --red 32 --green 32 --blue 32 --period 500
     */
    cmd_register_led_ws2812();

    /**
     * @brief Construct a new cmd register touchpad sensor object
     *
     * @test touchpad_sensor --init --create --gpio 4
     *       touchpad_sensor --read --gpio 4
     */
    cmd_register_touchpad_sensor();
    
    /**
     * @brief Construct a new cmd register touchpad button object
     *
     * @test touchpad_button --init --create --gpio 1
     */
    cmd_register_touchpad_button();
    
    /**
     * @brief Construct a new cmd register sensor mpu6050 object
     *
     * @test sensor_mpu6050 --init --sda 10 --scl 12
     *       sensor_mpu6050 --read 1000
     */
    cmd_register_sensor_mpu6050();

    /**
     * @brief Construct a new cmd register sensor vibration object
     *
     * @test sensor_vibration --init --gpio 2
     *       sensor_vibration --read 1000
     */
    cmd_register_sensor_vibration();

    /**
     * @brief Construct a new cmd register voice control object
     *
     * @test voice_control --init --gpio_bclk 39 --gpio_ws 38 --gpio_din 40
     *       voice_control --commands_text "da kai dian deng"
     */
    cmd_register_voice_control();
    
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
