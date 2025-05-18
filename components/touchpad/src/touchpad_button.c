#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "touch_element/touch_button.h"
#include "esp_log.h"

#include "touchpad_button.h"
#include <string.h>

static const char *TAG = "touch_button";
// #define CONFIG_TOUCH_ELEM_EVENT 1

typedef struct {
    gpio_num_t gpio;
    touchpad_button_cb_t cb;
    touch_button_handle_t handle;
} touchpad_info_t;

static touchpad_info_t **g_touchpad_info;
static touchpad_button_config_t *g_touchpad_button_config;

static int touchpad_button_gpio_to_channel(gpio_num_t gpio)
{
    int channel = -1;

    for (int i = 0; i < g_touchpad_button_config->max_channel; i++) {
        if (g_touchpad_info[i] && g_touchpad_info[i]->gpio == gpio) {
            channel = i;
            break;
        }
    }

    return channel;
}

static void button_handler_cb(touch_button_handle_t out_handle, touch_button_message_t *out_message, void *arg)
{
    (void) out_handle; //Unused

    if (out_message->event == TOUCH_BUTTON_EVT_ON_PRESS) {
        ESP_LOGI(TAG, "Button[%d] Press", (int)arg);
    } else if (out_message->event == TOUCH_BUTTON_EVT_ON_RELEASE) {
        ESP_LOGI(TAG, "Button[%d] Release", (int)arg);
    } else if (out_message->event == TOUCH_BUTTON_EVT_ON_LONGPRESS) {
        ESP_LOGI(TAG, "Button[%d] LongPress", (int)arg);
    }

    int channel = touchpad_button_gpio_to_channel((gpio_num_t)arg);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio is not initialized");
        return;
    }

    if (g_touchpad_info[channel]->cb) {
        g_touchpad_info[channel]->cb((gpio_num_t)arg, out_message->event);
    }
}

esp_err_t touchpad_button_create(gpio_num_t gpio, float sensitivity, touchpad_button_cb_t cb)
{
    ESP_LOGI(TAG, "Touch button create, gpio: %d, sensitivity: %f", gpio, sensitivity);

    int channel = -1;

    for (int i = 0; i < g_touchpad_button_config->max_channel; i++) {
        if (g_touchpad_info[i] == NULL) {
            channel = i;
            break;
        } else if (g_touchpad_info[i]->gpio == gpio) {
            channel = i;
            break;
        }
    }

    if (channel == -1) {
        ESP_LOGE(TAG, "Touch button channel is full");
        return ESP_FAIL;
    }

    touchpad_info_t *touchpad_info = g_touchpad_info[channel] = calloc(1, sizeof(touchpad_info_t));
    touchpad_info->gpio = gpio;
    touchpad_info->cb = cb;

    touch_button_config_t button_config = {
        .channel_num = gpio,
        .channel_sens = sensitivity,
    };

    /* Create Touch buttons */
    ESP_ERROR_CHECK(touch_button_create(&button_config, &(touchpad_info->handle)));
    /* Subscribe touch button events (On Press, On Release, On LongPress) */
    ESP_ERROR_CHECK(touch_button_subscribe_event(touchpad_info->handle,
                    TOUCH_ELEM_EVENT_ON_PRESS | TOUCH_ELEM_EVENT_ON_RELEASE | TOUCH_ELEM_EVENT_ON_LONGPRESS,
                    (void *)gpio));
#ifdef CONFIG_TOUCH_ELEM_EVENT
    /* Set EVENT as the dispatch method */
    ESP_ERROR_CHECK(touch_button_set_dispatch_method(touchpad_info->handle, TOUCH_ELEM_DISP_EVENT));
#else
    /* Set EVENT as the dispatch method */
    ESP_ERROR_CHECK(touch_button_set_dispatch_method(touchpad_info->handle, TOUCH_ELEM_DISP_CALLBACK));
    /* Register a handler function to handle event messages */
    ESP_ERROR_CHECK(touch_button_set_callback(touchpad_info->handle, button_handler_cb));
#endif
    /* Set LongPress event trigger threshold time */
    ESP_ERROR_CHECK(touch_button_set_longpress(touchpad_info->handle, g_touchpad_button_config->longpress_ms));

    // TODO: Start touch element library if not started yet
    if (gpio == GPIO_NUM_9) {
        touch_element_start();
        ESP_LOGI(TAG, "Touch element library start");
    }

    return ESP_OK;
}

esp_err_t touchpad_button_delete(gpio_num_t gpio)
{
    int channel = touchpad_button_gpio_to_channel(gpio);

    if (channel == -1) {
        ESP_LOGE(TAG, "gpio is not initialized");
        return ESP_ERR_INVALID_ARG;
    }

    if (g_touchpad_info[channel]->handle != NULL) {
        // touch_button_unsubscribe_event(g_touchpad_info[channel]->handle, TOUCH_ELEM_EVENT_ON_PRESS | TOUCH_ELEM_EVENT_ON_RELEASE | TOUCH_ELEM_EVENT_ON_LONGPRESS);
        touch_button_delete(g_touchpad_info[channel]->handle);
        g_touchpad_info[channel]->handle = NULL;
    }

    free(g_touchpad_info[channel]);
    g_touchpad_info[channel] = NULL;

    return ESP_OK;
}

#if CONFIG_TOUCH_ELEM_EVENT
static void button_handler_task(void *arg)
{
    (void) arg; //Unused
    touch_elem_message_t element_message;

    while (1) {
        /* Waiting for touch element messages */
        touch_element_message_receive(&element_message, portMAX_DELAY);

        if (element_message.element_type != TOUCH_ELEM_TYPE_BUTTON) {
            continue;
        }

        /* Decode message */
        const touch_button_message_t *button_message = touch_button_get_message(&element_message);

        if (button_message->event == TOUCH_BUTTON_EVT_ON_PRESS) {
            ESP_LOGI(TAG, "Button[%d] Press", (int)element_message.arg);
        } else if (button_message->event == TOUCH_BUTTON_EVT_ON_RELEASE) {
            ESP_LOGI(TAG, "Button[%d] Release", (int)element_message.arg);
        } else if (button_message->event == TOUCH_BUTTON_EVT_ON_LONGPRESS) {
            ESP_LOGI(TAG, "Button[%d] LongPress", (int)element_message.arg);
        }
    }
}
#endif

esp_err_t touchpad_button_init(const touchpad_button_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    g_touchpad_button_config = (touchpad_button_config_t *)calloc(1, sizeof(touchpad_button_config_t));
    memcpy(g_touchpad_button_config, config, sizeof(touchpad_button_config_t));
    g_touchpad_info = calloc(g_touchpad_button_config->max_channel, sizeof(touchpad_info_t *));

    /* Initialize Touch Element library */
    touch_elem_global_config_t global_config = TOUCH_ELEM_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_element_install(&global_config));
    ESP_LOGI(TAG, "Touch element library installed");

    touch_button_global_config_t button_global_config = TOUCH_BUTTON_GLOBAL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(touch_button_install(&button_global_config));


#ifdef CONFIG_TOUCH_ELEM_EVENT
    /* Create a handler task to handle event messages */
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * 1024, NULL, 5, NULL);
#endif


    return ESP_OK;
}

esp_err_t touchpad_button_deinit()
{
    if (g_touchpad_button_config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < g_touchpad_button_config->max_channel; i++) {
        if (g_touchpad_info[i] != NULL) {
            if (g_touchpad_info[i]->handle != NULL) {
                // touch_button_unsubscribe_event(g_touchpad_info[i]->handle, TOUCH_ELEM_EVENT_ON_PRESS | TOUCH_ELEM_EVENT_ON_RELEASE | TOUCH_ELEM_EVENT_ON_LONGPRESS);
                touch_button_delete(g_touchpad_info[i]->handle);
            }

            free(g_touchpad_info[i]);
        }
    }

    free(g_touchpad_info);
    g_touchpad_info = NULL;

    free(g_touchpad_button_config);
    g_touchpad_button_config = NULL;

    touch_element_stop();
    ESP_LOGI(TAG, "Touch element library stop");

    return ESP_OK;
}
