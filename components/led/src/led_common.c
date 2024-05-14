#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>

#include "esp_log.h"
#include "led_common.h"




esp_err_t led_gamma_table_create(uint32_t *data, size_t size, uint32_t range, float correction)
{
    ESP_LOGW("TAG", "Creating gamma table, size: %d, range: %d, correction: %f", size, range, correction);

    /**
     * @brief gamma curve formula: y=a*x^(1/gm)
     * x ∈ (0,(GAMMA_TABLE_SIZE-1)/GAMMA_TABLE_SIZE)
     * a = GAMMA_TABLE_SIZE
     */
    for (int i = 0; i < size; i++) {
        float gamma_value = powf(i / (float)size, 1.0 / correction) * range;
        data[i] = (uint32_t)(gamma_value + 0.5);
    }

    data[size - 1] = range;

// ESP_LOG_BUFFER_HEXDUMP("gamma_table", data, size * sizeof(uint32_t), ESP_LOG_INFO);
    return ESP_OK;
}

esp_err_t led_hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value,
                      uint8_t *red, uint8_t *green, uint8_t *blue)
{
    uint16_t hi = (hue / 60) % 6;
    uint16_t F = 100 * hue / 60 - 100 * hi;
    uint16_t P = value * (100 - saturation) / 100;
    uint16_t Q = value * (10000 - F * saturation) / 10000;
    uint16_t T = value * (10000 - saturation * (100 - F)) / 10000;

    switch (hi) {
        case 0:
            *red   = value;
            *green = T;
            *blue  = P;
            break;

        case 1:
            *red   = Q;
            *green = value;
            *blue  = P;
            break;

        case 2:
            *red   = P;
            *green = value;
            *blue  = T;
            break;

        case 3:
            *red   = P;
            *green = Q;
            *blue  = value;
            break;

        case 4:
            *red   = T;
            *green = P;
            *blue  = value;
            break;

        case 5:
            *red   = value;
            *green = P;
            *blue  = Q;
            break;

        default:
            return ESP_FAIL;
    }

    *red   = *red * 255 / 100;
    *green = *green * 255 / 100;
    *blue  = *blue * 255 / 100;

    return ESP_OK;
}


esp_err_t led_rgb2hsv(uint8_t red, uint8_t green, uint8_t blue,
                      uint16_t *h, uint8_t *s, uint8_t *v)
{
    float hue, saturation, value;
    float m_max = MAX(red, MAX(green, blue));
    float m_min = MIN(red, MIN(green, blue));
    float m_delta = m_max - m_min;

    value = m_max / 255.0f;

    if (m_delta == 0) {
        hue = 0;
        saturation = 0;
    } else {
        saturation = m_delta / m_max;

        if (red == m_max) {
            hue = (green - blue) / m_delta;
        } else if (green == m_max) {
            hue = 2 + (blue - red) / m_delta;
        } else {
            hue = 4 + (red - green) / m_delta;
        }

        hue = hue * 60;

        if (hue < 0) {
            hue = hue + 360;
        }
    }

    *h = (int)(hue + 0.5);
    *s = (int)(saturation * 100 + 0.5);
    *v = (int)(value * 100 + 0.5);

    return ESP_OK;
}
