#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

#include "buzzer.h"


#define LEDC_CHANNEL        LEDC_CHANNEL_6

// Global variables
static int g_buzzer_volume            = 50;                     // Volume range: 0 (mute) to 100 (max volume)
static buzzer_play_mode_t g_play_mode = PLAY_MODE_INVALID;   // Default play mode
static buzzer_beep_t g_beep_type      = BEEP_STARTUP;           // Default beep type
static bool g_paused_flag             = false;                  // Playback pause flag
static buzzer_song_t g_current_song   = SONG_CASTLE_IN_THE_SKY; // Currently playing song number

static buzzer_config_t *g_buzzer_config = NULL;

static const char *TAG = "buzzer";

// Tone frequency table
typedef struct {
    const char *note;
    uint32_t frequency;
} buzzer_tone_t;

static const buzzer_tone_t buzzer_tones[] = {
    {"C0", 16}, {"C#0", 17}, {"D0", 18}, {"D#0", 19}, {"E0", 21}, {"F0", 22}, {"F#0", 23}, {"G0", 24},
    {"G#0", 26}, {"A0", 28}, {"A#0", 29}, {"B0", 31}, {"C1", 33}, {"C#1", 35}, {"D1", 37}, {"D#1", 39},
    {"E1", 41}, {"F1", 44}, {"F#1", 46}, {"G1", 49}, {"G#1", 52}, {"A1", 55}, {"A#1", 58}, {"B1", 62},
    {"C2", 65}, {"C#2", 69}, {"D2", 73}, {"D#2", 78}, {"E2", 82}, {"F2", 87}, {"F#2", 92}, {"G2", 98},
    {"G#2", 104}, {"A2", 110}, {"A#2", 117}, {"B2", 123}, {"C3", 131}, {"C#3", 139}, {"D3", 147}, {"D#3", 156},
    {"E3", 165}, {"F3", 175}, {"F#3", 185}, {"G3", 196}, {"G#3", 208}, {"A3", 220}, {"A#3", 233}, {"B3", 247},
    {"C4", 262}, {"C#4", 277}, {"D4", 294}, {"D#4", 311}, {"E4", 330}, {"F4", 349}, {"F#4", 370}, {"G4", 392},
    {"G#4", 415}, {"A4", 440}, {"A#4", 466}, {"B4", 494}, {"C5", 523}, {"C#5", 554}, {"D5", 587}, {"D#5", 622},
    {"E5", 659}, {"F5", 698}, {"F#5", 740}, {"G5", 784}, {"G#5", 831}, {"A5", 880}, {"A#5", 932}, {"B5", 988},
    {"C6", 1047}, {"C#6", 1109}, {"D6", 1175}, {"D#6", 1245}, {"E6", 1319}, {"F6", 1397}, {"F#6", 1480}, {"G6", 1568},
    {"G#6", 1661}, {"A6", 1760}, {"A#6", 1865}, {"B6", 1976}, {"C7", 2093}, {"C#7", 2217}, {"D7", 2349}, {"D#7", 2489},
    {"E7", 2637}, {"F7", 2794}, {"F#7", 2960}, {"G7", 3136}, {"G#7", 3322}, {"A7", 3520}, {"A#7", 3729}, {"B7", 3951},
    {"C8", 4186}
};

// Find the frequency corresponding to a note
static uint32_t buzzer_get_frequency(const char *note)
{
    for (size_t i = 0; i < sizeof(buzzer_tones) / sizeof(buzzer_tones[0]); i++) {
        if (strcmp(buzzer_tones[i].note, note) == 0) {
            return buzzer_tones[i].frequency;
        }
    }
    return 0; // Return 0 if not found
}


// Set volume, range 0~100
esp_err_t buzzer_set_volume(uint8_t volume)
{
    if (volume > 100) {
        ESP_LOGE(TAG, "Invalid volume: %d", volume);
        return ESP_ERR_INVALID_ARG;
    }

    g_buzzer_volume = volume;

    return ESP_OK;
}

// Toggle pause state
esp_err_t buzzer_pause_song(bool enable)
{
    g_paused_flag = enable;

    return ESP_OK;
}

// Play a single note
static void buzzer_play_note_pwm(uint32_t frequency, uint32_t duration_ms)
{
    if (frequency == 0 || g_paused_flag) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        return;
    }

    // Set frequency
    if (ledc_set_freq(g_buzzer_config->speed_mode, g_buzzer_config->timer_num, frequency) != ESP_OK) {
        ESP_LOGE("buzzer_play_note_pwm", "Failed to set frequency");
        return;
    }

    // Map volume from 0~100 to duty cycle 0~1023
    uint32_t duty = (g_buzzer_volume * 1023) / 100;

    // Set duty cycle (volume)
    if (ledc_set_duty(g_buzzer_config->speed_mode, LEDC_CHANNEL, duty) != ESP_OK) {
        ESP_LOGE("buzzer_play_note_pwm", "Failed to set duty cycle");
        return;
    }
    if (ledc_update_duty(g_buzzer_config->speed_mode, LEDC_CHANNEL) != ESP_OK) {
        ESP_LOGE("buzzer_play_note_pwm", "Failed to update duty cycle");
        return;
    }

    // Play for the duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Turn off the note
    if (ledc_set_duty(g_buzzer_config->speed_mode, LEDC_CHANNEL, 0) != ESP_OK) {
        ESP_LOGE("buzzer_play_note_pwm", "Failed to reset duty cycle");
        return;
    }
    if (ledc_update_duty(g_buzzer_config->speed_mode, LEDC_CHANNEL) != ESP_OK) {
        ESP_LOGE("buzzer_play_note_pwm", "Failed to update duty cycle");
        return;
    }
}

// Play a melody string
static void buzzer_play_melody_pwm(const char *song)
{
    if (song == NULL || g_paused_flag) return;

    char *song_copy = strdup(song); // Create a copy of the string
    if (song_copy == NULL) {
        ESP_LOGE("buzzer_play_melody_pwm", "Memory allocation failed");
        return;
    }

    char *token = strtok(song_copy, ";");

    buzzer_play_mode_t play_mode = g_play_mode;

    while (token != NULL && !g_paused_flag && play_mode == g_play_mode) {
        char note_str[8] = {0};
        uint32_t time = 0, duration = 0;
        uint32_t instrument = 0; // Not used, but needs to be parsed

        // Parse format: time note duration instrument
        int parsed = sscanf(token, "%lu %7s %lu %ld", &time, note_str, &duration, &instrument);
        if (parsed >= 3) {
            uint32_t freq = buzzer_get_frequency(note_str);
            if (freq == 0) {
                ESP_LOGW("buzzer_play_melody_pwm", "Unknown note: %s", note_str);
            }
            buzzer_play_note_pwm(freq, duration * 200); // Adjust duration
            vTaskDelay(pdMS_TO_TICKS(50));              // Pause between notes
        } else {
            ESP_LOGW("buzzer_play_melody_pwm", "Failed to parse note: %s", token);
        }

        token = strtok(NULL, ";");
    }

    free(song_copy);
}

// Melody data for ten classic children's songs
static const char *buzzer_songs[SONG_NUM_MAX] = {
    // SONG_CASTLE_IN_THE_SKY
    "0 E4 1 1;2 G4 1 1;4 A4 2 1;6 G4 1 1;8 E4 1 1;10 D4 2 1;12 C4 4 1",
    // SONG_HAPPY_BIRTHDAY
    "0 G4 1 1;2 G4 1 1;4 A4 2 1;6 G4 2 1;8 C5 2 1;10 B4 4 1;14 G4 1 1;16 G4 1 1;18 A4 2 1;20 G4 2 1;22 D5 2 1;24 C5 4 1",
    // SONG_PAINTER
    "0 C4 1 1;2 D4 1 1;4 E4 2 1;6 C4 2 1;8 E4 1 1;10 F4 1 1;12 G4 4 1",
    // SONG_TWO_TIGERS
    "0 G4 1 1;2 A4 1 1;4 G4 1 1;6 A4 1 1;8 G4 1 1;10 D4 2 1;12 G4 2 1;14 D4 4 1",
    // SONG_TWINKLE_TWINKLE
    "0 C4 1 1;2 C4 1 1;4 G4 1 1;6 G4 1 1;8 A4 1 1;10 A4 1 1;12 G4 2 1;14 F4 1 1;16 F4 1 1;18 E4 1 1;20 E4 1 1;22 D4 1 1;24 D4 1 1;26 C4 2 1",
    // SONG_JINGLE_BELLS
    "0 E4 1 1;2 E4 1 1;4 E4 2 1;6 E4 1 1;8 E4 1 1;10 E4 2 1;12 E4 1 1;14 G4 1 1;16 C4 1 1;18 D4 1 1;20 E4 4 1",
    // SONG_LITTLE_BEE
    "0 C4 1 1;2 E4 1 1;4 G4 2 1;6 G4 1 1;8 E4 1 1;10 C4 2 1;12 G4 2 1;14 G4 4 1",
    // SONG_NUMBER_SONG
    "0 C4 1 1;2 D4 1 1;4 E4 1 1;6 F4 1 1;8 G4 1 1;10 A4 1 1;12 B4 1 1;14 C5 2 1",
    // SONG_LONDON_BRIDGE
    "0 C4 1 1;2 D4 1 1;4 E4 1 1;6 C4 1 1;8 E4 1 1;10 D4 1 1;12 C4 2 1",
    // SONG_MARY_LAMB
    "0 E4 1 1;2 D4 1 1;4 C4 1 1;6 D4 1 1;8 E4 1 1;10 E4 1 1;12 E4 2 1;14 D4 1 1;16 D4 1 1;18 D4 2 1;20 E4 1 1;22 G4 1 1;24 G4 2 1"
};

// Play the specified song by number
esp_err_t buzzer_play_song(buzzer_song_t song_number, buzzer_play_mode_t play_mode)
{
    if (song_number < SONG_CASTLE_IN_THE_SKY || song_number > SONG_NUM_MAX) {
        ESP_LOGW("buzzer_play_song", "Invalid song number: %d", song_number);
        return ESP_ERR_INVALID_ARG;
    }

    g_play_mode = play_mode;
    g_current_song = song_number;
    g_paused_flag = false;

    return ESP_OK;
}

// Play the list of songs
static void buzzer_play_songs_task(void *arg)
{
    while (1) {
        if (g_paused_flag || g_play_mode == PLAY_MODE_INVALID) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        switch (g_play_mode) {
            case PLAY_MODE_CONTINUOUS:
                for (buzzer_song_t i = SONG_CASTLE_IN_THE_SKY; i / 3 < SONG_NUM_MAX; i++) {
                    g_current_song = i / 3;
                    buzzer_play_melody_pwm(buzzer_songs[i / 3]);
                    if (g_paused_flag || g_play_mode != PLAY_MODE_CONTINUOUS) {
                            break;
                    }
                }
                break;

            case PLAY_MODE_REPEAT_ONE:
                buzzer_play_melody_pwm(buzzer_songs[g_current_song]);
                break;

            case PLAY_MODE_SHUFFLE:
                g_current_song = rand() % (SONG_NUM_MAX);
                buzzer_play_melody_pwm(buzzer_songs[g_current_song]);
                break;

            case PLAY_MODE_BEEP: {
                switch (g_beep_type) {
                    case BEEP_TIP:
                        buzzer_play_note_pwm(1000, 100); // 1kHz, 100ms
                        break;
                    case BEEP_ERROR:
                        for (int i = 0; i < 3; i++) {
                            buzzer_play_note_pwm(400, 200); // 400Hz, 200ms
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        break;
                    case BEEP_SUCCESS:
                        buzzer_play_melody_pwm("0 G4 1 1;1 C5 1 1;2 E5 1 1"); // Ascending scale
                        break;
                    case BEEP_WARNING:
                        for (int i = 0; i < 2; i++) {
                            buzzer_play_note_pwm(600, 150); // 600Hz, 150ms
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                        break;
                    case BEEP_STARTUP:
                        buzzer_play_melody_pwm("0 C4 1 1;1 E4 1 1;2 G4 1 1"); // Short ascending scale
                        break;
                    case BEEP_SHUTDOWN:
                        buzzer_play_melody_pwm("0 G4 1 1;1 E4 1 1;2 C4 1 1"); // Short descending scale
                        break;
                    case BEEP_MESSAGE:
                        buzzer_play_note_pwm(1200, 150); // 1.2kHz, 150ms
                        vTaskDelay(pdMS_TO_TICKS(50));
                        buzzer_play_note_pwm(1400, 150); // 1.4kHz, 150ms
                        break;
                    case BEEP_CONFIRM:
                        buzzer_play_note_pwm(800, 200); // 800Hz, 200ms
                        break;
                    default:
                        ESP_LOGW("buzzer_play_beep", "Unknown notification sound type: %d", g_beep_type);
                        break;
                }

                g_play_mode = PLAY_MODE_INVALID;
                break;
            }

            default:
                ESP_LOGW("buzzer_play_songs", "Unknown play mode");
                break;
        }

    }
}

// Play notification sounds based on the given type
esp_err_t buzzer_play_beep(buzzer_beep_t beep_type)
{
    g_paused_flag = false;
    g_beep_type = beep_type;
    g_play_mode = PLAY_MODE_BEEP;

    return ESP_OK;
}


// Initialize the LEDC module for PWM
esp_err_t buzzer_init(const buzzer_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = config->speed_mode,
        .timer_num        = config->timer_num,
        .duty_resolution  = config->duty_resolution,
        .freq_hz          = config->freq_hz,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = config->speed_mode,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = config->timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = config->gpio_num,
        .duty           = 0, // Initial duty cycle is 0
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    if (g_buzzer_config == NULL) {
        g_buzzer_config = (buzzer_config_t *)calloc(1, sizeof(buzzer_config_t));
        if (g_buzzer_config == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed");
            return ESP_ERR_NO_MEM;
        }
    }

    memcpy(g_buzzer_config, config, sizeof(buzzer_config_t));

    buzzer_set_volume(config->volume);

    xTaskCreate(buzzer_play_songs_task, "buzzer_play_songs", 4096, NULL, 5, NULL);

    return ESP_OK;
}

#if 0
void app_main(void) {
    // Initialize hardware
    buzzer_ledc_init();

    // Play startup sound
    ESP_LOGI(TAG, "Playing startup sound...");
    buzzer_play_beep(BEEP_STARTUP);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Playing warning sound...");
    buzzer_play_beep(BEEP_WARNING);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Playing error sound...");
    buzzer_play_beep(BEEP_ERROR);

    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Playing success sound...");
    buzzer_play_beep(BEEP_SUCCESS);


    // // Example: change volume and play mode
    // vTaskDelay(pdMS_TO_TICKS(5000));
    // buzzer_set_volume(80); // Increase volume to 80
    // g_play_mode = PLAY_MODE_REPEAT_ONE; // Set to repeat single

    // // Example: switch to next song
    // vTaskDelay(pdMS_TO_TICKS(10000));
    // g_current_song = SONG_HAPPY_BIRTHDAY; // Switch to "Happy Birthday"

    // // Example: pause playback
    // vTaskDelay(pdMS_TO_TICKS(10000));
    // buzzer_toggle_pause(); // Pause playback

    // // Example: resume playback
    // vTaskDelay(pdMS_TO_TICKS(5000));
    // buzzer_toggle_pause(); // Resume playback

    // Other business logic...
}
#endif
