#include "tasks.h"
#include "system_data.h"
#include "config.h"
#include "hardware.h"
#include "mpu6050.h"
#include "mqtt_handler.h"
#include "sms_handler.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TASKS";

// GPS Kalman filter - declare extern from nmea_parser component
extern void *gps_kalman_filter_create(float q, float r, float p);
extern void gps_kalman_filter_update(void *handle, float lat, float lng, float *filtered_lat, float *filtered_lng);

static void *gps_kf_handle = NULL;

// Helper function: Calculate distance between two GPS coordinates
static float calculate_distance(float lat1, float lng1, float lat2, float lng2)
{
    const float R = 6371000.0f; // Earth radius in meters
    float lat1_rad = lat1 * M_PI / 180.0f;
    float lng1_rad = lng1 * M_PI / 180.0f;
    float lat2_rad = lat2 * M_PI / 180.0f;
    float lng2_rad = lng2 * M_PI / 180.0f;

    float dlat = lat2_rad - lat1_rad;
    float dlng = lng2_rad - lng1_rad;

    float a = sinf(dlat / 2) * sinf(dlat / 2) +
              cosf(lat1_rad) * cosf(lat2_rad) *
                  sinf(dlng / 2) * sinf(dlng / 2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));

    return R * c;
}

// Helper: Set buzzer state
static void set_buzzer(bool state)
{
    gpio_set_level(BUZZER_PIN, state ? 1 : 0);
}

// Helper: Play beep pattern
static void play_beep_pattern(int count, int on_time_ms, int off_time_ms)
{
    for (int i = 0; i < count; i++)
    {
        set_buzzer(true);
        vTaskDelay(pdMS_TO_TICKS(on_time_ms));
        set_buzzer(false);
        if (i < count - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(off_time_ms));
        }
    }
}

// Helper: Read battery voltage
static float read_battery_voltage(void)
{
    int adc_raw = 0;
    float vout_sum = 0.0f;

    for (int i = 0; i < 10; i++)
    {
        adc_oneshot_read(g_adc_handle, ADC_CHANNEL_6, &adc_raw);
        float vout = (adc_raw / 4095.0f) * 3.3f;
        vout_sum += vout;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float voltage = (vout_sum / 10.0f) * 6.0f * 1.05f;

    // Constrain
    if (voltage < BATTERY_LOW_VOLTAGE)
        voltage = BATTERY_LOW_VOLTAGE;
    if (voltage > BATTERY_HIGH_VOLTAGE)
        voltage = BATTERY_HIGH_VOLTAGE;

    return voltage;
}

// Helper: Calculate battery percentage
static int calc_battery_percent(float voltage)
{
    int percentage = ((voltage - BATTERY_LOW_VOLTAGE) /
                      (BATTERY_HIGH_VOLTAGE - BATTERY_LOW_VOLTAGE)) *
                     100;

    if (percentage < 0)
        percentage = 0;
    if (percentage > 100)
        percentage = 100;

    return percentage;
}

// Helper: GPS sleep/wake commands
static void gps_put_to_sleep(void)
{
    bool already_sleeping = false;
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        already_sleeping = g_system_state.gps_sleep_state;
        if (!already_sleeping)
        {
            g_system_state.gps_sleep_state = true;
        }
        xSemaphoreGive(g_state_mutex);
    }

    if (!already_sleeping)
    {
        const char *cmd = "$PCAS05,1*1E\r\n";
        uart_write_bytes(GPS_UART_NUM, cmd, strlen(cmd));
        ESP_LOGI("GPS", "Sleep cmd sent");
    }
}

static void gps_wake_from_sleep(void)
{
    bool already_awake = false;
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        already_awake = !g_system_state.gps_sleep_state;
        if (!already_awake)
        {
            g_system_state.gps_sleep_state = false;
        }
        xSemaphoreGive(g_state_mutex);
    }

    if (!already_awake)
    {
        const char *cmd = "$PCAS10,0*1C\r\n";
        uart_write_bytes(GPS_UART_NUM, cmd, strlen(cmd));
        vTaskDelay(pdMS_TO_TICKS(800));
        ESP_LOGI("GPS", "Wake cmd sent");
    }
}

// Simple NMEA parser for GPS data
static bool parse_gps_data(float *lat, float *lng)
{
    char buffer[512];
    int len = uart_read_bytes(GPS_UART_NUM, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(300));

    if (len <= 0)
        return false;

    buffer[len] = '\0';

    // Look for $GPGGA or $GNGGA sentence
    char *gga = strstr(buffer, "GGA,");
    if (!gga)
        return false;

    // Parse: $xxGGA,time,lat,N/S,lon,E/W,fix,...
    char *token = strtok(gga, ",");
    if (!token)
        return false;

    // Skip time
    token = strtok(NULL, ",");
    if (!token)
        return false;

    // Latitude
    token = strtok(NULL, ",");
    if (!token || strlen(token) < 4)
        return false;

    float lat_deg = (token[0] - '0') * 10 + (token[1] - '0');
    float lat_min = atof(token + 2);
    float latitude = lat_deg + lat_min / 60.0f;

    // N/S
    token = strtok(NULL, ",");
    if (!token)
        return false;
    if (token[0] == 'S')
        latitude = -latitude;

    // Longitude
    token = strtok(NULL, ",");
    if (!token || strlen(token) < 5)
        return false;

    float lng_deg = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');
    float lng_min = atof(token + 3);
    float longitude = lng_deg + lng_min / 60.0f;

    // E/W
    token = strtok(NULL, ",");
    if (!token)
        return false;
    if (token[0] == 'W')
        longitude = -longitude;

    // Fix quality
    token = strtok(NULL, ",");
    if (!token)
        return false;
    int fix = atoi(token);

    if (fix > 0)
    {
        *lat = latitude;
        *lng = longitude;
        return true;
    }

    return false;
}

void task_button_handler(void *pvParameters)
{
    ESP_LOGI(TAG, "Button handler task started");
    vTaskDelay(pdMS_TO_TICKS(2000));

    bool last_mode_state = gpio_get_level(MODE_BUTTON_PIN);
    bool last_reset_state = gpio_get_level(RESET_BUTTON_PIN);
    uint64_t mode_press_time = 0;
    uint64_t reset_press_time = 0;
    bool mode_pressed = false;
    bool reset_pressed = false;

    while (1)
    {
        bool mode_state = gpio_get_level(MODE_BUTTON_PIN);
        bool reset_state = gpio_get_level(RESET_BUTTON_PIN);
        uint64_t now = esp_timer_get_time() / 1000; // ms

        // MODE button (active low)
        if (!mode_state && last_mode_state)
        {
            mode_press_time = now;
            mode_pressed = true;
        }
        else if (mode_state && !last_mode_state && mode_pressed)
        {
            uint64_t held = now - mode_press_time;
            mode_pressed = false;

            if (held >= 50 && held <= 10000)
            {
                button_event_t ev = BUTTON_EVENT_MODE_SHORT_PRESS;
                xQueueSend(g_button_event_queue, &ev, pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        last_mode_state = mode_state;

        // RESET button (active low)
        if (!reset_state && last_reset_state)
        {
            reset_press_time = now;
            reset_pressed = true;
        }
        else if (reset_state && !last_reset_state && reset_pressed)
        {
            uint64_t held = now - reset_press_time;
            reset_pressed = false;

            if (held >= 50 && held <= 5000)
            {
                if (held >= LONG_PRESS_RESET_MS)
                {
                    button_event_t ev = BUTTON_EVENT_RESET_LONG_PRESS;
                    xQueueSend(g_button_event_queue, &ev, pdMS_TO_TICKS(10));
                }
                else
                {
                    button_event_t ev = BUTTON_EVENT_RESET_SHORT_PRESS;
                    xQueueSend(g_button_event_queue, &ev, pdMS_TO_TICKS(10));
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        last_reset_state = reset_state;

        // Process button events
        button_event_t be;
        if (xQueueReceive(g_button_event_queue, &be, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            bool owner_present = false;

            if (be == BUTTON_EVENT_MODE_SHORT_PRESS)
            {
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    g_system_state.owner_present = !g_system_state.owner_present;
                    owner_present = g_system_state.owner_present;
                    xSemaphoreGive(g_state_mutex);
                }

                // Save to NVS
                nvs_handle_t handle;
                if (nvs_open("system", NVS_READWRITE, &handle) == ESP_OK)
                {
                    nvs_set_u8(handle, "ownerPresent", owner_present ? 1 : 0);
                    nvs_commit(handle);
                    nvs_close(handle);
                }

                if (owner_present)
                {
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        g_system_state.alarm_stage = STAGE_NONE;
                        g_system_state.motion_detected = false;
                        g_system_state.last_motion_time_ms = 0;
                        g_system_state.strong_motion_detected = false;
                        g_system_state.strong_motion_start_time_ms = 0;
                        g_system_state.strong_motion_count = 0;
                        xSemaphoreGive(g_state_mutex);
                    }

                    gpio_set_level(LED_ALERT_PIN, 0);
                    gpio_set_level(BUZZER_PIN, 0);
                    mqtt_send_safe();
                    ESP_LOGI(TAG, "Owner present -> STAGE_NONE");
                }
                else
                {
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        g_system_state.alarm_stage = STAGE_WARNING;
                        g_system_state.last_motion_time_ms = esp_timer_get_time() / 1000;
                        xSemaphoreGive(g_state_mutex);
                    }

                    mpu6050_wake_from_sleep();
                    mqtt_send_safe();
                    ESP_LOGI(TAG, "Owner absent -> STAGE_WARNING");
                }
            }
            else if (be == BUTTON_EVENT_RESET_SHORT_PRESS)
            {
                // Reset from TRACKING to NONE
                bool do_reset = false;
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    if (g_system_state.alarm_stage == STAGE_TRACKING)
                    {
                        g_system_state.alarm_stage = STAGE_NONE;
                        g_system_state.gps_valid = false;
                        g_system_state.current_lat = 0.0;
                        g_system_state.current_lng = 0.0;
                        g_system_state.motion_detected = false;
                        g_system_state.strong_motion_detected = false;
                        g_system_state.owner_present = true;
                        g_system_state.init_lat = 0.0;
                        g_system_state.init_lng = 0.0;
                        g_system_state.sent_mid_range_move_sms = false;
                        do_reset = true;
                    }
                    xSemaphoreGive(g_state_mutex);
                }

                if (do_reset)
                {
                    gpio_set_level(LED_ALERT_PIN, 0);
                    gpio_set_level(BUZZER_PIN, 0);
                    mqtt_send_safe();

                    nvs_handle_t handle;
                    if (nvs_open("system", NVS_READWRITE, &handle) == ESP_OK)
                    {
                        nvs_set_u8(handle, "ownerPresent", 1);
                        nvs_commit(handle);
                        nvs_close(handle);
                    }
                    if (nvs_open("sms_cd", NVS_READWRITE, &handle) == ESP_OK)
                    {
                        nvs_erase_all(handle);
                        nvs_commit(handle);
                        nvs_close(handle);
                    }
                    ESP_LOGI(TAG, "RESET short -> STAGE_NONE");
                }
            }
            else if (be == BUTTON_EVENT_RESET_LONG_PRESS)
            {
                ESP_LOGI(TAG, "RESET long -> factory reset");
                nvs_handle_t handle;
                if (nvs_open("system", NVS_READWRITE, &handle) == ESP_OK)
                {
                    nvs_erase_all(handle);
                    nvs_commit(handle);
                    nvs_close(handle);
                }
                if (nvs_open("sms_cd", NVS_READWRITE, &handle) == ESP_OK)
                {
                    nvs_erase_all(handle);
                    nvs_commit(handle);
                    nvs_close(handle);
                }
                play_beep_pattern(4, 80, 80);
                vTaskDelay(pdMS_TO_TICKS(300));
                esp_restart();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void task_mpu_handler(void *pvParameters)
{
    ESP_LOGI(TAG, "MPU handler task started");

    // Initialize MPU6050
    if (mpu6050_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 init failed, task exiting");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        alarm_stage_t alarm_stage;
        bool mpu_sleep;

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            alarm_stage = g_system_state.alarm_stage;
            mpu_sleep = g_system_state.mpu_sleep_state;
            xSemaphoreGive(g_state_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Sleep/wake MPU based on stage
        if ((alarm_stage == STAGE_NONE || alarm_stage == STAGE_TRACKING) && !mpu_sleep)
        {
            mpu6050_put_to_sleep();
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                g_system_state.mpu_sleep_state = true;
                xSemaphoreGive(g_state_mutex);
            }
        }
        else if ((alarm_stage == STAGE_WARNING || alarm_stage == STAGE_ALERT) && mpu_sleep)
        {
            mpu6050_wake_from_sleep();
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                g_system_state.mpu_sleep_state = false;
                xSemaphoreGive(g_state_mutex);
            }
        }

        // Read motion if awake and in WARNING/ALERT
        if (!mpu_sleep && (alarm_stage == STAGE_WARNING || alarm_stage == STAGE_ALERT))
        {
            float deviation = mpu6050_calc_deviation();
            uint64_t now = esp_timer_get_time() / 1000;

            if (deviation > STRONG_ACCEL_THRESHOLD)
            {
                bool should_send_mqtt = false;
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    if (!g_system_state.strong_motion_detected)
                    {
                        g_system_state.strong_motion_detected = true;
                        g_system_state.strong_motion_start_time_ms = now;
                        g_system_state.motion_detected = true;
                        g_system_state.strong_motion_count = 1;
                        should_send_mqtt = true;
                    }
                    else
                    {
                        g_system_state.strong_motion_count++;
                    }
                    g_system_state.last_motion_time_ms = now;
                    xSemaphoreGive(g_state_mutex);
                }

                ESP_LOGI(TAG, "Strong motion detected (%.2f m/s^2)", deviation);
                if (should_send_mqtt)
                {
                    mqtt_send_safe();
                }
            }
            else if (deviation > ACCEL_MIN_DETECT && deviation <= STRONG_ACCEL_THRESHOLD)
            {
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    if (!g_system_state.motion_detected)
                    {
                        g_system_state.motion_detected = true;
                        g_system_state.last_motion_time_ms = now;
                        xSemaphoreGive(g_state_mutex);
                        mqtt_send_safe();
                    }
                    else
                    {
                        g_system_state.last_motion_time_ms = now;
                        xSemaphoreGive(g_state_mutex);
                    }
                }
                ESP_LOGI(TAG, "Motion detected (%.2f m/s^2)", deviation);
            }
            else if (deviation <= LIGHT_ACCEL_DEADBAND)
            {
                uint64_t last_motion = 0;
                bool has_motion = false, has_strong = false;

                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    has_motion = g_system_state.motion_detected;
                    has_strong = g_system_state.strong_motion_detected;
                    last_motion = g_system_state.last_motion_time_ms;
                    xSemaphoreGive(g_state_mutex);
                }

                if ((has_motion || has_strong) && (now - last_motion) > WARNING_ACTIVITY_TIMEOUT_MS)
                {
                    ESP_LOGI(TAG, "No motion for 2 minutes -> reset motion flags");
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        g_system_state.motion_detected = false;
                        g_system_state.strong_motion_detected = false;
                        g_system_state.strong_motion_start_time_ms = 0;
                        g_system_state.strong_motion_count = 0;
                        xSemaphoreGive(g_state_mutex);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void task_gps_handler(void *pvParameters)
{
    ESP_LOGI(TAG, "GPS handler task started");

    // Initialize GPS Kalman filter
    gps_kf_handle = gps_kalman_filter_create(0.1, 1.0, 0.01);

    while (1)
    {
        alarm_stage_t stage;
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            stage = g_system_state.alarm_stage;
            xSemaphoreGive(g_state_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (stage == STAGE_NONE || stage == STAGE_WARNING)
        {
            gps_put_to_sleep();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (stage == STAGE_ALERT || stage == STAGE_TRACKING)
        {
            gps_wake_from_sleep();

            float raw_lat, raw_lng;
            if (parse_gps_data(&raw_lat, &raw_lng))
            {
                float filtered_lat, filtered_lng;
                gps_kalman_filter_update(gps_kf_handle, raw_lat, raw_lng, &filtered_lat, &filtered_lng);

                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    g_system_state.current_lat = filtered_lat;
                    g_system_state.current_lng = filtered_lng;
                    g_system_state.gps_valid = true;

                    if (!g_system_state.had_valid_fix)
                    {
                        g_system_state.had_valid_fix = true;
                        g_system_state.gps_signal_lost = false;
                        ESP_LOGI("GPS", "First valid fix");
                    }
                    else if (g_system_state.gps_signal_lost)
                    {
                        g_system_state.gps_signal_lost = false;
                        ESP_LOGI("GPS", "Signal recovered");
                    }

                    g_system_state.last_gps_char_time_ms = esp_timer_get_time() / 1000;
                    xSemaphoreGive(g_state_mutex);
                }
            }
            else
            {
                // Check for signal loss
                uint64_t last_char = 0;
                bool had_fix = false;

                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    last_char = g_system_state.last_gps_char_time_ms;
                    had_fix = g_system_state.had_valid_fix;
                    xSemaphoreGive(g_state_mutex);
                }

                uint64_t now = esp_timer_get_time() / 1000;
                if (had_fix && (now - last_char > 5000))
                {
                    bool need_notify = false;
                    float last_lat, last_lng;

                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        if (!g_system_state.gps_signal_lost)
                        {
                            g_system_state.gps_signal_lost = true;
                            need_notify = true;
                            last_lat = g_system_state.current_lat;
                            last_lng = g_system_state.current_lng;
                        }
                        xSemaphoreGive(g_state_mutex);
                    }

                    if (need_notify)
                    {
                        ESP_LOGI("GPS", "Signal lost - using last known position");
                        char msg[160];
                        snprintf(msg, sizeof(msg),
                                 "CANH BAO: Mat tin hieu GPS. Vi tri cuoi: https://maps.google.com/?q=%.6f,%.6f",
                                 last_lat, last_lng);

                        if (sms_check_cooldown(SMS_ALERT_GPS_LOST, 0))
                        {
                            sms_message_t sms;
                            strncpy(sms.message, msg, sizeof(sms.message) - 1);
                            sms.type = SMS_ALERT_GPS_LOST;
                            xQueueSend(g_sms_queue, &sms, pdMS_TO_TICKS(100));
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_sim_handler(void *pvParameters)
{
    ESP_LOGI(TAG, "SIM handler task started");

    extern volatile uint64_t g_last_sim_activity;

    while (1)
    {
        alarm_stage_t alarm_stage;
        bool owner_present;
        bool sim_sleep;

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            alarm_stage = g_system_state.alarm_stage;
            owner_present = g_system_state.owner_present;
            sim_sleep = g_system_state.sim_sleep_state;
            xSemaphoreGive(g_state_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Check SMS queue
        UBaseType_t sms_waiting = uxQueueMessagesWaiting(g_sms_queue);
        if (sms_waiting > 0)
        {
            g_last_sim_activity = esp_timer_get_time() / 1000;

            if (xSemaphoreTake(g_sim_busy_semaphore, pdMS_TO_TICKS(10000)) == pdTRUE)
            {
                if (sim_sleep)
                {
                    sim_wake_from_sleep();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                sms_message_t outgoing;
                while (xQueueReceive(g_sms_queue, &outgoing, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    ESP_LOGI(TAG, "Sending SMS: %.50s...", outgoing.message);
                    bool sent = sms_send_message(PHONE_NUMBER, outgoing.message);

                    if (!sent)
                    {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        sent = sms_send_message(PHONE_NUMBER, outgoing.message);
                    }

                    if (sent)
                        ESP_LOGI(TAG, "SMS sent OK");
                    else
                        ESP_LOGI(TAG, "SMS send FAILED");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

                xSemaphoreGive(g_sim_busy_semaphore);
            }
            continue;
        }

        // Read incoming SMS if awake
        if (!sim_sleep)
        {
            if (xSemaphoreTake(g_sim_busy_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                char unread[1024];
                if (sms_read_unread(unread, sizeof(unread)))
                {
                    if (strlen(unread) > 0)
                    {
                        sim_process_incoming_sms(unread);
                    }
                }
                xSemaphoreGive(g_sim_busy_semaphore);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        // Auto sleep if idle
        bool should_sleep = false;
        uint64_t now = esp_timer_get_time() / 1000;

        if (alarm_stage == STAGE_NONE && owner_present)
        {
            should_sleep = true;
        }
        else if (alarm_stage == STAGE_WARNING)
        {
            bool has_motion = false;
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                has_motion = g_system_state.motion_detected || g_system_state.strong_motion_detected;
                xSemaphoreGive(g_state_mutex);
            }

            if (!has_motion && (now - g_last_sim_activity > 30000))
            {
                should_sleep = true;
            }
        }

        if (should_sleep && !sim_sleep)
        {
            if (xSemaphoreTake(g_sim_busy_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                if (uxQueueMessagesWaiting(g_sms_queue) == 0)
                {
                    sim_put_to_sleep();
                    ESP_LOGI(TAG, "SIM auto sleep (idle)");
                }
                xSemaphoreGive(g_sim_busy_semaphore);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_alarm_manager(void *pvParameters)
{
    ESP_LOGI(TAG, "Alarm manager task started");

    uint64_t last_blink = 0;
    bool led_state = false;
    uint64_t last_buzzer_toggle = 0;
    uint64_t buzzer_off_time = 0;
    bool buzzer_is_on = false;

    while (1)
    {
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            alarm_stage_t stage = g_system_state.alarm_stage;
            uint64_t now = esp_timer_get_time() / 1000;
            bool gps_valid = g_system_state.gps_valid;
            float current_lat = g_system_state.current_lat;
            float current_lng = g_system_state.current_lng;
            float init_lat = g_system_state.init_lat;
            float init_lng = g_system_state.init_lng;
            bool strong_motion = g_system_state.strong_motion_detected;
            uint64_t strong_start = g_system_state.strong_motion_start_time_ms;
            int strong_count = g_system_state.strong_motion_count;
            uint64_t last_motion = g_system_state.last_motion_time_ms;
            bool motion_detected = g_system_state.motion_detected;

            xSemaphoreGive(g_state_mutex);

            switch (stage)
            {
            case STAGE_NONE:
                gpio_set_level(LED_ALERT_PIN, 0);
                gpio_set_level(BUZZER_PIN, 0);

                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    g_system_state.init_lat = 0.0;
                    g_system_state.init_lng = 0.0;
                    g_system_state.strong_motion_detected = false;
                    g_system_state.strong_motion_start_time_ms = 0;
                    g_system_state.strong_motion_count = 0;
                    g_system_state.sent_mid_range_move_sms = false;
                    xSemaphoreGive(g_state_mutex);
                }
                break;

            case STAGE_WARNING:
                gpio_set_level(LED_ALERT_PIN, 1);
                gpio_set_level(BUZZER_PIN, 0);

                // Check escalate WARNING -> ALERT
                if (strong_motion && strong_start > 0 && strong_count >= STRONG_MOTION_COUNT_THRESHOLD)
                {
                    uint64_t strong_duration = now - strong_start;
                    if (strong_duration >= STRONG_MOTION_DURATION_MS)
                    {
                        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                        {
                            g_system_state.alarm_stage = STAGE_ALERT;
                            xSemaphoreGive(g_state_mutex);
                        }

                        if (sms_check_cooldown(SMS_ALERT_ALARM, 0))
                        {
                            sms_message_t msg;
                            strncpy(msg.message, "CANH BAO: Phat hien rung lac manh lien tuc! He thong chuyen sang che do BAO DONG.", sizeof(msg.message) - 1);
                            msg.type = SMS_ALERT_ALARM;
                            xQueueSend(g_sms_queue, &msg, pdMS_TO_TICKS(100));
                        }
                        mqtt_send_safe();
                        ESP_LOGI(TAG, "WARNING -> ALERT (strong motion persisted)");
                    }
                }
                // Reset motion after timeout
                else if (motion_detected || strong_motion)
                {
                    if ((now - last_motion) >= WARNING_ACTIVITY_TIMEOUT_MS)
                    {
                        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                        {
                            g_system_state.motion_detected = false;
                            g_system_state.strong_motion_detected = false;
                            g_system_state.strong_motion_start_time_ms = 0;
                            g_system_state.strong_motion_count = 0;
                            xSemaphoreGive(g_state_mutex);
                        }
                        ESP_LOGI(TAG, "No motion for 2min -> reset flags");
                        mqtt_send_safe();
                    }
                }
                break;

            case STAGE_ALERT:
                // Blink LED
                if (now - last_blink >= 1000)
                {
                    led_state = !led_state;
                    gpio_set_level(LED_ALERT_PIN, led_state ? 1 : 0);
                    last_blink = now;
                }

                // Buzzer beep
                if (buzzer_is_on && now >= buzzer_off_time)
                {
                    set_buzzer(false);
                    buzzer_is_on = false;
                    last_buzzer_toggle = now;
                }
                if (!buzzer_is_on && now - last_buzzer_toggle >= 1000)
                {
                    set_buzzer(true);
                    buzzer_is_on = true;
                    buzzer_off_time = now + 200;
                }

                // Save initial GPS position
                if (gps_valid && (init_lat == 0.0 && init_lng == 0.0))
                {
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        g_system_state.init_lat = current_lat;
                        g_system_state.init_lng = current_lng;
                        xSemaphoreGive(g_state_mutex);
                    }

                    char msg[160];
                    snprintf(msg, sizeof(msg),
                             "Vi tri phat hien rung manh: https://maps.google.com/?q=%.6f,%.6f",
                             current_lat, current_lng);
                    if (sms_check_cooldown(SMS_ALERT_ALARM, 0))
                    {
                        sms_message_t sms;
                        strncpy(sms.message, msg, sizeof(sms.message) - 1);
                        sms.type = SMS_ALERT_ALARM;
                        xQueueSend(g_sms_queue, &sms, pdMS_TO_TICKS(100));
                    }
                    mqtt_send_safe();
                }

                // Check distance movement
                if (gps_valid && (init_lat != 0.0 && init_lng != 0.0))
                {
                    float distance = calculate_distance(init_lat, init_lng, current_lat, current_lng);

                    if (distance >= DISTANCE_THRESHOLD_MAX)
                    {
                        // -> TRACKING
                        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                        {
                            g_system_state.alarm_stage = STAGE_TRACKING;
                            xSemaphoreGive(g_state_mutex);
                        }

                        gpio_set_level(BUZZER_PIN, 0);
                        gpio_set_level(LED_ALERT_PIN, 0);
                        mqtt_send_safe();

                        char buf[200];
                        snprintf(buf, sizeof(buf),
                                 "THEO DOI: Xe da di chuyen %.1fm. Vi tri: https://maps.google.com/?q=%.6f,%.6f",
                                 distance, current_lat, current_lng);
                        if (sms_check_cooldown(SMS_ALERT_MOVEMENT, 0))
                        {
                            sms_message_t sms;
                            strncpy(sms.message, buf, sizeof(sms.message) - 1);
                            sms.type = SMS_ALERT_MOVEMENT;
                            xQueueSend(g_sms_queue, &sms, pdMS_TO_TICKS(100));
                        }
                        ESP_LOGI(TAG, "ALERT -> TRACKING");
                    }
                    else if (distance >= DISTANCE_THRESHOLD_MIN && distance < DISTANCE_THRESHOLD_MAX)
                    {
                        // Mid-range warning
                        bool sent = false;
                        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                        {
                            sent = g_system_state.sent_mid_range_move_sms;
                            if (!sent)
                            {
                                g_system_state.sent_mid_range_move_sms = true;
                            }
                            xSemaphoreGive(g_state_mutex);
                        }

                        if (!sent)
                        {
                            char buf[250];
                            snprintf(buf, sizeof(buf),
                                     "BAO DONG: Xe da di chuyen %.1fm. Vi tri: https://maps.google.com/?q=%.6f,%.6f. HAY RA KIEM TRA XE",
                                     distance, current_lat, current_lng);
                            if (sms_check_cooldown(SMS_ALERT_MOVEMENT, 0))
                            {
                                sms_message_t sms;
                                strncpy(sms.message, buf, sizeof(sms.message) - 1);
                                sms.type = SMS_ALERT_MOVEMENT;
                                xQueueSend(g_sms_queue, &sms, pdMS_TO_TICKS(100));
                            }
                            mqtt_send_safe();
                        }
                    }
                    else if (distance < DISTANCE_THRESHOLD_MIN)
                    {
                        // Back to WARNING after timeout
                        if ((now - last_motion) >= WARNING_ACTIVITY_TIMEOUT_MS)
                        {
                            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                            {
                                g_system_state.alarm_stage = STAGE_WARNING;
                                g_system_state.motion_detected = false;
                                g_system_state.last_motion_time_ms = now;
                                g_system_state.strong_motion_detected = false;
                                g_system_state.sent_mid_range_move_sms = false;
                                xSemaphoreGive(g_state_mutex);
                            }

                            char buf[200];
                            snprintf(buf, sizeof(buf),
                                     "BAO DONG: Xe da bi dich chuyen %.1fm. Vi tri: https://maps.google.com/?q=%.6f,%.6f",
                                     distance, current_lat, current_lng);
                            if (sms_check_cooldown(SMS_ALERT_MOVEMENT, 0))
                            {
                                sms_message_t sms;
                                strncpy(sms.message, buf, sizeof(sms.message) - 1);
                                sms.type = SMS_ALERT_MOVEMENT;
                                xQueueSend(g_sms_queue, &sms, pdMS_TO_TICKS(100));
                            }
                            mqtt_send_safe();
                            ESP_LOGI(TAG, "Light move -> back to WARNING");
                        }
                    }
                }

                // Periodic MQTT update (30s)
                static uint64_t last_mqtt_time = 0;
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    last_mqtt_time = g_system_state.last_gps_time_ms;
                    xSemaphoreGive(g_state_mutex);
                }

                if (gps_valid && (now - last_mqtt_time >= GPS_UPDATE_INTERVAL_MS))
                {
                    mqtt_send_safe();
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        g_system_state.last_gps_time_ms = now;
                        xSemaphoreGive(g_state_mutex);
                    }
                }
                break;

            case STAGE_TRACKING:
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    g_system_state.sent_mid_range_move_sms = false;
                    xSemaphoreGive(g_state_mutex);
                }

                bool send_sms = false;
                // Periodic updates
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    last_mqtt_time = g_system_state.last_gps_time_ms;
                    send_sms = g_system_state.send_sms_on_tracking;
                    xSemaphoreGive(g_state_mutex);
                }

                if (gps_valid && (now - last_mqtt_time >= GPS_UPDATE_INTERVAL_MS))
                {
                    mqtt_send_safe();
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        g_system_state.last_gps_time_ms = now;
                        xSemaphoreGive(g_state_mutex);
                    }
                }

                if (gps_valid && send_sms && sms_check_cooldown(SMS_ALERT_POSITION, 0))
                {
                    char buf[200];
                    snprintf(buf, sizeof(buf),
                             "THEO DOI: Vi tri: https://maps.google.com/?q=%.6f,%.6f",
                             current_lat, current_lng);
                    sms_message_t sms;
                    strncpy(sms.message, buf, sizeof(sms.message) - 1);
                    sms.type = SMS_ALERT_POSITION;
                    xQueueSend(g_sms_queue, &sms, pdMS_TO_TICKS(100));
                }
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_sleep_manager(void *pvParameters)
{
    ESP_LOGI(TAG, "Sleep manager task started");

    while (1)
    {
        alarm_stage_t stage;
        bool owner_present;
        uint64_t last_motion_time;

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            stage = g_system_state.alarm_stage;
            owner_present = g_system_state.owner_present;
            last_motion_time = g_system_state.last_motion_time_ms;
            xSemaphoreGive(g_state_mutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // ========== DEEP SLEEP: STAGE_NONE && owner present ==========
        if (stage == STAGE_NONE && owner_present)
        {
            static uint64_t last_activity_check = 0;
            static bool waiting_for_idle = false;

            // Wait for SMS queue to be empty
            UBaseType_t sms_waiting = uxQueueMessagesWaiting(g_sms_queue);
            if (sms_waiting > 0)
            {
                last_activity_check = esp_timer_get_time() / 1000;
                waiting_for_idle = true;
                ESP_LOGI(TAG, "Activity detected (%d SMS waiting), reset idle timer", sms_waiting);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            if (!waiting_for_idle)
            {
                last_activity_check = esp_timer_get_time() / 1000;
                waiting_for_idle = true;
                ESP_LOGI(TAG, "Starting 30s idle countdown before deep sleep...");
            }

            uint64_t idle_time = (esp_timer_get_time() / 1000) - last_activity_check;
            if (idle_time < 30000)
            {
                ESP_LOGI(TAG, "Waiting for idle... %llu/30000 ms", idle_time);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            // Reset flag
            waiting_for_idle = false;

            // Final check SMS queue
            sms_waiting = uxQueueMessagesWaiting(g_sms_queue);
            if (sms_waiting > 0)
            {
                ESP_LOGI(TAG, "Found %d SMS after idle wait, restart", sms_waiting);
                continue;
            }

            // Acquire SIM lock
            if (xSemaphoreTake(g_sim_busy_semaphore, pdMS_TO_TICKS(10000)) == pdTRUE)
            {
                ESP_LOGI(TAG, "Preparing deep sleep...");

                // Read battery
                float bat_v = read_battery_voltage();
                int bat_pct = calc_battery_percent(bat_v);
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    g_system_state.battery_voltage = bat_v;
                    g_system_state.battery_percent = bat_pct;
                    xSemaphoreGive(g_state_mutex);
                }

                mqtt_send_safe();

                // Sleep SIM
                sim_put_to_sleep();
                ESP_LOGI(TAG, "SIM sleeping before deep sleep");

                xSemaphoreGive(g_sim_busy_semaphore);
            }
            else
            {
                ESP_LOGW(TAG, "Cannot acquire SIM lock for deep sleep, retry...");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            // Sleep other modules
            gps_put_to_sleep();
            mpu6050_put_to_sleep();

            // Configure wakeup
            esp_sleep_enable_timer_wakeup(SLEEP_WAKE_INTERVAL_US);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)MODE_BUTTON_PIN, 0);
    
            ESP_LOGI(TAG, "Going to deep sleep for 20 minutes...");
            vTaskDelay(pdMS_TO_TICKS(200));

            esp_deep_sleep_start();
        }

        // ========== LIGHT SLEEP: STAGE_WARNING && no motion for 2+ minutes ==========
        if (stage == STAGE_WARNING && !owner_present)
        {
            uint64_t time_since_motion = (esp_timer_get_time() / 1000) - last_motion_time;

            if (time_since_motion > LIGHT_SLEEP_TIMEOUT_MS)
            {
                // Wait for SMS queue to be empty
                UBaseType_t sms_waiting = uxQueueMessagesWaiting(g_sms_queue);
                if (sms_waiting > 0)
                {
                    ESP_LOGI(TAG, "Waiting for %d SMS to be sent...", sms_waiting);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }

                // Acquire SIM lock and turn off SIM
                if (xSemaphoreTake(g_sim_busy_semaphore, pdMS_TO_TICKS(5000)) == pdTRUE)
                {
                    ESP_LOGI(TAG, "Preparing light sleep...");

                    mqtt_send_safe();
                    sim_put_to_sleep();

                    ESP_LOGI(TAG, "SIM sleeping before light sleep");

                    xSemaphoreGive(g_sim_busy_semaphore);
                }
                else
                {
                    ESP_LOGW(TAG, "Cannot acquire SIM lock, retry later");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }

                // Sleep GPS
                gps_put_to_sleep();

                // Configure MPU for motion detection interrupt
                mpu6050_wake_from_sleep();
                vTaskDelay(pdMS_TO_TICKS(50));

                // Configure motion interrupt: threshold=20 (~320mg), duration=1ms
                mpu6050_config_motion_interrupt(20, 1);

                // Enable motion interrupt
                mpu6050_enable_motion_interrupt();

                // Clear any pending interrupt
                uint8_t int_status = mpu6050_get_interrupt_status();
                (void)int_status; // Suppress unused variable warning

                ESP_LOGI(TAG, "Entering light sleep (STAGE_WARNING)");
                vTaskDelay(pdMS_TO_TICKS(200));

                // Configure wakeup sources
                esp_sleep_enable_ext0_wakeup((gpio_num_t)MODE_BUTTON_PIN, 0);
                esp_sleep_enable_ext1_wakeup(1ULL << MPU_INT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);
                esp_sleep_enable_timer_wakeup(SLEEP_WAKE_INTERVAL_US);

                esp_light_sleep_start();

                // Handle wakeup
                esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

                // Disable motion interrupt and reconfigure MPU for normal operation
                mpu6050_disable_motion_interrupt();

                // Restore normal MPU configuration
                mpu6050_wake_from_sleep(); // This will reconfigure for normal operation

                switch (cause)
                {
                case ESP_SLEEP_WAKEUP_EXT0:
                    ESP_LOGI(TAG, "Woke from button press");
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        g_system_state.last_motion_time_ms = esp_timer_get_time() / 1000;
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;

                case ESP_SLEEP_WAKEUP_EXT1:
                    ESP_LOGI(TAG, "Woke from MPU interrupt");
                    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        g_system_state.last_motion_time_ms = esp_timer_get_time() / 1000;
                        g_system_state.motion_detected = true;
                        xSemaphoreGive(g_state_mutex);
                    }
                    break;

                case ESP_SLEEP_WAKEUP_TIMER:
                    ESP_LOGI(TAG, "Woke from timer");
                    break;

                default:
                    ESP_LOGI(TAG, "Unknown wakeup reason");
                    break;
                }

                // Wake SIM from sleep
                ESP_LOGI(TAG, "Waking SIM from sleep...");
                sim_wake_from_sleep();
                vTaskDelay(pdMS_TO_TICKS(1000));

                extern volatile uint64_t g_last_sim_activity;
                g_last_sim_activity = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "SIM ready after light sleep");

                // Send MQTT update if woke from motion
                if (cause == ESP_SLEEP_WAKEUP_EXT1)
                {
                    ESP_LOGI(TAG, "Sending MQTT update after motion wake...");
                    mqtt_send_safe();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

void task_battery_monitor(void *pvParameters)
{
    ESP_LOGI(TAG, "Battery monitor task started");

    while (1)
    {
        float bat_v = read_battery_voltage();
        int bat_pct = calc_battery_percent(bat_v);
        bool low_battery = false;

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_system_state.battery_voltage = bat_v;
            g_system_state.battery_percent = bat_pct;
            low_battery = g_system_state.low_battery;
            xSemaphoreGive(g_state_mutex);
        }

        if (bat_pct <= 20 && !low_battery)
        {
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                g_system_state.low_battery = true;
                xSemaphoreGive(g_state_mutex);
            }
            ESP_LOGI(TAG, "Battery low: %d%% Sending SMS Message", bat_pct);

            char buf[120];
            snprintf(buf, sizeof(buf), "CANH BAO: Pin thap: %d%%. Vui long sac pin.", bat_pct);
            if (sms_check_cooldown(SMS_ALERT_LOW_BATTERY, 0))
            {
                sms_message_t msg;
                strncpy(msg.message, buf, sizeof(msg.message) - 1);
                msg.type = SMS_ALERT_LOW_BATTERY;
                xQueueSend(g_sms_queue, &msg, pdMS_TO_TICKS(100));
            }
            mqtt_send_safe();
        }
        else if (bat_pct >= 80 && low_battery)
        {
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                g_system_state.low_battery = false;
                xSemaphoreGive(g_state_mutex);
            }
        }

        // Sleep for 15 minutes
        for (int i = 0; i < 15 * 60; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
