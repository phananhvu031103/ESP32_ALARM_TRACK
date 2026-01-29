#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "system_data.h"
#include "config.h"
#include "hardware.h"
#include "tasks.h"
#include "mqtt_handler.h"
#include "sms_handler.h"

static const char *TAG = "MAIN";

// Global system state
volatile system_state_t g_system_state = {
    .battery_voltage = 0.0f,
    .current_lat = 0.0f,
    .current_lng = 0.0f,
    .init_lat = 0.0f,
    .init_lng = 0.0f,
    .last_motion_time_ms = 0,
    .last_gps_time_ms = 0,
    .last_gps_char_time_ms = 0,
    .strong_motion_start_time_ms = 0,
    .battery_percent = 0,
    .strong_motion_count = 0,
    .alarm_stage = STAGE_NONE,
    .owner_present = true,
    .motion_detected = false,
    .sim_connected = false,
    .low_battery = false,
    .gps_valid = false,
    .gps_signal_lost = false,
    .had_valid_fix = false,
    .mqtt_connected = false,
    .mpu_sleep_state = false,
    .gps_sleep_state = true,
    .sim_sleep_state = true,
    .strong_motion_detected = false,
    .send_sms_on_tracking = true,
    .sent_mid_range_move_sms = false,
};

// Global handles
SemaphoreHandle_t g_state_mutex = NULL;
SemaphoreHandle_t g_sim_busy_semaphore = NULL;
QueueHandle_t g_sms_queue = NULL;
QueueHandle_t g_button_event_queue = NULL;

static void load_system_state_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("system", NVS_READONLY, &handle);

    if (err == ESP_OK)
    {
        uint8_t owner_present = 1;
        nvs_get_u8(handle, "ownerPresent", &owner_present);

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_system_state.owner_present = (owner_present != 0);

            if (g_system_state.owner_present)
            {
                g_system_state.alarm_stage = STAGE_NONE;
                ESP_LOGI(TAG, "Owner present on startup -> STAGE_NONE");
            }
            else
            {
                g_system_state.alarm_stage = STAGE_WARNING;
                g_system_state.last_motion_time_ms = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "Owner absent on startup -> STAGE_WARNING");
            }

            xSemaphoreGive(g_state_mutex);
        }

        nvs_close(handle);
    }
    else
    {
        ESP_LOGI(TAG, "No saved state in NVS, using defaults");

        // Save default state
        if (nvs_open("system", NVS_READWRITE, &handle) == ESP_OK)
        {
            nvs_set_u8(handle, "ownerPresent", 1);
            nvs_commit(handle);
            nvs_close(handle);
        }
    }
}

static void handle_wakeup_cause(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        ESP_LOGI(TAG, "Woke up from MODE button press");
        // Toggle owner state
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_system_state.owner_present = !g_system_state.owner_present;

            nvs_handle_t handle;
            if (nvs_open("system", NVS_READWRITE, &handle) == ESP_OK)
            {
                nvs_set_u8(handle, "ownerPresent", g_system_state.owner_present ? 1 : 0);
                nvs_commit(handle);
                nvs_close(handle);
            }

            if (g_system_state.owner_present)
            {
                g_system_state.alarm_stage = STAGE_NONE;
            }
            else
            {
                g_system_state.alarm_stage = STAGE_WARNING;
                g_system_state.last_motion_time_ms = esp_timer_get_time() / 1000;
            }

            xSemaphoreGive(g_state_mutex);
        }
        break;

    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(TAG, "Woke up from timer");
        break;

    case ESP_SLEEP_WAKEUP_EXT1:
        ESP_LOGI(TAG, "Woke up from MPU motion");
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_system_state.motion_detected = true;
            g_system_state.last_motion_time_ms = esp_timer_get_time() / 1000;
            xSemaphoreGive(g_state_mutex);
        }
        break;

    default:
        ESP_LOGI(TAG, "Power on or reset");
        break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==== ESP32 GPS Tracking System ====");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create synchronization objects
    g_state_mutex = xSemaphoreCreateMutex();
    g_sim_busy_semaphore = xSemaphoreCreateMutex();
    g_sms_queue = xQueueCreate(10, sizeof(sms_message_t));
    g_button_event_queue = xQueueCreate(5, sizeof(button_event_t));

    if (!g_state_mutex || !g_sim_busy_semaphore || !g_sms_queue || !g_button_event_queue)
    {
        ESP_LOGE(TAG, "Failed to create synchronization objects");
        esp_restart();
    }

    // Initialize hardware
    if (hardware_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Hardware initialization failed");
        esp_restart();
    }

    // Load system state from NVS
    load_system_state_from_nvs();

    // Handle wakeup cause
    handle_wakeup_cause();

    // Initialize SMS handler
    sms_handler_init();

    // Initialize SIM module
    if (sim_init())
    {
        ESP_LOGI(TAG, "SIM initialized successfully");

        // Send initial MQTT update
        mqtt_send_safe();
    }
    else
    {
        ESP_LOGW(TAG, "SIM initialization failed, will retry later");
    }

    // Configure watchdog timer (60 seconds)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 60000,
        .idle_core_mask = 0,
        .trigger_panic = true};
    esp_task_wdt_init(&wdt_config);

    // Create FreeRTOS tasks
    BaseType_t xReturned;

    // Core 0: Time-critical tasks (MPU, Alarm, Button)
    xReturned = xTaskCreatePinnedToCore(
        task_mpu_handler,
        "MPU",
        MPU_TASK_STACK_SIZE,
        NULL,
        MPU_TASK_PRIORITY,
        NULL,
        MPU_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create MPU task");
    }

    xReturned = xTaskCreatePinnedToCore(
        task_alarm_manager,
        "Alarm",
        ALARM_TASK_STACK_SIZE,
        NULL,
        ALARM_TASK_PRIORITY,
        NULL,
        ALARM_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Alarm task");
    }

    xReturned = xTaskCreatePinnedToCore(
        task_button_handler,
        "Button",
        BUTTON_TASK_STACK_SIZE,
        NULL,
        BUTTON_TASK_PRIORITY,
        NULL,
        BUTTON_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Button task");
    }

    // Core 1: Communication & monitoring tasks (SIM, GPS, Battery)
    xReturned = xTaskCreatePinnedToCore(
        task_sim_handler,
        "SIM",
        SIM_TASK_STACK_SIZE,
        NULL,
        SIM_TASK_PRIORITY,
        NULL,
        SIM_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create SIM task");
    }

    xReturned = xTaskCreatePinnedToCore(
        task_gps_handler,
        "GPS",
        GPS_TASK_STACK_SIZE,
        NULL,
        GPS_TASK_PRIORITY,
        NULL,
        GPS_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create GPS task");
    }

    xReturned = xTaskCreatePinnedToCore(
        task_battery_monitor,
        "Battery",
        BATTERY_TASK_STACK_SIZE,
        NULL,
        BATTERY_TASK_PRIORITY,
        NULL,
        BATTERY_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Battery task");
    }

    xReturned = xTaskCreatePinnedToCore(
        task_sleep_manager,
        "Sleep",
        SLEEP_TASK_STACK_SIZE,
        NULL,
        SLEEP_TASK_PRIORITY,
        NULL,
        SLEEP_TASK_CORE_ID);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Sleep task");
    }

    ESP_LOGI(TAG, "System started successfully");
    ESP_LOGI(TAG, "---------------------------------------");

    // Beep pattern to indicate startup
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BUZZER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BUZZER_PIN, 0);

    esp_task_wdt_add(NULL);
    // Main loop - just feed watchdog
    while (1)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
