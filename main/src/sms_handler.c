#include "sms_handler.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "SMS_HANDLER";

// SMS cooldown periods in milliseconds (same as Arduino code)
static const uint32_t SMS_COOLDOWN_MS[] = {
    20000,  // SMS_ALERT_ALARM - 20s
    60000,  // SMS_ALERT_MOVEMENT - 1min
    60000,  // SMS_ALERT_POSITION - 1min
    180000, // SMS_ALERT_GPS_LOST - 3min
    900000, // SMS_ALERT_LOW_BATTERY - 15min
    600000, // SMS_ALERT_SYSTEM_ERROR - 10min
};

static uint64_t last_sms_sent_time[SMS_TYPE_COUNT] = {0};

void sms_handler_init(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("sms_cd", NVS_READONLY, &handle);
    if (err == ESP_OK)
    {
        for (int i = 0; i < SMS_TYPE_COUNT; i++)
        {
            char key[16];
            snprintf(key, sizeof(key), "t%d", i);
            uint64_t value = 0;
            nvs_get_u64(handle, key, &value);
            last_sms_sent_time[i] = value;
        }
        nvs_close(handle);
        ESP_LOGI(TAG, "SMS cooldown data loaded from NVS");
    }
    else
    {
        ESP_LOGI(TAG, "No SMS cooldown data in NVS, starting fresh");
    }
}

bool sms_check_cooldown(sms_alert_type_t type, uint32_t custom_cooldown_ms)
{
    if (type >= SMS_TYPE_COUNT)
    {
        ESP_LOGE(TAG, "Invalid SMS type: %d", type);
        return false;
    }

    uint64_t now = esp_timer_get_time() / 1000; // Convert to ms
    uint32_t cooldown = (custom_cooldown_ms > 0) ? custom_cooldown_ms : SMS_COOLDOWN_MS[type];

    // First time sending
    if (last_sms_sent_time[type] == 0)
    {
        last_sms_sent_time[type] = now;

        // Save to NVS
        nvs_handle_t handle;
        if (nvs_open("sms_cd", NVS_READWRITE, &handle) == ESP_OK)
        {
            char key[16];
            snprintf(key, sizeof(key), "t%d", type);
            nvs_set_u64(handle, key, now);
            nvs_commit(handle);
            nvs_close(handle);
        }

        ESP_LOGI(TAG, "First send for type %d allowed", type);
        return true;
    }

    // Check overflow
    if (last_sms_sent_time[type] > now)
    {
        last_sms_sent_time[type] = 0;
        ESP_LOGW(TAG, "Timer overflow detected, reset cooldown");
        return true;
    }

    // Check cooldown
    if ((now - last_sms_sent_time[type]) >= cooldown)
    {
        last_sms_sent_time[type] = now;

        // Save to NVS
        nvs_handle_t handle;
        if (nvs_open("sms_cd", NVS_READWRITE, &handle) == ESP_OK)
        {
            char key[16];
            snprintf(key, sizeof(key), "t%d", type);
            nvs_set_u64(handle, key, now);
            nvs_commit(handle);
            nvs_close(handle);
        }

        return true;
    }

    ESP_LOGD(TAG, "Type %d still in cooldown (%" PRIu64 "/%" PRIu32 " ms)",
             type, now - last_sms_sent_time[type], cooldown);
    return false;
}

bool sms_send_message(const char *phone_number, const char *message)
{
    if (!phone_number || !message)
    {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    // Send AT command to start SMS
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r\n", phone_number);
    uart_write_bytes(SIM_UART_NUM, cmd, strlen(cmd));

    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for prompt

    // Send message content
    uart_write_bytes(SIM_UART_NUM, message, strlen(message));

    // Send Ctrl+Z to finish
    char ctrl_z = 0x1A;
    uart_write_bytes(SIM_UART_NUM, &ctrl_z, 1);

    // Wait for response
    char response[256];
    int len = uart_read_bytes(SIM_UART_NUM, response, sizeof(response) - 1, pdMS_TO_TICKS(10000));

    if (len > 0)
    {
        response[len] = '\0';
        if (strstr(response, "OK") || strstr(response, "CMGS"))
        {
            ESP_LOGI(TAG, "SMS sent successfully");
            return true;
        }
    }

    ESP_LOGE(TAG, "SMS send failed");
    return false;
}

bool sms_read_unread(char *buffer, int len)
{
    if (!buffer || len <= 0)
    {
        return false;
    }

    const char *cmd = "AT+CMGL=\"REC UNREAD\"\r\n";
    uart_write_bytes(SIM_UART_NUM, cmd, strlen(cmd));

    vTaskDelay(pdMS_TO_TICKS(2000));

    int read_len = uart_read_bytes(SIM_UART_NUM, buffer, len - 1, pdMS_TO_TICKS(2000));
    if (read_len > 0)
    {
        buffer[read_len] = '\0';
        return true;
    }

    return false;
}
