#include "mqtt_handler.h"
#include "system_data.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "sms_handler.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "MQTT";

// Global variable để track SIM activity
volatile uint64_t g_last_sim_activity = 0;

// Helper function to send AT command and wait for response
bool sim_send_at_expect(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    uart_write_bytes(SIM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART_NUM, "\r\n", 2);

    char response[512];
    int len = uart_read_bytes(SIM_UART_NUM, response, sizeof(response) - 1, pdMS_TO_TICKS(timeout_ms));

    if (len > 0)
    {
        response[len] = '\0';
        if (expect != NULL && expect[0] != '\0')
        {
            if (strstr(response, expect) != NULL)
            {
                return true;
            }
        }
        return true;
    }
    return false;
}

// Helper to read response with timeout
static bool sim_read_response(char *buffer, int buffer_size, const char *pattern, uint32_t timeout_ms)
{
    memset(buffer, 0, buffer_size);
    uint64_t start = esp_timer_get_time() / 1000;
    int total_read = 0;

    while ((esp_timer_get_time() / 1000 - start) < timeout_ms && total_read < buffer_size - 1)
    {
        int len = uart_read_bytes(SIM_UART_NUM, buffer + total_read,
                                  buffer_size - total_read - 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            total_read += len;
            buffer[total_read] = '\0';

            if (pattern && strstr(buffer, pattern) != NULL)
            {
                vTaskDelay(pdMS_TO_TICKS(300));
                len = uart_read_bytes(SIM_UART_NUM, buffer + total_read,
                                      buffer_size - total_read - 1, pdMS_TO_TICKS(300));
                if (len > 0)
                {
                    total_read += len;
                    buffer[total_read] = '\0';
                }
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return (total_read > 0);
}

bool sim_init(void)
{
    ESP_LOGI(TAG, "Initializing SIM A7682S...");

    // Send wake sequence
    ESP_LOGI(TAG, "Sending wake sequence...");
    for (int i = 0; i < 5; i++)
    {
        uart_write_bytes(SIM_UART_NUM, "AT\r\n", 4);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    uart_write_bytes(SIM_UART_NUM, "AT+CSCLK=0\r\n", 12);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    // Test communication
    bool responded = false;
    for (int retry = 0; retry < 3; retry++)
    {
        if (sim_send_at_expect("AT", "OK", 2000))
        {
            responded = true;
            break;
        }
        ESP_LOGW(TAG, "No response, retry %d/3", retry + 1);
        uart_write_bytes(SIM_UART_NUM, "AT\r\n", 4);
        vTaskDelay(pdMS_TO_TICKS(200));
        uart_write_bytes(SIM_UART_NUM, "AT+CSCLK=0\r\n", 12);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!responded)
    {
        ESP_LOGE(TAG, "SIM module not responding");
        return false;
    }

    sim_send_at_expect("AT+CMGF=1", "OK", 2000); // SMS text mode
    sim_send_at_expect("AT+CREG?", "OK", 2000);  // Check registration

    if (!sim_send_at_expect("AT+CGATT=1", "OK", 5000))
    {
        ESP_LOGE(TAG, "CGATT failed!");
        return false;
    }

    char apn_cmd[128];
    snprintf(apn_cmd, sizeof(apn_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", SIM_APN);
    sim_send_at_expect(apn_cmd, "OK", 2000);

    if (!sim_send_at_expect("AT+CGACT=1,1", "OK", 8000))
    {
        ESP_LOGE(TAG, "CGACT failed!");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        g_system_state.sim_sleep_state = false;
        g_system_state.sim_connected = true;
        xSemaphoreGive(g_state_mutex);
    }

    ESP_LOGI(TAG, "SIM initialized successfully");
    return true;
}

void sim_put_to_sleep(void)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (g_system_state.sim_sleep_state)
        {
            xSemaphoreGive(g_state_mutex);
            return;
        }
        g_system_state.sim_sleep_state = true;
        xSemaphoreGive(g_state_mutex);
    }

    // Close MQTT connections
    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTDISC=0,60\r\n", 19);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTREL=0\r\n", 15);
    vTaskDelay(pdMS_TO_TICKS(300));
    uart_flush(SIM_UART_NUM);

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTSTOP\r\n", 14);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    // Deactivate PDP context
    uart_write_bytes(SIM_UART_NUM, "AT+CGACT=0,1\r\n", 14);
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(SIM_UART_NUM);

    // Enter sleep mode
    uart_write_bytes(SIM_UART_NUM, "AT+CSCLK=2\r\n", 12);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "SIM sleep cmd sent (network deactivated)");
}

void sim_wake_from_sleep(void)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (!g_system_state.sim_sleep_state)
        {
            xSemaphoreGive(g_state_mutex);
            return;
        }
        g_system_state.sim_sleep_state = false;
        xSemaphoreGive(g_state_mutex);
    }

    // Wake up
    uart_write_bytes(SIM_UART_NUM, "AT\r\n", 4);
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(SIM_UART_NUM, "AT\r\n", 4);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Disable sleep
    uart_write_bytes(SIM_UART_NUM, "AT+CSCLK=0\r\n", 12);
    vTaskDelay(pdMS_TO_TICKS(300));
    uart_flush(SIM_UART_NUM);

    // Reactivate PDP
    uart_write_bytes(SIM_UART_NUM, "AT+CGACT=1,1\r\n", 14);
    char resp[256];
    sim_read_response(resp, sizeof(resp), "OK", 5000);

    ESP_LOGI(TAG, "SIM wake cmd sent (network reactivated)");
}

bool mqtt_connect(void)
{
    ESP_LOGI(TAG, "Connecting to MQTT broker...");

    // Stop old sessions
    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTDISC=0,60\r\n", 19);
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(SIM_UART_NUM);

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTREL=0\r\n", 15);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTSTOP\r\n", 14);
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(SIM_UART_NUM);

    // Start MQTT service
    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTSTART\r\n", 15);
    char start_resp[256];
    sim_read_response(start_resp, sizeof(start_resp), "+CMQTTSTART:", 5000);

    if (strstr(start_resp, "+CMQTTSTART: 0") == NULL && strstr(start_resp, "OK") == NULL)
    {
        uart_write_bytes(SIM_UART_NUM, "AT+CMQTTSTOP\r\n", 14);
        vTaskDelay(pdMS_TO_TICKS(1000));
        uart_flush(SIM_UART_NUM);

        uart_write_bytes(SIM_UART_NUM, "AT+CMQTTSTART\r\n", 15);
        sim_read_response(start_resp, sizeof(start_resp), "+CMQTTSTART:", 5000);
    }

    // Create MQTT client
    char accq_cmd[128];
    snprintf(accq_cmd, sizeof(accq_cmd), "AT+CMQTTACCQ=0,\"%s\"\r\n", MQTT_CLIENT_ID);
    if (!sim_send_at_expect(accq_cmd, "OK", 3000))
    {
        ESP_LOGE(TAG, "ACCQ failed!");
        return false;
    }

    // Connect to broker (fixed format)
    char connect_cmd[256];
    snprintf(connect_cmd, sizeof(connect_cmd),
             "AT+CMQTTCONNECT=0,\"tcp://%s:%s\",60,1\r\n",
             MQTT_BROKER, MQTT_PORT);
    uart_write_bytes(SIM_UART_NUM, connect_cmd, strlen(connect_cmd));

    char conn_resp[512];
    sim_read_response(conn_resp, sizeof(conn_resp), "+CMQTTCONNECT:", 20000);

    if (strstr(conn_resp, "+CMQTTCONNECT: 0,0") != NULL)
    {
        ESP_LOGI(TAG, "Connected successfully!");

        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            g_system_state.mqtt_connected = true;
            xSemaphoreGive(g_state_mutex);
        }
        return true;
    }

    ESP_LOGE(TAG, "Connection failed");
    return false;
}

void mqtt_disconnect(void)
{
    ESP_LOGI(TAG, "Disconnecting...");

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTDISC=0,60\r\n", 19);
    vTaskDelay(pdMS_TO_TICKS(2000));
    uart_flush(SIM_UART_NUM);

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTREL=0\r\n", 15);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTSTOP\r\n", 14);
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(SIM_UART_NUM);

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        g_system_state.mqtt_connected = false;
        xSemaphoreGive(g_state_mutex);
    }

    ESP_LOGI(TAG, "Disconnected");
}

bool mqtt_publish_topic_payload(const char *topic, const char *payload)
{
    int topic_len = strlen(topic);
    int payload_len = strlen(payload);

    ESP_LOGI(TAG, "Publishing to %s (%d bytes)", topic, payload_len);

    // Set topic
    char topic_cmd[64];
    snprintf(topic_cmd, sizeof(topic_cmd), "AT+CMQTTTOPIC=0,%d\r\n", topic_len);
    uart_write_bytes(SIM_UART_NUM, topic_cmd, strlen(topic_cmd));

    char topic_resp[256];
    sim_read_response(topic_resp, sizeof(topic_resp), ">", 3000);

    if (strstr(topic_resp, ">") == NULL)
    {
        ESP_LOGE(TAG, "Topic prompt timeout");
        return false;
    }

    uart_write_bytes(SIM_UART_NUM, topic, topic_len);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    // Set payload
    char payload_cmd[64];
    snprintf(payload_cmd, sizeof(payload_cmd), "AT+CMQTTPAYLOAD=0,%d\r\n", payload_len);
    uart_write_bytes(SIM_UART_NUM, payload_cmd, strlen(payload_cmd));

    char pay_resp[256];
    sim_read_response(pay_resp, sizeof(pay_resp), ">", 3000);

    if (strstr(pay_resp, ">") == NULL)
    {
        ESP_LOGE(TAG, "Payload prompt timeout");
        return false;
    }

    uart_write_bytes(SIM_UART_NUM, payload, payload_len);
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush(SIM_UART_NUM);

    // Publish
    uart_write_bytes(SIM_UART_NUM, "AT+CMQTTPUB=0,1,60\r\n", 20);
    char pub_resp[256];
    sim_read_response(pub_resp, sizeof(pub_resp), "+CMQTTPUB:", 10000);

    if (strstr(pub_resp, "+CMQTTPUB: 0,0") != NULL)
    {
        ESP_LOGI(TAG, "Publish OK");
        return true;
    }

    ESP_LOGE(TAG, "Publish failed");
    return false;
}

bool mqtt_publish_data(void)
{
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        ESP_LOGE(TAG, "Failed to create JSON");
        return false;
    }

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        cJSON_AddNumberToObject(root, "battery_percent", g_system_state.battery_percent);
        cJSON_AddNumberToObject(root, "battery_voltage", g_system_state.battery_voltage);
        cJSON_AddNumberToObject(root, "owner_present", g_system_state.owner_present ? 1 : 0);
        cJSON_AddNumberToObject(root, "alarm_stage", (int)g_system_state.alarm_stage);
        cJSON_AddNumberToObject(root, "motion_detected", g_system_state.motion_detected ? 1 : 0);
        cJSON_AddNumberToObject(root, "mqtt_connected", 1);
        cJSON_AddNumberToObject(root, "gps_valid", g_system_state.gps_valid ? 1 : 0);

        if (g_system_state.gps_valid)
        {
            cJSON_AddNumberToObject(root, "latitude", g_system_state.current_lat);
            cJSON_AddNumberToObject(root, "longitude", g_system_state.current_lng);
        }

        cJSON_AddNumberToObject(root, "strong_motion", g_system_state.strong_motion_detected ? 1 : 0);
        cJSON_AddNumberToObject(root, "low_battery", g_system_state.low_battery ? 1 : 0);
        cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000);

        xSemaphoreGive(g_state_mutex);
    }

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_string)
    {
        ESP_LOGE(TAG, "Failed to print JSON");
        return false;
    }

    ESP_LOGI(TAG, "Sending: %s", json_string);

    bool result = mqtt_publish_topic_payload(MQTT_TOPIC_PUB, json_string);
    free(json_string);

    return result;
}

bool mqtt_send_safe(void)
{
    bool success = false;

    if (xSemaphoreTake(g_sim_busy_semaphore, pdMS_TO_TICKS(10000)) == pdTRUE)
    {
        bool sim_sleep = false;
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            sim_sleep = g_system_state.sim_sleep_state;
            xSemaphoreGive(g_state_mutex);
        }

        if (sim_sleep)
        {
            sim_wake_from_sleep();
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // Retry 2 times
        for (int retry = 0; retry < 2 && !success; retry++)
        {
            if (mqtt_connect())
            {
                success = mqtt_publish_data();
                mqtt_disconnect();
            }

            if (!success && retry < 1)
            {
                ESP_LOGW(TAG, "Retry...");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        xSemaphoreGive(g_sim_busy_semaphore);
    }
    else
    {
        ESP_LOGE(TAG, "Cannot acquire SIM lock");
    }

    return success;
}

void sim_process_incoming_sms(const char *resp)
{
    if (!resp)
        return;

    const char *pos = resp;

    while (*pos)
    {
        const char *nl = strchr(pos, '\n');
        if (!nl)
            break;

        size_t line_len = nl - pos;
        char line[256];
        if (line_len >= sizeof(line))
            line_len = sizeof(line) - 1;
        memcpy(line, pos, line_len);
        line[line_len] = '\0';

        // Trim whitespace
        char *start = line;
        while (*start && (*start == ' ' || *start == '\r'))
            start++;

        pos = nl + 1;

        if (strncmp(start, "+CMGL:", 6) == 0)
        {
            nl = strchr(pos, '\n');
            if (!nl)
                break;

            line_len = nl - pos;
            char body[256];
            if (line_len >= sizeof(body))
                line_len = sizeof(body) - 1;
            memcpy(body, pos, line_len);
            body[line_len] = '\0';

            pos = nl + 1;

            // Trim
            start = body;
            while (*start && (*start == ' ' || *start == '\r'))
                start++;

            // Convert to lowercase for comparison
            char low[256];
            strncpy(low, start, sizeof(low) - 1);
            for (char *p = low; *p; p++)
            {
                if (*p >= 'A' && *p <= 'Z')
                    *p = *p + 32;
            }

            ESP_LOGI(TAG, "SMS: %s", start);

            // Process commands
            if (strstr(low, "gps") != NULL)
            {
                // Handle GPS request
                float lat = 0.0f, lng = 0.0f;
                bool gps_ok = false;

                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    gps_ok = g_system_state.gps_valid;
                    lat = g_system_state.current_lat;
                    lng = g_system_state.current_lng;
                    xSemaphoreGive(g_state_mutex);
                }

                if (!gps_ok)
                {
                    if (sms_check_cooldown(SMS_ALERT_SYSTEM_ERROR, 0))
                    {
                        sms_message_t msg;
                        strncpy(msg.message, "Khong the lay vi tri: GPS chua san sang", sizeof(msg.message) - 1);
                        msg.type = SMS_ALERT_SYSTEM_ERROR;
                        xQueueSend(g_sms_queue, &msg, pdMS_TO_TICKS(100));
                    }
                }
                else
                {
                    char buf[160];
                    snprintf(buf, sizeof(buf), "VI TRI: https://maps.google.com/?q=%.6f,%.6f", lat, lng);
                    sms_message_t msg;
                    strncpy(msg.message, buf, sizeof(msg.message) - 1);
                    msg.type = SMS_ALERT_POSITION;
                    if (sms_check_cooldown(SMS_ALERT_POSITION, 1000))
                    {
                        xQueueSend(g_sms_queue, &msg, pdMS_TO_TICKS(100));
                    }
                }
            }
            else if ((strstr(low, "stop tracking") != NULL) ||
                     (strstr(low, "stop") != NULL && strstr(low, "tracking") != NULL))
            {
                // Reset to STAGE_NONE
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    g_system_state.alarm_stage = STAGE_NONE;
                    g_system_state.motion_detected = false;
                    g_system_state.owner_present = true;
                    g_system_state.gps_valid = false;
                    g_system_state.current_lat = 0.0;
                    g_system_state.current_lng = 0.0;
                    g_system_state.strong_motion_detected = false;
                    xSemaphoreGive(g_state_mutex);
                }

                // Save to NVS
                nvs_handle_t handle;
                if (nvs_open("system", NVS_READWRITE, &handle) == ESP_OK)
                {
                    nvs_set_u8(handle, "ownerPresent", 1);
                    nvs_commit(handle);
                    nvs_close(handle);
                }

                // Clear SMS cooldowns
                if (nvs_open("sms_cd", NVS_READWRITE, &handle) == ESP_OK)
                {
                    nvs_erase_all(handle);
                    nvs_commit(handle);
                    nvs_close(handle);
                }

                mqtt_send_safe();

                if (sms_check_cooldown(SMS_ALERT_SYSTEM_ERROR, 1000))
                {
                    sms_message_t msg;
                    strncpy(msg.message, "Da chuyen ve che do cho", sizeof(msg.message) - 1);
                    msg.type = SMS_ALERT_SYSTEM_ERROR;
                    xQueueSend(g_sms_queue, &msg, pdMS_TO_TICKS(100));
                }

                ESP_LOGI(TAG, "STOP_TRACKING processed");
            }
            else if (strstr(low, "stop sms") != NULL)
            {
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    g_system_state.send_sms_on_tracking = false;
                    xSemaphoreGive(g_state_mutex);
                }

                if (sms_check_cooldown(SMS_ALERT_SYSTEM_ERROR, 1000))
                {
                    sms_message_t msg;
                    strncpy(msg.message, "Da ngung gui SMS vi tri", sizeof(msg.message) - 1);
                    msg.type = SMS_ALERT_SYSTEM_ERROR;
                    xQueueSend(g_sms_queue, &msg, pdMS_TO_TICKS(100));
                }

                ESP_LOGI(TAG, "STOP SMS processed");
            }
        }
    }
}
