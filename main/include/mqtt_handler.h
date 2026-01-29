#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize SIM module với AT commands
 */
bool sim_init(void);

/**
 * @brief Put SIM module to sleep
 */
void sim_put_to_sleep(void);

/**
 * @brief Wake SIM module from sleep
 */
void sim_wake_from_sleep(void);

/**
 * @brief Connect to MQTT broker using AT commands
 */
bool mqtt_connect(void);

/**
 * @brief Disconnect from MQTT broker
 */
void mqtt_disconnect(void);

/**
 * @brief Publish JSON data to MQTT broker
 */
bool mqtt_publish_data(void);

/**
 * @brief Send MQTT safely (với semaphore)
 */
bool mqtt_send_safe(void);

/**
 * @brief Send AT command and expect response
 */
bool sim_send_at_expect(const char *cmd, const char *expect, uint32_t timeout_ms);

/**
 * @brief Process incoming SMS messages
 */
void sim_process_incoming_sms(const char *response);

#endif // MQTT_HANDLER_H
