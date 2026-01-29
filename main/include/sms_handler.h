#ifndef SMS_HANDLER_H
#define SMS_HANDLER_H

#include "system_data.h"
#include <stdbool.h>

/**
 * @brief Initializes the SMS handler.
 *
 * Loads the last sent times for SMS cooldowns from NVS.
 */
void sms_handler_init(void);

/**
 * @brief Checks if an SMS of a certain type can be sent based on cooldowns.
 *
 * @param type The type of SMS alert.
 * @param custom_cooldown_ms An optional custom cooldown in milliseconds. If 0, the default is used.
 * @return true if the SMS can be sent, false otherwise.
 */
bool sms_check_cooldown(sms_alert_type_t type, uint32_t custom_cooldown_ms);

/**
 * @brief Sends an SMS message using the modem.
 *
 * This function will acquire the SIM semaphore before sending.
 *
 * @param phone_number The destination phone number.
 * @param message The message content.
 * @return true if the message was sent successfully, false otherwise.
 */
bool sms_send_message(const char *phone_number, const char *message);

/**
 * @brief Reads all unread SMS messages from the SIM module.
 *
 * @param buffer Buffer to store the raw response from the modem.
 * @param len Length of the buffer.
 * @return true on success, false on failure.
 */
bool sms_read_unread(char *buffer, int len);

#endif // SMS_HANDLER_H
