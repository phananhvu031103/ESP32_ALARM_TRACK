#ifndef HARDWARE_H
#define HARDWARE_H

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

extern adc_oneshot_unit_handle_t g_adc_handle;
/**
 * @brief Initializes all hardware peripherals for the application.
 *
 * This function calls all other initialization functions for I2C, UARTs,
 * GPIO, and ADC.
 *
 * @return esp_err_t ESP_OK on success, or an error code from one of the
 *         underlying driver initialization functions.
 */
esp_err_t hardware_init(void);

/**
 * @brief Initializes the I2C master driver.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t hardware_init_i2c(void);

/**
 * @brief Initializes the UART drivers for GPS and SIM modules.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t hardware_init_uarts(void);

/**
 * @brief Initializes GPIO pins for buttons, LEDs, and buzzer.
 *        Also sets up GPIO ISR for the buttons.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t hardware_init_gpio(void);

/**
 * @brief Initializes the ADC for battery voltage monitoring.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t hardware_init_adc(void);

#endif // HARDWARE_H
