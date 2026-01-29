#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include <stdbool.h>
/**
 * @brief Initializes the MPU6050 sensor.
 *
 * Probes for the sensor on the I2C bus, resets it, and configures its
 * operating parameters (accelerometer range, filter bandwidth). Also
 * initializes the Kalman filter for the accelerometer data.
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL if the sensor is not found or fails to init.
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Puts the MPU6050 sensor into sleep mode.
 */
void mpu6050_put_to_sleep(void);

/**
 * @brief Wakes the MPU6050 sensor from sleep mode.
 */
void mpu6050_wake_from_sleep(void);

/**
 * @brief Reads the accelerometer, applies the Kalman filter, and calculates
 *        the deviation from the 1G gravity vector.
 *
 * @return float The absolute deviation of the total acceleration vector from 9.8 m/s^2.
 */
float mpu6050_calc_deviation(void);

/**
 * @brief Configure MPU6050 motion detection interrupt
 * @param threshold Motion threshold in mg (milligrams), default ~320mg
 * @param duration Duration in ms
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t mpu6050_config_motion_interrupt(uint8_t threshold, uint8_t duration);

/**
 * @brief Enable motion detection interrupt
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t mpu6050_enable_motion_interrupt(void);

/**
 * @brief Disable motion detection interrupt
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t mpu6050_disable_motion_interrupt(void);

/**
 * @brief Get interrupt status and clear it
 *
 * @return uint8_t The interrupt status.
 */
uint8_t mpu6050_get_interrupt_status(void);

/**
 * @brief This function will be called from the main GPIO ISR when an MPU interrupt occurs.
 *        It should set a flag that can be checked by the MPU task.
 */
void mpu6050_isr_handler(void);

/**
 * @brief Checks if the MPU interrupt flag has been set by the ISR.
 *
 * @return true if an interrupt has occurred, false otherwise.
 */
bool mpu6050_get_interrupt_flag(void);

/**
 * @brief Clears the MPU interrupt flag.
 */
void mpu6050_clear_interrupt_flag(void);

#endif // MPU6050_H
