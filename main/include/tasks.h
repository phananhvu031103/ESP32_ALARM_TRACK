#ifndef TASKS_H
#define TASKS_H

// ================== Task Configuration ==================

// MPU Task
#define MPU_TASK_STACK_SIZE 3072
#define MPU_TASK_PRIORITY 4
#define MPU_TASK_CORE_ID 0

// GPS Task
#define GPS_TASK_STACK_SIZE 3072
#define GPS_TASK_PRIORITY 2
#define GPS_TASK_CORE_ID 1

// SIM Task
#define SIM_TASK_STACK_SIZE 4096
#define SIM_TASK_PRIORITY 2
#define SIM_TASK_CORE_ID 1

// Alarm Manager Task
#define ALARM_TASK_STACK_SIZE 4096
#define ALARM_TASK_PRIORITY 3
#define ALARM_TASK_CORE_ID 0

// Button Handler Task
#define BUTTON_TASK_STACK_SIZE 2048
#define BUTTON_TASK_PRIORITY 5
#define BUTTON_TASK_CORE_ID 0

// Battery Monitor Task
#define BATTERY_TASK_STACK_SIZE 3072
#define BATTERY_TASK_PRIORITY 1
#define BATTERY_TASK_CORE_ID 1

// Sleep Manager Task
#define SLEEP_TASK_STACK_SIZE 4096
#define SLEEP_TASK_PRIORITY 3
#define SLEEP_TASK_CORE_ID 0
/**
 * @brief Main state machine task.
 *
 * Manages the alarm state (NONE, WARNING, ALERT, TRACKING) based on
 * inputs from other tasks (motion, GPS distance, etc.). Controls the
 * buzzer and alert LED.
 */
void task_alarm_manager(void *pvParameters);

/**
 * @brief MPU6050 motion detection task.
 *
 * Periodically reads the MPU6050 sensor to detect motion and updates
 * the global system state accordingly.
 */
void task_mpu_handler(void *pvParameters);

/**
 * @brief GPS power management and monitoring task.
 *
 * Puts the GPS to sleep or wakes it up based on the current alarm stage.
 * Also monitors for GPS signal loss.
 */
void task_gps_handler(void *pvParameters);

/**
 * @brief SIM/SMS handling task.
 *
 * Waits for messages in the SMS queue and sends them. Can also be
 * extended to check for incoming SMS commands.
 */
void task_sim_handler(void *pvParameters);

/**
 * @brief Button input handling task.
 *
 * Handles debouncing, short presses, and long presses for the mode
 * and reset buttons.
 */
void task_button_handler(void *pvParameters);

/**
 * @brief Battery monitoring task.
 *
 * Periodically reads the battery voltage via ADC, calculates the
 * percentage, and updates the global system state.
 */
void task_battery_monitor(void *pvParameters);

/**
 * @brief Sleep manager task.
 *
 * Handles deep sleep (STAGE_NONE) and light sleep (STAGE_WARNING)
 */
void task_sleep_manager(void *pvParameters);

#endif // TASKS_H
