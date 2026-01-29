#ifndef CONFIG_H
#define CONFIG_H

// ================== Pin & config ==================
// NOTE: These are GPIO numbers
#define BUZZER_PIN 18
#define BATTERY_PIN 34 // ADC1_CHANNEL_6
#define MPU_INT_PIN 32
#define LED_ALERT_PIN 14
#define MODE_BUTTON_PIN 27
#define RESET_BUTTON_PIN 35
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define SIM_RX_PIN 25
#define SIM_TX_PIN 26

// I2C for MPU6050
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_MASTER_FREQ_HZ 400000

// UART configuration
#define GPS_UART_NUM UART_NUM_1
#define SIM_UART_NUM UART_NUM_2
#define UART_BUF_SIZE 1024

// ================== Timing and Intervals ==================
#define SLEEP_WAKE_INTERVAL_US (20ULL * 60 * 1000000) // 15 minutes for deep sleep
#define GPS_UPDATE_INTERVAL_MS 60000                  // 60s
#define BUZZER_BEEP_MS 150
#define BUZZER_INTERVAL_MS 10000 // 10s
#define ALERT_LED_BLINK_MS 1000
#define WARNING_ACTIVITY_TIMEOUT_MS 120000UL // 2 minutes
#define POSITION_REPORT_INTERVAL_MS 30000UL  // 30s
#define LONG_PRESS_RESET_MS 3000UL
#define LIGHT_SLEEP_TIMEOUT_MS 140000UL   // 2 min 10s no motion -> light sleep
#define STRONG_MOTION_DURATION_MS 60000UL // 60s strong motion -> escalate
#define ALERT_NO_GPS_TIMEOUT_MS 300000UL  // 5 minutes no GPS -> fallback

// ================== Thresholds & Behavior ==================
#define BATTERY_LOW_VOLTAGE 6.0f
#define BATTERY_HIGH_VOLTAGE 8.4f
#define STRONG_ACCEL_THRESHOLD 4.8f
#define LIGHT_ACCEL_DEADBAND 1.5f
#define ACCEL_MIN_DETECT 2.0f
#define STRONG_MOTION_COUNT_THRESHOLD 3
#define DISTANCE_THRESHOLD_MAX 10.0f // meters
#define DISTANCE_THRESHOLD_MIN 3.0f  // meters

// ================== MQTT Configuration ==================
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT "1883"
#define MQTT_CLIENT_ID "ESP32_GPS_01"
#define MQTT_TOPIC_PUB "gps/tracker/data"

// ================== SIM & SMS Configuration ==================
#define PHONE_NUMBER "+84886966103"
#define SIM_APN "v-internet"

// ================== Task Configuration ==================
#define TASK_MAIN_STACK_SIZE 4096
#define TASK_MPU_STACK_SIZE 3072
#define TASK_GPS_STACK_SIZE 3072
#define TASK_SIM_STACK_SIZE 4096
#define TASK_ALARM_STACK_SIZE 3072
#define TASK_BUTTON_STACK_SIZE 2048
#define TASK_BATTERY_STACK_SIZE 2048

#endif // CONFIG_H
