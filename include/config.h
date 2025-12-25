#ifndef CONFIG_H
#define CONFIG_H

// ================== Pin & config ==================
#define BUZZER_PIN 18
#define BATTERY_PIN 34
#define MPU_INT_PIN 32
#define LED_ALERT_PIN 14
#define MODE_BUTTON_PIN 27
#define RESET_BUTTON_PIN 35
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define SIM_RX_PIN 25
#define SIM_TX_PIN 26

#define SLEEP_WAKE_INTERVAL_MS (20ULL * 60 * 1000000) // 20 phút
#define GPS_UPDATE_INTERVAL_MS 30000                  // 30s
#define BUZZER_BEEP_MS 150
#define BUZZER_INTERVAL_MS 10000 // 10s
#define ALERT_LED_BLINK_MS 1000
#define BATTERY_LOW_THRESHOLD 6.0f
#define BATTERY_HIGH_THRESHOLD 8.4f
#define WARNING_ACTIVITY_TIMEOUT 120000UL // 2 phút
#define LONG_PRESS_RESET_MS 3000UL
#define LIGHT_SLEEP_TIMEOUT 140000UL   // 2 phút 10s không có motion → light sleep
#define STRONG_MOTION_DURATION 30000UL // 30s motion mạnh → escalate
#define ALERT_NO_GPS_TIMEOUT 180000UL  // 5 phút không có GPS → fallback

// MQTT Configuration
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT "1883"
#define MQTT_CLIENT_ID "ESP32_GPS_01"
#define MQTT_TOPIC_PUB "gps/tracker/data"

#define PHONE_NUMBER "+84886966103"
#define SIM_APN "v-internet"

// Motion thresholds
#define STRONG_ACCEL_THRESHOLD 3.6f
#define LIGHT_ACCEL_DEADBAND 1.5f
#define ACCEL_MIN_DETECT 2.0f
#define STRONG_MOTION_COUNT_THRESHOLD 3
#define DISTANCE_THRESHOLD_MAX 10.0f // 10 m
#define DISTANCE_THRESHOLD_MIN 3.0f  // 3 m

// MPU Registers
#define MPU_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_CONFIG 0x1A
#define MPU6050_MOT_THR 0x1F
#define MPU6050_MOT_DUR 0x20
#define MPU6050_INT_PIN_CONFIG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A

// GPS Commands
#define GPS_SLEEP_CMD "$PCAS05,1*1E\r\n"
#define GPS_WAKE_CMD "$PCAS10,0*1C\r\n"

#endif // CONFIG_H
