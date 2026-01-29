#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "stdint.h"
// ================== State definitions ==================
typedef enum
{
    STAGE_NONE = 0,
    STAGE_WARNING,
    STAGE_ALERT,
    STAGE_TRACKING
} alarm_stage_t;

typedef struct
{
    float battery_voltage;
    float current_lat, current_lng;
    float init_lat, init_lng;
    uint64_t last_motion_time_ms;
    uint64_t last_gps_time_ms;
    uint64_t last_gps_char_time_ms;
    uint64_t strong_motion_start_time_ms;
    int battery_percent;
    int strong_motion_count;
    alarm_stage_t alarm_stage;
    bool owner_present;
    bool motion_detected;
    bool sim_connected;
    bool low_battery;
    bool gps_valid;
    bool gps_signal_lost;
    bool had_valid_fix;
    bool mqtt_connected;
    bool mpu_sleep_state;
    bool gps_sleep_state;
    bool sim_sleep_state;
    bool strong_motion_detected;
    bool send_sms_on_tracking;
    bool sent_mid_range_move_sms;
} system_state_t;

// ================== SMS Queue & Cooldown ==================
typedef enum
{
    SMS_ALERT_ALARM,
    SMS_ALERT_MOVEMENT,
    SMS_ALERT_POSITION,
    SMS_ALERT_GPS_LOST,
    SMS_ALERT_LOW_BATTERY,
    SMS_ALERT_SYSTEM_ERROR,
    SMS_TYPE_COUNT // Helper to get the number of enum elements
} sms_alert_type_t;

typedef struct
{
    char message[160];
    sms_alert_type_t type;
} sms_message_t;

// ================== Button Event Queue ==================
typedef enum
{
    BUTTON_EVENT_MODE_SHORT_PRESS,
    BUTTON_EVENT_RESET_SHORT_PRESS,
    BUTTON_EVENT_RESET_LONG_PRESS,
} button_event_t;

// ================== Global Handles & State ==================

// Global variable to hold the system state
extern volatile system_state_t g_system_state;

// Mutex to protect access to g_system_state
extern SemaphoreHandle_t g_state_mutex;

// Semaphore to ensure only one task uses the SIM module at a time
extern SemaphoreHandle_t g_sim_busy_semaphore;

// Queues for inter-task communication
extern QueueHandle_t g_sms_queue;
extern QueueHandle_t g_button_event_queue;

#endif // SYSTEM_DATA_H
