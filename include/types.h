#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// ================== State definitions ==================
enum AlarmStage
{
    STAGE_NONE = 0,
    STAGE_WARNING,
    STAGE_ALERT,
    STAGE_TRACKING
};

struct SystemState
{
    float batteryVoltage;
    float currentLat, currentLng;
    float initLat, initLng;
    unsigned long lastMotionTime;
    unsigned long lastPositionMQTTTime;
    unsigned long lastPositionSMSTime;
    unsigned long lastGpsCharTime;
    unsigned long strongMotionStartTime;
    int batteryPercent;
    int strongMotionCount;
    AlarmStage alarmStage;
    bool ownerPresent;
    bool motionDetected;
    bool simConnected;
    bool lowBattery;
    bool gpsValid;
    bool gpsSignalLost;
    bool hadValidFix;
    bool mqttConnected;
    bool mpuSleepState;
    bool gpsSleepState;
    bool simSleepState;
    bool strongMotionDetected;
    bool sendSMSOnTracking;
    bool sentMidRangeMoveSMS;
    bool sentStrongMotionMQTT;
};

// ================== SMS queue / cooldown ==================
enum SMSAlertType
{
    SMS_ALERT_ALARM,
    SMS_ALERT_MOVEMENT,
    SMS_ALERT_POSITION,
    SMS_ALERT_GPS_LOST,
    SMS_ALERT_LOW_BATTERY,
    SMS_ALERT_SYSTEM_ERROR,
    SMS_ALERT_EMERGENCY
};

typedef struct
{
    char message[160];
    SMSAlertType type;
} SMSMessage;

// SMS Cooldown times
extern const unsigned long SMS_COOLDOWN[];

#endif // TYPES_H
