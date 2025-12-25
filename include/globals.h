#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include "kalman_filter.h"
#include "types.h"

// ================== Global Variables ==================
extern volatile SystemState sysState;
extern volatile unsigned long globalLastSIMActivity;

// ================== FreeRTOS / hardware ==================
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t simBusySemaphore;
extern QueueHandle_t smsQueue;
extern QueueHandle_t buttonEventQueue;

extern TaskHandle_t taskHandleMPU;
extern TaskHandle_t taskHandleGPS;
extern TaskHandle_t taskHandleSIM;
extern TaskHandle_t taskHandleAlarm;
extern TaskHandle_t taskHandleButton;
extern TaskHandle_t taskHandleSleep;
extern TaskHandle_t taskHandleBattery;

extern HardwareSerial gpsSerial;
extern HardwareSerial simSerial;

// ================== Hardware Objects ==================
extern MPUKalmanFilter mpuKalman;
extern Adafruit_MPU6050 mpu;
extern TinyGPSPlus gps;
extern GPSKalmanFilter gpsKalman;
extern Preferences preferences;

// ================== SMS Cooldown ==================
extern unsigned long lastSMSSentTime[7];

// ================== MPU Interrupt ==================
extern volatile bool mpuInterruptFlag;
extern portMUX_TYPE mux;

#endif // GLOBALS_H
