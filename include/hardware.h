#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include "config.h"

// ================== MPU Functions ==================
void writeMPU(uint8_t reg, uint8_t value);
uint8_t readMPU(uint8_t reg);
void putMPUToSleep();
void wakeMPUFromSleep();
float calcDeviation();
void IRAM_ATTR IRAM_mpuISR();

// ================== GPS Functions ==================
void putGPSToSleep();
void wakeGPSFromSleep();
bool updateGPS();

// ================== SIM Functions ==================
bool sendATExpect(const char *cmd, const char *expect, unsigned long timeoutMs);
bool initSIM();
void putSIMToSleep();
void wakeSIMFromSleep();

// ================== Battery Functions ==================
void initBatteryADC();
float readBatteryVoltage();
int calcBatteryPercent(float voltage);
void setBuzzer(bool state);
void playBeepPattern(int count, int onTime = 100, int offTime = 1000);

// ================== Utility Functions ==================
float calculateDistance(float lat1, float lng1, float lat2, float lng2);

#endif // HARDWARE_H
