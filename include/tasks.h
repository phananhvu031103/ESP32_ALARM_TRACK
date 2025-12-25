#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>

// ================== FreeRTOS Tasks ==================
void taskButtonHandler(void *param);
void taskMPUHandler(void *param);
void taskAlarmManager(void *param);
void taskGPSHandler(void *param);
void taskSIMHandler(void *param);
void taskSleepManager(void *param);
void taskBatteryMonitor(void *param);

#endif // TASKS_H
