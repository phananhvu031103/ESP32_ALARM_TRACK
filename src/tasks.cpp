#include <Arduino.h>
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include <Preferences.h>

#include "config.h"
#include "types.h"
#include "globals.h"
#include "hardware.h"
#include "tasks.h"
#include "mqtt_handler.h"
#include "sms_handler.h"

// ================== TASK: Button Handler ==================
void taskButtonHandler(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    const TickType_t debounceTicks = pdMS_TO_TICKS(50);
    bool lastModeState = digitalRead(MODE_BUTTON_PIN);
    bool lastResetState = digitalRead(RESET_BUTTON_PIN);
    unsigned long modePressTime = 0;
    unsigned long resetPressTime = 0;
    bool resetPressed = false;
    bool modePressed = false;

    while (true)
    {
        esp_task_wdt_reset();

        bool modeState = digitalRead(MODE_BUTTON_PIN);
        bool resetState = digitalRead(RESET_BUTTON_PIN);
        unsigned long now = millis();

        // MODE button
        if (!modeState && lastModeState)
        {
            modePressTime = now;
            modePressed = true;
        }
        else if (modeState && !lastModeState)
        {
            unsigned long held = now - modePressTime;
            modePressed = false;
            if (held >= 50 && held <= 10000)
            {
                int ev = 1;
                xQueueSend(buttonEventQueue, &ev, pdMS_TO_TICKS(10));
            }
            vTaskDelay(debounceTicks);
        }
        lastModeState = modeState;

        // RESET button
        if (!resetState && lastResetState)
        {
            resetPressTime = millis();
            resetPressed = true;
        }
        else if (resetState && !lastResetState && resetPressed)
        {
            unsigned long held = now - resetPressTime;
            resetPressed = false;
            if (held >= 50 && held <= 5000)
            {
                if (held >= LONG_PRESS_RESET_MS)
                {
                    Serial.printf("[BUTTON] RESET long press detected (%lu ms)\n", held);
                    int ev = 3;
                    xQueueSend(buttonEventQueue, &ev, pdMS_TO_TICKS(10));
                }
                else
                {
                    int ev = 2;
                    xQueueSend(buttonEventQueue, &ev, pdMS_TO_TICKS(10));
                }
                vTaskDelay(pdMS_TO_TICKS(debounceTicks));
            }
            else if (!resetPressed && resetState)
            {
                resetPressTime = 0;
            }
        }

        lastResetState = resetState;

        int be;
        if (xQueueReceive(buttonEventQueue, &be, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            bool ownerPresent = false;

            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                switch (be)
                {
                case 1: // Toggle owner
                    sysState.ownerPresent = !sysState.ownerPresent;
                    ownerPresent = sysState.ownerPresent;
                    preferences.begin("system", false);
                    preferences.putBool("ownerPresent", ownerPresent);
                    preferences.end();
                    break;
                case 2:
                    break;
                case 3:
                    break;
                }
                xSemaphoreGive(stateMutex);
            }

            if (be == 1)
            {
                if (ownerPresent)
                {
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        sysState.alarmStage = STAGE_NONE;
                        sysState.motionDetected = false;
                        sysState.lastMotionTime = 0;
                        sysState.strongMotionDetected = false;
                        sysState.strongMotionStartTime = 0;
                        sysState.strongMotionCount = 0;
                        sysState.sentStrongMotionMQTT = false;
                        xSemaphoreGive(stateMutex);
                    }

                    digitalWrite(LED_ALERT_PIN, LOW);
                    digitalWrite(BUZZER_PIN, LOW);

                    sendMQTTSafe();

                    Serial.println("[BUTTON] Owner present -> STAGE_NONE");
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                else
                {
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        sysState.alarmStage = STAGE_WARNING;
                        sysState.lastMotionTime = millis();
                        xSemaphoreGive(stateMutex);
                    }

                    wakeMPUFromSleep();
                    sendMQTTSafe();

                    Serial.println("[BUTTON] Owner absent -> STAGE_WARNING");
                }
            }
            else if (be == 2) // Short reset
            {
                bool doReset = false;
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    if (sysState.alarmStage == STAGE_TRACKING)
                    {
                        sysState.alarmStage = STAGE_NONE;
                        sysState.motionDetected = false;
                        sysState.strongMotionDetected = false;
                        sysState.ownerPresent = true;
                        sysState.lastMotionTime = 0;
                        sysState.strongMotionStartTime = 0;
                        sysState.strongMotionCount = 0;
                        sysState.sentStrongMotionMQTT = false;
                        sysState.gpsValid = false;
                        sysState.initLat = 0.0;
                        sysState.initLng = 0.0;
                        sysState.sentMidRangeMoveSMS = false;
                        doReset = true;
                    }
                    xSemaphoreGive(stateMutex);
                }

                if (doReset)
                {
                    digitalWrite(LED_ALERT_PIN, LOW);
                    digitalWrite(BUZZER_PIN, LOW);

                    sendMQTTSafe();

                    preferences.begin("system", false);
                    preferences.putBool("ownerPresent", true);
                    preferences.end();

                    preferences.begin("sms_cd", false);
                    preferences.clear();
                    preferences.end();

                    Serial.println("[BUTTON] RESET short -> STAGE_NONE");
                }
            }
            else if (be == 3) // Long reset
            {
                Serial.println("[BUTTON] RESET long -> factory reset");
                preferences.begin("system", false);
                preferences.clear();
                preferences.end();

                preferences.begin("sms_cd", false);
                preferences.clear();
                preferences.end();

                playBeepPattern(4, 80, 80);
                vTaskDelay(pdMS_TO_TICKS(300));
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================== TASK: MPU Handler ==================
void taskMPUHandler(void *param)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    vTaskDelay(pdMS_TO_TICKS(100));

    bool mpuOk = false;
    for (int retry = 0; retry < 3; retry++)
    {
        if (mpu.begin())
        {
            mpuOk = true;
            break;
        }
        Serial.printf("[MPU] Init failed, retry %d/3\n", retry + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!mpuOk)
    {
        Serial.println("[MPU] I2C error - check wiring!");
        vTaskDelete(NULL);
        return;
    }

    Serial.println("[MPU] Initialized OK");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[MPU] Warming up Kalman filter...");
    for (int i = 0; i < 20; i++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float fx, fy, fz;
        mpuKalman.updateAcceleration(a.acceleration.x, a.acceleration.y, a.acceleration.z, &fx, &fy, &fz);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("[MPU] Kalman filter ready");
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), IRAM_mpuISR, RISING);

    while (true)
    {
        esp_task_wdt_reset();
        AlarmStage alarmStage;
        bool mpuSleep;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            alarmStage = sysState.alarmStage;
            mpuSleep = sysState.mpuSleepState;
            xSemaphoreGive(stateMutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if ((alarmStage == STAGE_NONE || alarmStage == STAGE_TRACKING) && !mpuSleep)
        {
            putMPUToSleep();
        }
        else if ((alarmStage == STAGE_WARNING || alarmStage == STAGE_ALERT) && mpuSleep)
        {
            wakeMPUFromSleep();
        }

        if (!mpuSleep && (alarmStage == STAGE_WARNING || alarmStage == STAGE_ALERT))
        {
            float deviation = calcDeviation();
            unsigned long now = millis();

            if (deviation > STRONG_ACCEL_THRESHOLD)
            {
                bool shouldSendMQTT = false;
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    if (!sysState.strongMotionDetected)
                    {
                        sysState.strongMotionDetected = true;
                        sysState.strongMotionStartTime = now;
                        sysState.motionDetected = true;
                        sysState.strongMotionCount = 1;
                    }
                    else
                    {
                        sysState.strongMotionCount++;
                    }
                    sysState.lastMotionTime = now;
                    if (!sysState.sentStrongMotionMQTT)
                    {
                        sysState.sentStrongMotionMQTT = true;
                        shouldSendMQTT = true;
                    }
                    xSemaphoreGive(stateMutex);

                    Serial.printf("[MPU] Strong motion detected (%.2f m/s^2) - will escalate if persists", deviation);
                    if (shouldSendMQTT)
                    {
                        sendMQTTSafe();
                    }
                }
            }
            else if (deviation > ACCEL_MIN_DETECT && deviation <= STRONG_ACCEL_THRESHOLD)
            {
                Serial.printf("[MPU] Motion detected (%.2f m/s^2)\n", deviation);
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    if (!sysState.motionDetected)
                    {
                        sysState.motionDetected = true;
                        sysState.lastMotionTime = now;
                        xSemaphoreGive(stateMutex);
                        sendMQTTSafe();
                    }
                    else
                    {
                        sysState.lastMotionTime = now;
                        xSemaphoreGive(stateMutex);
                    }
                }
            }
            else if (deviation <= LIGHT_ACCEL_DEADBAND)
            {
                unsigned long lastMotion = 0;
                bool hasMotion = false;
                bool hasStrongMotion = false;
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    hasMotion = sysState.motionDetected;
                    hasStrongMotion = sysState.strongMotionDetected;
                    lastMotion = sysState.lastMotionTime;
                    xSemaphoreGive(stateMutex);
                }

                if ((hasMotion || hasStrongMotion) && (millis() - lastMotion) > WARNING_ACTIVITY_TIMEOUT)
                {
                    Serial.println("[MPU] No motion for 2 minutes -> reset motion flags");
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        sysState.motionDetected = false;
                        sysState.strongMotionDetected = false;
                        sysState.strongMotionStartTime = 0;
                        sysState.strongMotionCount = 0;
                        sysState.sentStrongMotionMQTT = false;
                        xSemaphoreGive(stateMutex);
                    }
                }
            }
        }

        if (mpuInterruptFlag)
        {
            portENTER_CRITICAL(&mux);
            mpuInterruptFlag = false;
            portEXIT_CRITICAL(&mux);

            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sysState.lastMotionTime = millis();
                xSemaphoreGive(stateMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ================== TASK: Alarm Manager ==================
void taskAlarmManager(void *param)
{
    unsigned long lastBlink = 0;
    bool ledState = false;
    unsigned long lastBuzzerToggle = 0;
    unsigned long buzzerOffTime = 0;
    bool buzzerIsOn = false;

    while (true)
    {
        esp_task_wdt_reset();
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            AlarmStage stage = sysState.alarmStage;
            unsigned long now = millis();
            bool gpsValid = sysState.gpsValid;
            bool gpsSignalLost = sysState.gpsSignalLost;
            bool mqttConnected = sysState.mqttConnected;
            float currentLat = sysState.currentLat;
            float currentLng = sysState.currentLng;
            float initLat = sysState.initLat;
            float initLng = sysState.initLng;
            bool strongMotionDetected = sysState.strongMotionDetected;
            unsigned long strongMotionStartTime = sysState.strongMotionStartTime;
            int strongMotionCount = sysState.strongMotionCount;
            bool motionDetected = sysState.motionDetected;
            unsigned long lastMotionTime = sysState.lastMotionTime;
            bool sendSMSOnTracking = sysState.sendSMSOnTracking;
            unsigned long lastPositionMQTTTime = sysState.lastPositionMQTTTime;
            unsigned long lastPositionSMSTime = sysState.lastPositionSMSTime;
            bool sentMidRangeMoveSMS = sysState.sentMidRangeMoveSMS;

            xSemaphoreGive(stateMutex);

            switch (stage)
            {
            case STAGE_NONE:
                digitalWrite(LED_ALERT_PIN, LOW);
                digitalWrite(BUZZER_PIN, LOW);

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    sysState.initLat = 0.0;
                    sysState.initLng = 0.0;
                    sysState.strongMotionDetected = false;
                    sysState.strongMotionStartTime = 0;
                    sysState.strongMotionCount = 0;
                    sysState.sentMidRangeMoveSMS = false;
                    sysState.sentStrongMotionMQTT = false;
                    xSemaphoreGive(stateMutex);
                }
                break;

            case STAGE_WARNING:
                digitalWrite(LED_ALERT_PIN, HIGH);
                digitalWrite(BUZZER_PIN, LOW);

                if (strongMotionDetected && strongMotionStartTime > 0 &&
                    strongMotionCount >= STRONG_MOTION_COUNT_THRESHOLD)
                {
                    unsigned long strongDuration = millis() - strongMotionStartTime;
                    if (strongDuration >= STRONG_MOTION_DURATION)
                    {
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                        {
                            sysState.alarmStage = STAGE_ALERT;
                            mqttConnected = sysState.mqttConnected;
                            xSemaphoreGive(stateMutex);
                        }

                        if (checkSMSCooldown(SMS_ALERT_ALARM))
                        {
                            enqueueSMS("CANH BAO: Phat hien rung lac manh lien tuc! He thong chuyen sang che do BAO DONG.", SMS_ALERT_ALARM);
                        }

                        sendMQTTSafe();
                        Serial.println("[ALARM] WARNING -> ALERT (strong motion persisted >60s)");
                    }
                }
                else if (motionDetected || strongMotionDetected)
                {
                    if ((millis() - lastMotionTime) >= WARNING_ACTIVITY_TIMEOUT)
                    {
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                        {
                            sysState.motionDetected = false;
                            sysState.strongMotionDetected = false;
                            sysState.strongMotionStartTime = 0;
                            sysState.strongMotionCount = 0;
                            sysState.sentStrongMotionMQTT = false;
                            xSemaphoreGive(stateMutex);
                        }
                        Serial.println("[ALARM] No motion for 2min -> reset motion flags");
                        sendMQTTSafe();
                    }
                }
                break;

            case STAGE_ALERT:
            {
                unsigned long lastGpsCharTime;
                if (millis() - lastBlink >= 1000)
                {
                    ledState = !ledState;
                    digitalWrite(LED_ALERT_PIN, ledState ? HIGH : LOW);
                    lastBlink = millis();
                }

                now = millis();
                if (buzzerIsOn && now >= buzzerOffTime)
                {
                    setBuzzer(false);
                    buzzerIsOn = false;
                    lastBuzzerToggle = now;
                }
                if (!buzzerIsOn && now - lastBuzzerToggle >= 1000)
                {
                    setBuzzer(true);
                    buzzerIsOn = true;
                    buzzerOffTime = now + 200;
                }
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    lastGpsCharTime = sysState.lastGpsCharTime;
                    xSemaphoreGive(stateMutex);
                }
                if (!gpsValid)
                {
                    unsigned long timeSinceLastMotion = now - lastMotionTime;
                    if (timeSinceLastMotion >= WARNING_ACTIVITY_TIMEOUT)
                    {
                        Serial.println("[ALARM] No GPS and no motion for 2 minutes -> reset to WARNING");

                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                        {
                            sysState.alarmStage = STAGE_WARNING;
                            sysState.motionDetected = false;
                            sysState.lastMotionTime = now;
                            sysState.strongMotionDetected = false;
                            sysState.strongMotionStartTime = 0;
                            sysState.strongMotionCount = 0;
                            sysState.sentStrongMotionMQTT = false;
                            xSemaphoreGive(stateMutex);
                        }

                        digitalWrite(BUZZER_PIN, LOW);
                        digitalWrite(LED_ALERT_PIN, HIGH);
                        if (checkSMSCooldown(SMS_ALERT_GPS_LOST))
                        {
                            enqueueSMS("Khong co tin hieu GPS. He thong quay ve che do CANH BAO.", SMS_ALERT_GPS_LOST);
                        }
                        sendMQTTSafe();
                    }
                    else if (gpsSignalLost && (now - lastGpsCharTime > ALERT_NO_GPS_TIMEOUT))
                    {
                        Serial.println("[ALERT] GPS timeout 5 min -> back to WARNING");
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                        {
                            sysState.alarmStage = STAGE_WARNING;
                            sysState.lastMotionTime = now;
                            xSemaphoreGive(stateMutex);
                        }

                        digitalWrite(BUZZER_PIN, LOW);
                        digitalWrite(LED_ALERT_PIN, HIGH);
                        sendMQTTSafe();
                        if (checkSMSCooldown(SMS_ALERT_GPS_LOST))
                        {
                            enqueueSMS("Khong co tin hieu GPS. He thong quay ve che do CANH BAO", SMS_ALERT_GPS_LOST);
                        }
                    }
                }

                if (gpsValid && (initLat == 0.0 && initLng == 0.0))
                {
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        sysState.initLat = currentLat;
                        sysState.initLng = currentLng;
                        sysState.lastPositionMQTTTime = now;
                        xSemaphoreGive(stateMutex);
                    }

                    char msg[200];
                    snprintf(msg, sizeof(msg),
                             "Vi tri phat hien rung manh: https://maps.google.com/?q=%.6f,%.6f",
                             currentLat, currentLng);
                    if (checkSMSCooldown(SMS_ALERT_ALARM))
                        enqueueSMS(msg, SMS_ALERT_ALARM);

                    sendMQTTSafe();
                }

                if (gpsValid && (initLat != 0.0 && initLng != 0.0))
                {
                    float distance = calculateDistance(initLat, initLng, currentLat, currentLng);

                    if (distance >= DISTANCE_THRESHOLD_MAX)
                    {
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE)
                        {
                            sysState.alarmStage = STAGE_TRACKING;
                            sysState.lastPositionSMSTime = millis();
                            mqttConnected = sysState.mqttConnected;
                            xSemaphoreGive(stateMutex);
                        }

                        sendMQTTSafe();

                        digitalWrite(BUZZER_PIN, LOW);
                        digitalWrite(LED_ALERT_PIN, LOW);

                        Serial.println("[ALARM] Transition STAGE_ALERT to STAGE_TRACKING");

                        char buf[200];
                        snprintf(buf, sizeof(buf),
                                 "THEO DOI: Xe da di chuyen hon %.1fm. Vi tri: https://maps.google.com/?q=%.6f,%.6f",
                                 distance, currentLat, currentLng);
                        if (checkSMSCooldown(SMS_ALERT_MOVEMENT))
                        {
                            enqueueSMS(buf, SMS_ALERT_MOVEMENT);
                        }
                    }
                    else if (distance >= DISTANCE_THRESHOLD_MIN && distance < DISTANCE_THRESHOLD_MAX)
                    {
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                        {
                            if (!sysState.sentMidRangeMoveSMS)
                            {
                                char buf[250];
                                snprintf(buf, sizeof(buf),
                                         "BAO DONG: Xe da di chuyen hon %.1fm. Vi tri: https://maps.google.com/?q=%.6f,%.6f. HAY RA KIEM TRA XE",
                                         distance, currentLat, currentLng);

                                if (checkSMSCooldown(SMS_ALERT_MOVEMENT))
                                {
                                    sysState.sentMidRangeMoveSMS = true;
                                    enqueueSMS(buf, SMS_ALERT_MOVEMENT);
                                }

                                sendMQTTSafe();
                            }
                            xSemaphoreGive(stateMutex);
                        }
                    }
                    else if (distance < DISTANCE_THRESHOLD_MIN)
                    {
                        unsigned long timeSinceLastMotion = now - lastMotionTime;

                        if (timeSinceLastMotion >= WARNING_ACTIVITY_TIMEOUT)
                        {
                            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                            {
                                sysState.alarmStage = STAGE_WARNING;
                                sysState.motionDetected = false;
                                sysState.lastMotionTime = now;
                                sysState.strongMotionDetected = false;
                                sysState.strongMotionStartTime = 0;
                                sysState.strongMotionCount = 0;
                                sysState.sentMidRangeMoveSMS = false;
                                sysState.sentStrongMotionMQTT = false;
                                xSemaphoreGive(stateMutex);
                            }

                            char buf[200];
                            snprintf(buf, sizeof(buf),
                                     "BAO DONG: Xe da bi dich chuyen %.1fm. Vi tri: https://maps.google.com/?q=%.6f,%.6f",
                                     distance, currentLat, currentLng);
                            if (checkSMSCooldown(SMS_ALERT_MOVEMENT))
                            {
                                enqueueSMS(buf, SMS_ALERT_MOVEMENT);
                            }

                            sendMQTTSafe();
                            Serial.println("[ALARM] Light move only -> back to STAGE_WARNING");
                        }
                    }
                }

                if (gpsValid && (now - lastPositionMQTTTime >= GPS_UPDATE_INTERVAL_MS))
                {
                    sendMQTTSafe();

                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        sysState.lastPositionMQTTTime = now;
                        xSemaphoreGive(stateMutex);
                    }
                }

                if (!gpsValid && gpsSignalLost)
                {
                    Serial.println("[GPS] Using last known position during alert");
                }
                break;
            }

            case STAGE_TRACKING:
            {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    sysState.sentMidRangeMoveSMS = false;
                    sysState.lastMotionTime = 0;
                    currentLat = sysState.currentLat;
                    currentLng = sysState.currentLng;
                    xSemaphoreGive(stateMutex);
                }
                if (gpsValid && (now - lastPositionMQTTTime >= GPS_UPDATE_INTERVAL_MS))
                {
                    sendMQTTSafe();

                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        sysState.lastPositionMQTTTime = now;
                        xSemaphoreGive(stateMutex);
                    }
                }
                else if (!gpsValid && gpsSignalLost)
                {
                    Serial.println("[GPS] Using last known position during tracking");
                }

                if (gpsValid && sendSMSOnTracking)
                {
                    char buf[200];
                    snprintf(buf, sizeof(buf),
                             "THEO DOI: Vi tri: https://maps.google.com/?q=%.6f,%.6f",
                             currentLat, currentLng);

                    if (checkSMSCooldown(SMS_ALERT_POSITION))
                    {
                        enqueueSMS(buf, SMS_ALERT_POSITION);
                    }

                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        sysState.lastPositionSMSTime = now;
                        xSemaphoreGive(stateMutex);
                    }

                    Serial.println("[ALARM] Sent periodic position SMS during tracking");
                }
                break;
            }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ================== TASK: GPS Handler ==================
void taskGPSHandler(void *param)
{
    while (true)
    {
        esp_task_wdt_reset();
        AlarmStage stage;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            stage = sysState.alarmStage;
            xSemaphoreGive(stateMutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (stage == STAGE_NONE || stage == STAGE_WARNING)
        {
            putGPSToSleep();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (stage == STAGE_ALERT || stage == STAGE_TRACKING)
        {
            wakeGPSFromSleep();
            updateGPS();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ================== TASK: SIM Handler ==================
void taskSIMHandler(void *param)
{
    simSerial.setTimeout(2000);
    SMSMessage outgoing;
    while (true)
    {
        esp_task_wdt_reset();
        AlarmStage alarmStage;
        bool ownerPresent;
        bool simSleep;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            alarmStage = sysState.alarmStage;
            ownerPresent = sysState.ownerPresent;
            simSleep = sysState.simSleepState;
            xSemaphoreGive(stateMutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        UBaseType_t smsWaiting = uxQueueMessagesWaiting(smsQueue);
        if (smsWaiting > 0)
        {
            globalLastSIMActivity = millis();
            if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(10000)) == pdTRUE)
            {
                if (simSleep)
                {
                    wakeSIMFromSleep();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                while (xQueueReceive(smsQueue, &outgoing, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.printf("[SIM] Sending SMS: %.50s...\n", outgoing.message);
                    bool sent = sendSMS(outgoing.message);

                    if (!sent)
                    {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        sent = sendSMS(outgoing.message);
                    }

                    Serial.println(sent ? "[SIM] SMS sent OK" : "[SIM] SMS send FAILED");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

                xSemaphoreGive(simBusySemaphore);
            }
            continue;
        }

        if (!simSleep)
        {
            if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                String unread = readUnreadSMS();
                if (unread.length() > 0)
                {
                    processIncomingSMS(unread);
                }
                xSemaphoreGive(simBusySemaphore);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        bool shouldSleep = false;

        if (alarmStage == STAGE_NONE && ownerPresent)
        {
            shouldSleep = true;
        }
        else if (alarmStage == STAGE_WARNING)
        {
            bool hasMotion = false;
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                hasMotion = sysState.motionDetected || sysState.strongMotionDetected;
                xSemaphoreGive(stateMutex);
            }

            if (!hasMotion && (millis() - globalLastSIMActivity > 30000))
            {
                shouldSleep = true;
            }
        }

        if (shouldSleep && !simSleep)
        {
            if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                if (uxQueueMessagesWaiting(smsQueue) == 0)
                {
                    putSIMToSleep();
                    Serial.println("[SIM] Auto sleep (idle)");
                }
                xSemaphoreGive(simBusySemaphore);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ================== TASK: Sleep Manager ==================
void taskSleepManager(void *param)
{
    while (true)
    {
        esp_task_wdt_reset();
        AlarmStage stage;
        bool ownerPresent;
        unsigned long lastMotionTime;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            stage = sysState.alarmStage;
            ownerPresent = sysState.ownerPresent;
            lastMotionTime = sysState.lastMotionTime;
            xSemaphoreGive(stateMutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (stage == STAGE_NONE && ownerPresent)
        {
            static unsigned long lastActivityCheck = 0;
            static bool waitingForIdle = false;

            UBaseType_t smsWaiting = uxQueueMessagesWaiting(smsQueue);
            if (smsWaiting > 0)
            {
                lastActivityCheck = millis();
                waitingForIdle = true;
                Serial.printf("[SLEEP] Activity detected (%d SMS waiting), reset idle timer\n", smsWaiting);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            if (!waitingForIdle)
            {
                lastActivityCheck = millis();
                waitingForIdle = true;
                Serial.println("[SLEEP] Starting 30s idle countdown before deep sleep...");
            }
            unsigned long idleTime = millis() - lastActivityCheck;
            if (idleTime < 30000)
            {
                Serial.printf("[SLEEP] Waiting for idle... %lu/30000 ms\n", idleTime);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            waitingForIdle = false;

            smsWaiting = uxQueueMessagesWaiting(smsQueue);
            if (smsWaiting > 0)
            {
                Serial.printf("[SLEEP] Found %d SMS after idle wait, restart\n", smsWaiting);
                continue;
            }

            if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(10000)) == pdTRUE)
            {
                Serial.println("[SLEEP] Preparing deep sleep...");

                float batV = readBatteryVoltage();
                int batPct = calcBatteryPercent(batV);
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    sysState.batteryVoltage = batV;
                    sysState.batteryPercent = batPct;
                    xSemaphoreGive(stateMutex);
                }

                bool simSleep = false;
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    simSleep = sysState.simSleepState;
                    xSemaphoreGive(stateMutex);
                }

                if (simSleep)
                {
                    wakeSIMFromSleep();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                if (connectMQTT())
                {
                    sendDataToMQTT();
                    disconnectMQTT();
                    Serial.println("[SLEEP] Final MQTT update sent");
                }
                else
                {
                    Serial.println("[SLEEP] MQTT failed, continuing to deep sleep");
                }

                putSIMToSleep();
                Serial.println("[SIM] Sleeping before deep sleep");

                xSemaphoreGive(simBusySemaphore);
            }
            else
            {
                Serial.println("[SLEEP] Cannot acquire SIM lock for deep sleep, retry...");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            putGPSToSleep();
            putMPUToSleep();

            esp_sleep_enable_timer_wakeup(SLEEP_WAKE_INTERVAL_MS);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)MODE_BUTTON_PIN, 0);

            Serial.println("[SLEEP] Going to deep sleep...");
            Serial.flush();
            vTaskDelay(pdMS_TO_TICKS(200));

            esp_deep_sleep_start();
        }

        if (stage == STAGE_WARNING && !ownerPresent)
        {
            unsigned long timeSinceMotion = millis() - lastMotionTime;

            if (timeSinceMotion > LIGHT_SLEEP_TIMEOUT)
            {
                UBaseType_t smsWaiting = uxQueueMessagesWaiting(smsQueue);
                if (smsWaiting > 0)
                {
                    Serial.printf("[SLEEP] Waiting for %d SMS to be sent...\n", smsWaiting);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }

                if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(5000)) == pdTRUE)
                {
                    Serial.println("[SLEEP] Preparing light sleep...");

                    bool simSleep = false;
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        simSleep = sysState.simSleepState;
                        xSemaphoreGive(stateMutex);
                    }

                    if (simSleep)
                    {
                        wakeSIMFromSleep();
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }

                    if (connectMQTT())
                    {
                        sendDataToMQTT();
                        disconnectMQTT();
                    }

                    putSIMToSleep();
                    Serial.println("[SIM] Sleeping before light sleep");

                    xSemaphoreGive(simBusySemaphore);
                }
                else
                {
                    Serial.println("[SLEEP] Cannot acquire SIM lock, retry later");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                putGPSToSleep();
                writeMPU(MPU6050_PWR_MGMT_1, 0x00);
                vTaskDelay(pdMS_TO_TICKS(50));
                writeMPU(MPU6050_ACCEL_CONFIG, 0x00);
                writeMPU(MPU6050_CONFIG, 0x00);
                writeMPU(MPU6050_MOT_THR, 20);
                writeMPU(MPU6050_MOT_DUR, 1);
                writeMPU(MPU6050_INT_PIN_CONFIG, 0x20);
                writeMPU(MPU6050_INT_ENABLE, 0x40);
                uint8_t intStatus = readMPU(MPU6050_INT_STATUS);
                Serial.println("[SLEEP] Entering light sleep (STAGE_WARNING)");
                Serial.flush();
                vTaskDelay(pdMS_TO_TICKS(200));

                esp_sleep_enable_timer_wakeup(SLEEP_WAKE_INTERVAL_MS);
                esp_sleep_enable_ext0_wakeup((gpio_num_t)MODE_BUTTON_PIN, 0);
                esp_sleep_enable_ext1_wakeup(1ULL << MPU_INT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);
                esp_light_sleep_start();

                esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

                switch (cause)
                {
                case ESP_SLEEP_WAKEUP_EXT0:
                    Serial.println("[WAKE] Button pressed - toggling owner state!");
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        sysState.lastMotionTime = millis();
                        xSemaphoreGive(stateMutex);
                    }
                    break;

                case ESP_SLEEP_WAKEUP_EXT1:
                    Serial.println("[WAKE] MPU interrupt!");
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        sysState.lastMotionTime = millis();
                        sysState.motionDetected = true;
                        xSemaphoreGive(stateMutex);
                    }
                    break;
                case ESP_SLEEP_WAKEUP_TIMER:
                    Serial.println("[WAKE] Timer wakeup");
                    break;
                default:
                    Serial.println("[WAKE] Unknown wakeup reason");
                    break;
                }

                Serial.println("[WAKE] Waking SIM from sleep...");
                for (int i = 0; i < 3; i++)
                {
                    simSerial.println("AT");
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                simSerial.println("AT+CSCLK=0");
                vTaskDelay(pdMS_TO_TICKS(300));
                while (simSerial.available())
                    simSerial.read();

                simSerial.println("AT+CGACT=1,1");
                vTaskDelay(pdMS_TO_TICKS(3000));
                while (simSerial.available())
                    simSerial.read();

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    sysState.simSleepState = false;
                    xSemaphoreGive(stateMutex);
                }
                globalLastSIMActivity = millis();
                Serial.println("[WAKE] SIM ready");

                if (cause == ESP_SLEEP_WAKEUP_EXT1)
                {
                    Serial.println("[WAKE] Sending MQTT update after motion wake...");
                    if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(5000)) == pdTRUE)
                    {
                        if (connectMQTT())
                        {
                            sendDataToMQTT();
                            disconnectMQTT();
                            Serial.println("[WAKE] MQTT update sent");
                        }
                        else
                        {
                            Serial.println("[WAKE] MQTT connect failed");
                        }
                        xSemaphoreGive(simBusySemaphore);
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

// ================== TASK: Battery Monitor ==================
void taskBatteryMonitor(void *param)
{
    while (true)
    {
        esp_task_wdt_reset();
        float batV = readBatteryVoltage();
        int batPct = calcBatteryPercent(batV);
        bool lowBattery = false;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            sysState.batteryVoltage = batV;
            sysState.batteryPercent = batPct;
            lowBattery = sysState.lowBattery;
            xSemaphoreGive(stateMutex);
        }

        if (batPct <= 20 && !lowBattery)
        {
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sysState.lowBattery = true;
                xSemaphoreGive(stateMutex);
            }

            char buf[120];
            snprintf(buf, sizeof(buf), "CANH BAO: Pin thap: %d%%. Vui long sac pin.", batPct);
            if (checkSMSCooldown(SMS_ALERT_LOW_BATTERY))
            {
                enqueueSMS(buf, SMS_ALERT_LOW_BATTERY);
            }

            sendMQTTSafe();
        }
        else if (batPct >= 80 && lowBattery)
        {
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                sysState.lowBattery = false;
                xSemaphoreGive(stateMutex);
            }
        }

        for (int i = 0; i < 15 * 60; i++)
        {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
