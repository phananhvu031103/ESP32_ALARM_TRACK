#include <Arduino.h>
#include <Wire.h>
#include "hardware.h"
#include "globals.h"
#include "config.h"
#include "sms_handler.h"
#include "mqtt_handler.h"
#include <Adafruit_Sensor.h>

// ================== MPU Functions ==================
void writeMPU(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readMPU(uint8_t reg)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 1);
    if (Wire.available())
    {
        return Wire.read();
    }
    return 0;
}

void putMPUToSleep()
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (sysState.mpuSleepState)
        {
            xSemaphoreGive(stateMutex);
            return;
        }
        sysState.mpuSleepState = true;
        xSemaphoreGive(stateMutex);
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x40);
    Wire.endTransmission();
    Serial.println("[MPU] sleep");
}

void wakeMPUFromSleep()
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (!sysState.mpuSleepState)
        {
            xSemaphoreGive(stateMutex);
            return;
        }
        sysState.mpuSleepState = false;
        xSemaphoreGive(stateMutex);
    }
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Warm up Kalman filter sau khi wake
    for (int i = 0; i < 10; i++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float fx, fy, fz;
        mpuKalman.updateAcceleration(a.acceleration.x, a.acceleration.y, a.acceleration.z, &fx, &fy, &fz);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("[MPU] wake");
}

float calcDeviation()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float fx, fy, fz;
    mpuKalman.updateAcceleration(a.acceleration.x, a.acceleration.y, a.acceleration.z, &fx, &fy, &fz);

    float totalAcc = sqrt(fx * fx + fy * fy + fz * fz);
    float deviation = fabs(totalAcc - 9.8f);
    if (fabs(deviation) < 1.0)
        deviation = 0.0f;
    return deviation;
}

void IRAM_ATTR IRAM_mpuISR()
{
    portENTER_CRITICAL_ISR(&mux);
    mpuInterruptFlag = true;
    portEXIT_CRITICAL_ISR(&mux);
}

// ================== GPS Functions ==================
void putGPSToSleep()
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (sysState.gpsSleepState)
        {
            xSemaphoreGive(stateMutex);
            return;
        }
        sysState.gpsSleepState = true;
        xSemaphoreGive(stateMutex);
    }
    gpsSerial.print(GPS_SLEEP_CMD);
    Serial.println("[GPS] sleep cmd sent");
}

void wakeGPSFromSleep()
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (!sysState.gpsSleepState)
        {
            xSemaphoreGive(stateMutex);
            return;
        }
        sysState.gpsSleepState = false;
        xSemaphoreGive(stateMutex);
    }
    gpsSerial.print(GPS_WAKE_CMD);
    vTaskDelay(pdMS_TO_TICKS(800));
    Serial.println("[GPS] wake cmd sent");
}

bool updateGPS()
{
    bool updated = false;
    unsigned long start = millis();
    unsigned long lastCharUpdate = 0;

    while (millis() - start < 300)
    {
        while (gpsSerial.available() > 0)
        {
            char c = (char)gpsSerial.read();
            gps.encode(c);
            updated = true;

            unsigned long now = millis();
            if (now - lastCharUpdate > 50)
            {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sysState.lastGpsCharTime = now;
                    xSemaphoreGive(stateMutex);
                }
                lastCharUpdate = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (gps.location.isValid())
    {
        float rawLat = gps.location.lat();
        float rawLng = gps.location.lng();
        float filteredLat, filteredLng;
        gpsKalman.updatePosition(rawLat, rawLng, &filteredLat, &filteredLng);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            sysState.currentLat = filteredLat;
            sysState.currentLng = filteredLng;
            sysState.gpsValid = true;

            if (!sysState.hadValidFix)
            {
                sysState.hadValidFix = true;
                sysState.gpsSignalLost = false;
                gpsKalman.reset(rawLat, rawLng);
                Serial.println("[GPS] first valid fix");
            }
            else if (sysState.gpsSignalLost)
            {
                sysState.gpsSignalLost = false;
                Serial.println("[GPS] signal recovered");
            }
            xSemaphoreGive(stateMutex);
        }
        return true;
    }

    // Handle signal loss
    unsigned long lastChar = 0;
    bool hadFix = false;
    bool gpsSignalLost = false;
    float lastLat = 0.0, lastLng = 0.0;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        lastChar = sysState.lastGpsCharTime;
        hadFix = sysState.hadValidFix;
        gpsSignalLost = sysState.gpsSignalLost;
        lastLat = sysState.currentLat;
        lastLng = sysState.currentLng;
        xSemaphoreGive(stateMutex);
    }

    if (hadFix && (millis() - lastChar > 5000))
    {
        bool needNotify = false;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (!sysState.gpsSignalLost)
            {
                sysState.gpsSignalLost = true;
                needNotify = true;
            }
            xSemaphoreGive(stateMutex);
        }
        if (needNotify)
        {
            Serial.println("[GPS] signal lost - using last known position");
            char msg[160];
            snprintf(msg, sizeof(msg), "CANH BAO: Mat tin hieu GPS. Vi tri cuoi: https://maps.google.com/?q=%.6f,%.6f", lastLat, lastLng);
            if (checkSMSCooldown(SMS_ALERT_GPS_LOST))
            {
                enqueueSMS(msg, SMS_ALERT_GPS_LOST);
            }
        }
    }
    return false;
}

// ================== SIM Functions ==================
bool sendATExpect(const char *cmd, const char *expect, unsigned long timeoutMs)
{
    simSerial.print(cmd);
    simSerial.print("\r\n");
    unsigned long start = millis();
    String resp = "";
    while (millis() - start < timeoutMs)
    {
        while (simSerial.available())
        {
            resp += (char)simSerial.read();
        }
        if (expect != nullptr && expect[0] != '\0')
        {
            if (resp.indexOf(expect) >= 0)
                return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

bool initSIM()
{
    Serial.println("Initializing SIM A7682S ...");

    // 1. Wake SIM nếu đang sleep
    Serial.println("[SIM] Sending wake sequence...");
    for (int i = 0; i < 5; i++)
    {
        simSerial.println("AT");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 2. Disable sleep mode
    simSerial.println("AT+CSCLK=0");
    vTaskDelay(pdMS_TO_TICKS(500));
    while (simSerial.available())
        simSerial.read();

    // 3. Kiểm tra module response
    bool responded = false;
    for (int retry = 0; retry < 3; retry++)
    {
        if (sendATExpect("AT", "OK", 2000))
        {
            responded = true;
            break;
        }
        Serial.printf("[SIM] No response, retry %d/3\n", retry + 1);

        // Thử wake lại
        simSerial.println("AT");
        vTaskDelay(pdMS_TO_TICKS(200));
        simSerial.println("AT+CSCLK=0");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!responded)
    {
        Serial.println("SIM module not responding.");
        return false;
    }

    // 4. Config cơ bản
    sendATExpect("AT+CMGF=1", "OK", 2000); // SMS text mode
    sendATExpect("AT+CREG?", "OK", 2000);  // Check registration

    // 5. Attach GPRS
    if (!sendATExpect("AT+CGATT=1", "OK", 5000))
    {
        Serial.println("CGATT failed!");
        return false;
    }

    // 6. Set APN
    String apnCmd = "AT+CGDCONT=1,\"IP\",\"" + String(SIM_APN) + "\"";
    sendATExpect(apnCmd.c_str(), "OK", 2000);

    // 7. Activate PDP context
    if (!sendATExpect("AT+CGACT=1,1", "OK", 8000))
    {
        Serial.println("CGACT failed!");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        sysState.simSleepState = false;
        sysState.simConnected = true;
        xSemaphoreGive(stateMutex);
    }

    Serial.println("SIM initialized successfully.");
    return true;
}

void putSIMToSleep()
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (sysState.simSleepState)
        {
            xSemaphoreGive(stateMutex);
            return;
        }
        sysState.simSleepState = true;
        xSemaphoreGive(stateMutex);
    }

    // 1. Đảm bảo MQTT đã đóng hoàn toàn
    simSerial.println("AT+CMQTTDISC=0,60");
    vTaskDelay(pdMS_TO_TICKS(500));
    while (simSerial.available())
        simSerial.read();

    simSerial.println("AT+CMQTTREL=0");
    vTaskDelay(pdMS_TO_TICKS(300));
    while (simSerial.available())
        simSerial.read();

    simSerial.println("AT+CMQTTSTOP");
    vTaskDelay(pdMS_TO_TICKS(500));
    while (simSerial.available())
        simSerial.read();

    // 2. Deactivate PDP context - QUAN TRỌNG!
    simSerial.println("AT+CGACT=0,1");
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (simSerial.available())
        simSerial.read();

    // 3. Giờ mới có thể sleep
    simSerial.println("AT+CSCLK=2");
    vTaskDelay(pdMS_TO_TICKS(200));

    Serial.println("[SIM] sleep cmd sent (network deactivated)");
}

void wakeSIMFromSleep()
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (!sysState.simSleepState)
        {
            xSemaphoreGive(stateMutex);
            return;
        }
        sysState.simSleepState = false;
        xSemaphoreGive(stateMutex);
    }

    // 1. Wake up module
    simSerial.println("AT");
    vTaskDelay(pdMS_TO_TICKS(100));
    simSerial.println("AT");
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2. Disable sleep mode
    simSerial.println("AT+CSCLK=0");
    vTaskDelay(pdMS_TO_TICKS(300));
    while (simSerial.available())
        simSerial.read();

    // 3. Reactivate PDP context
    simSerial.println("AT+CGACT=1,1");
    unsigned long start = millis();
    String resp = "";
    while (millis() - start < 5000)
    {
        while (simSerial.available())
        {
            resp += (char)simSerial.read();
        }
        if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0)
            break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    Serial.println("[SIM] wake cmd sent (network reactivated)");
}

// ================== Battery Functions ==================
void setBuzzer(bool state)
{
    digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}

void playBeepPattern(int count, int onTime, int offTime)
{
    for (int i = 0; i < count; i++)
    {
        setBuzzer(true);
        vTaskDelay(pdMS_TO_TICKS(onTime));
        setBuzzer(false);
        if (i < count - 1)
            vTaskDelay(pdMS_TO_TICKS(offTime));
    }
}

void initBatteryADC()
{
    analogSetAttenuation(ADC_11db); // Full range: 3.3V
    analogSetWidth(ADC_11db);
    adcAttachPin(BATTERY_PIN);

    Serial.println("[ADC] Battery ADC initialized");
}

float readBatteryVoltage()
{
    // Đo 10 lân lấy trung bình
    float vOut[10] = {0};
    float vOutSum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        vOut[i] = analogRead(BATTERY_PIN) * (3.3 / 4095.0);
        vOutSum += vOut[i];
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float voltage = vOutSum / 10.0 * 6.0 * 1.05; // Scale up according to voltage divider and calibration
    return constrain(voltage, BATTERY_LOW_THRESHOLD, BATTERY_HIGH_THRESHOLD);
}

int calcBatteryPercent(float voltage)
{
    int percentage = ((voltage - BATTERY_LOW_THRESHOLD) / (BATTERY_HIGH_THRESHOLD - BATTERY_LOW_THRESHOLD)) * 100;
    return constrain(percentage, 0, 100);
}

// ================== Utility Functions ==================
float calculateDistance(float lat1, float lng1, float lat2, float lng2)
{
    float lat1_rad = lat1 * DEG_TO_RAD;
    float lng1_rad = lng1 * DEG_TO_RAD;
    float lat2_rad = lat2 * DEG_TO_RAD;
    float lng2_rad = lng2 * DEG_TO_RAD;

    float dlng = lng2_rad - lng1_rad;
    float dlat = lat2_rad - lat1_rad;
    float a = pow(sin(dlat / 2), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(dlng / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000 * c;
}
