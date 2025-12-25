#include "sms_handler.h"
#include "globals.h"
#include "config.h"
#include "mqtt_handler.h"
#include <Preferences.h>

const unsigned long SMS_COOLDOWN[] = {
    20000UL,  // ALARM 20s
    60000UL,  // MOVEMENT 1min
    60000UL,  // POSITION 1min
    180000UL, // GPS_LOST 3min
    900000UL, // LOW_BATTERY 15min
    600000UL, // SYSTEM_ERROR 10min
    1000UL    // EMERGENCY 1s
};

bool checkSMSCooldown(SMSAlertType type)
{
    unsigned long now = millis();
    // Lần đầu tiên cho phép gửi
    if (lastSMSSentTime[type] == 0)
    {
        lastSMSSentTime[type] = now;
        preferences.begin("sms_cd", false);
        preferences.putULong(("t" + String((int)type)).c_str(), now);
        preferences.end();
        Serial.printf("[SMS] First send for type %d allowed\n", type);
        return true;
    }
    // Xử lý overflow của millis()
    if (lastSMSSentTime[type] > now)
    {
        lastSMSSentTime[type] = 0;
        Serial.println("[SMS] millis() overflow detected, reset cooldown");
        return true;
    }
    // Check cooldown
    if (now - lastSMSSentTime[type] >= SMS_COOLDOWN[type])
    {
        lastSMSSentTime[type] = now;
        preferences.begin("sms_cd", false);
        preferences.putULong(("t" + String((int)type)).c_str(), lastSMSSentTime[type]);
        preferences.end();
        return true;
    }
    Serial.printf("[SMS] Type %d still in cooldown (%lu/%lu ms)\n",
                  type, now - lastSMSSentTime[type], SMS_COOLDOWN[type]);
    return false;
}

void loadSMSCooldown()
{
    preferences.begin("sms_cd", true);
    for (int i = 0; i < 7; i++)
    {
        lastSMSSentTime[i] = preferences.getULong(("t" + String(i)).c_str(), 0);
    }
    preferences.end();
}

bool enqueueSMS(const char *msg, SMSAlertType type)
{
    SMSMessage m;
    memset(&m, 0, sizeof(m));
    strncpy(m.message, msg, sizeof(m.message) - 1);
    m.type = type;
    if (xQueueSend(smsQueue, &m, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("Enqueued SMS:");
        Serial.println(m.message);
        return true;
    }
    else
    {
        Serial.println("SMS queue full, enqueue failed");
        return false;
    }
}

bool sendSMS(const char *msg)
{
    simSerial.print("AT+CMGS=\"");
    simSerial.print(PHONE_NUMBER);
    simSerial.println("\"");
    vTaskDelay(pdMS_TO_TICKS(3000));

    simSerial.print(msg);
    simSerial.write(26); // Ctrl+Z

    unsigned long start = millis();
    String resp = "";
    while (millis() - start < 10000)
    {
        while (simSerial.available())
        {
            resp += (char)simSerial.read();
        }
        if (resp.indexOf("OK") >= 0 || resp.indexOf("CMGS") >= 0)
        {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

String readUnreadSMS()
{
    simSerial.println("AT+CMGL=\"REC UNREAD\"");
    unsigned long tstart = millis();
    String listResp = "";
    while (millis() - tstart < 2000)
    {
        while (simSerial.available())
        {
            listResp += (char)simSerial.read();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return listResp;
}

void processIncomingSMS(const String &resp)
{
    int pos = 0;

    while (true)
    {
        int nl = resp.indexOf("\n", pos);
        if (nl < 0)
            break;

        String line = resp.substring(pos, nl);
        pos = nl + 1;
        line.trim();

        if (line.startsWith("+CMGL:"))
        {
            int nextNl = resp.indexOf("\n", pos);
            if (nextNl < 0)
                break;

            String body = resp.substring(pos, nextNl);
            pos = nextNl + 1;
            body.trim();
            String low = body;
            low.toLowerCase();

            Serial.print("[SIM] SMS: ");
            Serial.println(body);

            if (low.indexOf("gps") >= 0)
            {
                float lat, lng;
                bool gpsOk;

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    gpsOk = sysState.gpsValid;
                    lat = sysState.currentLat;
                    lng = sysState.currentLng;
                    xSemaphoreGive(stateMutex);
                }

                if (!gpsOk)
                {
                    if (checkSMSCooldown(SMS_ALERT_SYSTEM_ERROR))
                    {
                        enqueueSMS("Khong the lay vi tri: GPS chua san sang", SMS_ALERT_SYSTEM_ERROR);
                    }
                }
                else
                {
                    char buf[160];
                    snprintf(buf, sizeof(buf), "VI TRI: https://maps.google.com/?q=%.6f,%.6f", lat, lng);
                    if (checkSMSCooldown(SMS_ALERT_EMERGENCY))
                        enqueueSMS(buf, SMS_ALERT_EMERGENCY);
                }
            }
            else if (low.indexOf("stop tracking") >= 0 || (low.indexOf("stop") >= 0 && low.indexOf("tracking") >= 0))
            {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    sysState.alarmStage = STAGE_NONE;
                    sysState.motionDetected = false;
                    sysState.ownerPresent = true;
                    sysState.gpsValid = false;
                    sysState.initLat = 0.0;
                    sysState.initLng = 0.0;
                    sysState.currentLat = 0.0;
                    sysState.currentLng = 0.0;
                    sysState.strongMotionDetected = false;
                    sysState.motionDetected = false;
                    xSemaphoreGive(stateMutex);
                }
                if (checkSMSCooldown(SMS_ALERT_EMERGENCY))
                {
                    enqueueSMS("Da chuyen ve che do cho", SMS_ALERT_EMERGENCY);
                }

                preferences.begin("system", false);
                preferences.putBool("ownerPresent", true);
                preferences.end();

                preferences.begin("sms_cd", false);
                preferences.clear();
                preferences.end();

                sendMQTTSafe();

                Serial.println("[SIM] STOP_TRACKING processed");
            }
            else if (low.indexOf("stop sms") >= 0)
            {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    sysState.sendSMSOnTracking = false;
                    xSemaphoreGive(stateMutex);
                }

                if (checkSMSCooldown(SMS_ALERT_EMERGENCY))
                {
                    enqueueSMS("Da ngung gui SMS vi tri", SMS_ALERT_EMERGENCY);
                }
                Serial.println("[SIM] STOP SMS processed");
            }
        }
    }
}
