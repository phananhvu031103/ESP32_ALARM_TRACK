#include "mqtt_handler.h"
#include "globals.h"
#include "config.h"
#include "hardware.h"
#include <ArduinoJson.h>

String sendMqttCmd(String cmd, int wait)
{
    while (simSerial.available())
        simSerial.read(); // Clear buffer

    Serial.print(">> ");
    Serial.println(cmd);

    simSerial.println(cmd);

    String resp = "";
    unsigned long start = millis();

    while (millis() - start < wait)
    {
        while (simSerial.available())
        {
            char c = simSerial.read();
            resp += c;
            Serial.write(c);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println();
    return resp;
}

bool sendMqttCmdExpect(String cmd, String expect, int wait)
{
    String resp = sendMqttCmd(cmd, wait);
    return resp.indexOf(expect) >= 0;
}

String waitMqttResponse(String pattern, int timeout)
{
    String resp = "";
    unsigned long start = millis();

    while (millis() - start < timeout)
    {
        while (simSerial.available())
        {
            char c = simSerial.read();
            resp += c;
            Serial.write(c);
        }

        if (resp.indexOf(pattern) >= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(300));
            while (simSerial.available())
            {
                resp += (char)simSerial.read();
            }
            return resp;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return resp;
}

bool connectMQTT()
{
    Serial.println("[MQTT] Connecting to broker...");

    // 1. Dừng MQTT session cũ (ignore errors)
    simSerial.println("AT+CMQTTDISC=0,60");
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (simSerial.available())
        simSerial.read();

    simSerial.println("AT+CMQTTREL=0");
    vTaskDelay(pdMS_TO_TICKS(500));
    while (simSerial.available())
        simSerial.read();

    simSerial.println("AT+CMQTTSTOP");
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (simSerial.available())
        simSerial.read();

    // 2. Khởi động MQTT service
    simSerial.println("AT+CMQTTSTART");
    String startResp = waitMqttResponse("+CMQTTSTART:", 5000);

    if (startResp.indexOf("+CMQTTSTART: 0") < 0 && startResp.indexOf("OK") < 0)
    {
        // Thử stop và start lại
        simSerial.println("AT+CMQTTSTOP");
        vTaskDelay(pdMS_TO_TICKS(1000));
        while (simSerial.available())
            simSerial.read();

        simSerial.println("AT+CMQTTSTART");
        startResp = waitMqttResponse("+CMQTTSTART:", 5000);
    }

    // 3. Tạo MQTT client (KHÔNG có TLS -> không có ",1" ở cuối)
    String accqCmd = "AT+CMQTTACCQ=0,\"" + String(MQTT_CLIENT_ID) + "\"";
    if (!sendMqttCmdExpect(accqCmd, "OK", 3000))
    {
        Serial.println("[MQTT] ACCQ failed!");
        return false;
    }

    // 4. Kết nối MQTT broker (không username/password cho public broker)
    String connectCmd = "AT+CMQTTCONNECT=0,\"tcp://" + String(MQTT_BROKER) + ":" + String(MQTT_PORT) + "\",60,1";

    simSerial.println(connectCmd);
    String connResp = waitMqttResponse("+CMQTTCONNECT:", 20000);

    if (connResp.indexOf("+CMQTTCONNECT: 0,0") >= 0)
    {
        Serial.println("[MQTT] Connected successfully!");

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            sysState.mqttConnected = true;
            xSemaphoreGive(stateMutex);
        }
        return true;
    }

    // Parse error code
    int errIdx = connResp.indexOf("+CMQTTCONNECT: 0,");
    if (errIdx >= 0)
    {
        int errCode = connResp.substring(errIdx + 17, errIdx + 19).toInt();
        Serial.printf("[MQTT] Connection failed, error code: %d\n", errCode);
    }
    else
    {
        Serial.println("[MQTT] Connection failed (timeout)");
    }

    return false;
}

void disconnectMQTT()
{
    Serial.println("[MQTT] Disconnecting...");

    simSerial.println("AT+CMQTTDISC=0,60");
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (simSerial.available())
        simSerial.read();

    simSerial.println("AT+CMQTTREL=0");
    vTaskDelay(pdMS_TO_TICKS(500));
    while (simSerial.available())
        simSerial.read();

    simSerial.println("AT+CMQTTSTOP");
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (simSerial.available())
        simSerial.read();

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        sysState.mqttConnected = false;
        xSemaphoreGive(stateMutex);
    }

    Serial.println("[MQTT] Disconnected");
}

bool publishMQTT(const char *topic, const char *payload)
{
    int topicLen = strlen(topic);
    int payloadLen = strlen(payload);

    Serial.printf("[MQTT] Publishing to %s (%d bytes)\n", topic, payloadLen);

    // 1. Set topic
    String topicCmd = "AT+CMQTTTOPIC=0," + String(topicLen);
    simSerial.println(topicCmd);

    // Đợi prompt ">"
    String topicResp = "";
    unsigned long t0 = millis();
    while (millis() - t0 < 3000)
    {
        while (simSerial.available())
        {
            char c = simSerial.read();
            topicResp += c;
            Serial.write(c);
        }
        if (topicResp.indexOf(">") >= 0)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (topicResp.indexOf(">") < 0)
    {
        Serial.println("[MQTT] Topic prompt timeout");
        return false;
    }

    // Gửi topic
    simSerial.print(topic);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Đọc response
    t0 = millis();
    while (millis() - t0 < 1000)
    {
        while (simSerial.available())
            Serial.write(simSerial.read());
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 2. Set payload
    String payloadCmd = "AT+CMQTTPAYLOAD=0," + String(payloadLen);
    simSerial.println(payloadCmd);

    // Đợi prompt ">"
    String payResp = "";
    t0 = millis();
    while (millis() - t0 < 3000)
    {
        while (simSerial.available())
        {
            char c = simSerial.read();
            payResp += c;
            Serial.write(c);
        }
        if (payResp.indexOf(">") >= 0)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (payResp.indexOf(">") < 0)
    {
        Serial.println("[MQTT] Payload prompt timeout");
        return false;
    }

    // Gửi payload
    simSerial.print(payload);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Đọc response
    t0 = millis();
    while (millis() - t0 < 1000)
    {
        while (simSerial.available())
            Serial.write(simSerial.read());
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 3. Publish
    simSerial.println("AT+CMQTTPUB=0,1,60");
    String pubResp = waitMqttResponse("+CMQTTPUB:", 10000);

    if (pubResp.indexOf("+CMQTTPUB: 0,0") >= 0)
    {
        Serial.println("[MQTT] Publish OK");
        return true;
    }

    Serial.println("[MQTT] Publish failed");
    return false;
}

bool sendDataToMQTT()
{
    StaticJsonDocument<512> doc;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        doc["battery_percent"] = sysState.batteryPercent;
        doc["battery_voltage"] = sysState.batteryVoltage;
        doc["owner_present"] = sysState.ownerPresent ? 1 : 0;
        doc["alarm_stage"] = (int)sysState.alarmStage;
        doc["motion_detected"] = sysState.motionDetected ? 1 : 0;
        doc["mqtt_connected"] = 1; // Đang gửi nên chắc chắn connected

        doc["gps_valid"] = sysState.gpsValid ? 1 : 0;
        if (sysState.gpsValid)
        {
            doc["latitude"] = sysState.currentLat;
            doc["longitude"] = sysState.currentLng;
        }

        doc["strong_motion"] = sysState.strongMotionDetected ? 1 : 0;
        doc["low_battery"] = sysState.lowBattery ? 1 : 0;
        doc["timestamp"] = millis();

        xSemaphoreGive(stateMutex);
    }

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    Serial.print("[MQTT] Sending: ");
    Serial.println(jsonBuffer);

    return publishMQTT(MQTT_TOPIC_PUB, jsonBuffer);
}

bool sendMQTTSafe()
{
    bool success = false;

    if (xSemaphoreTake(simBusySemaphore, pdMS_TO_TICKS(10000)) == pdTRUE)
    {
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

        // Retry connect 2 lần
        for (int retry = 0; retry < 2 && !success; retry++)
        {
            if (connectMQTT())
            {
                success = sendDataToMQTT();
                disconnectMQTT();
            }

            if (!success && retry < 1)
            {
                Serial.println("[MQTT] Retry...");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        xSemaphoreGive(simBusySemaphore);
    }
    else
    {
        Serial.println("[MQTT] Cannot acquire SIM lock");
    }

    return success;
}
