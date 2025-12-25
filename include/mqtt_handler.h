#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <Arduino.h>

// ================== MQTT Functions ==================
String sendMqttCmd(String cmd, int wait = 2000);
bool sendMqttCmdExpect(String cmd, String expect, int wait = 2000);
String waitMqttResponse(String pattern, int timeout = 15000);
bool connectMQTT();
void disconnectMQTT();
bool publishMQTT(const char *topic, const char *payload);
bool sendDataToMQTT();
bool sendMQTTSafe();

#endif // MQTT_HANDLER_H
