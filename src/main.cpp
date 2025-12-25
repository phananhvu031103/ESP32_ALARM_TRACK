#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include <ArduinoJson.h>

#include "config.h"
#include "types.h"
#include "globals.h"
#include "hardware.h"
#include "tasks.h"
#include "mqtt_handler.h"
#include "sms_handler.h"

// ================== Global Variable Definitions ==================
volatile SystemState sysState = {
    .batteryVoltage = 0.0f,
    .currentLat = 0.0f,
    .currentLng = 0.0f,
    .initLat = 0.0f,
    .initLng = 0.0f,
    .lastMotionTime = 0,
    .lastPositionMQTTTime = 0,
    .lastPositionSMSTime = 0,
    .lastGpsCharTime = 0,
    .strongMotionStartTime = 0,
    .batteryPercent = 0,
    .strongMotionCount = 0,
    .alarmStage = STAGE_NONE,
    .ownerPresent = true,
    .motionDetected = false,
    .simConnected = false,
    .lowBattery = false,
    .gpsValid = false,
    .gpsSignalLost = false,
    .hadValidFix = false,
    .mqttConnected = false,
    .mpuSleepState = false,
    .gpsSleepState = true,
    .simSleepState = true,
    .strongMotionDetected = false,
    .sendSMSOnTracking = true,
    .sentMidRangeMoveSMS = false,
    .sentStrongMotionMQTT = false};

volatile unsigned long globalLastSIMActivity = 0;

// FreeRTOS handles
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t simBusySemaphore;
QueueHandle_t smsQueue;
QueueHandle_t buttonEventQueue;

TaskHandle_t taskHandleMPU;
TaskHandle_t taskHandleGPS;
TaskHandle_t taskHandleSIM;
TaskHandle_t taskHandleAlarm;
TaskHandle_t taskHandleButton;
TaskHandle_t taskHandleSleep;
TaskHandle_t taskHandleBattery;

// Hardware Serial
HardwareSerial gpsSerial(1);
HardwareSerial simSerial(2);

// Hardware Objects
MPUKalmanFilter mpuKalman(0.5, 1.0, 0.05);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
GPSKalmanFilter gpsKalman(0.1, 1.0, 0.01);
Preferences preferences;

// SMS Cooldown
unsigned long lastSMSSentTime[7] = {0};

// MPU Interrupt
volatile bool mpuInterruptFlag = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================== SETUP ==================
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_ALERT_PIN, OUTPUT);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MPU_INT_PIN, INPUT);
    digitalWrite(LED_ALERT_PIN, LOW);

    Serial.println("====ESP32 GPS Tracking System====");

    initBatteryADC();

    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    simSerial.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

    // Create mutexes and queues
    stateMutex = xSemaphoreCreateMutex();
    smsQueue = xQueueCreate(10, sizeof(SMSMessage));
    buttonEventQueue = xQueueCreate(5, sizeof(int));
    simBusySemaphore = xSemaphoreCreateMutex();

    // Initialize state defaults
    sysState.alarmStage = STAGE_NONE;
    sysState.motionDetected = false;
    sysState.strongMotionDetected = false;
    sysState.simConnected = false;
    sysState.gpsValid = false;
    sysState.lowBattery = false;
    sysState.lastMotionTime = millis();
    sysState.lastPositionMQTTTime = 0;
    sysState.strongMotionStartTime = 0;
    sysState.strongMotionCount = 0;
    sysState.sentStrongMotionMQTT = false;
    sysState.lastGpsCharTime = millis();
    sysState.sendSMSOnTracking = true;
    sysState.sentMidRangeMoveSMS = false;
    sysState.mqttConnected = false;
    sysState.initLat = 0.0;
    sysState.initLng = 0.0;

    loadSMSCooldown();

    // Check wakeup cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    preferences.begin("system", false);
    if (!preferences.isKey("ownerPresent"))
    {
        preferences.putBool("ownerPresent", true);
        Serial.println("[INIT] NVS initialized with defaults");
    }

    bool savedOwner = preferences.getBool("ownerPresent", true);

    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Woke up from EXT0 (MODE Button)");
        savedOwner = !savedOwner;
        preferences.putBool("ownerPresent", savedOwner);
        sysState.ownerPresent = savedOwner;
        if (!sysState.ownerPresent)
        {
            sysState.alarmStage = STAGE_WARNING;
            sysState.lastMotionTime = millis();
            Serial.println("[INIT] MODE button -> STAGE_WARNING");
        }
        else
        {
            sysState.alarmStage = STAGE_NONE;
            Serial.println("[INIT] MODE button -> STAGE_NONE");
        }
        break;

    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Woke up from Timer");
        sysState.ownerPresent = savedOwner;
        if (sysState.ownerPresent)
        {
            sysState.alarmStage = STAGE_NONE;
        }
        break;

    default:
        Serial.println("Power on or other wakeup reason");
        sysState.ownerPresent = savedOwner;
        if (sysState.ownerPresent)
        {
            sysState.alarmStage = STAGE_NONE;
        }
        else
        {
            sysState.alarmStage = STAGE_WARNING;
        }
        break;
    }
    preferences.end();

    if (sysState.ownerPresent)
    {
        Serial.println("[INIT] Owner present on startup, STAGE_NONE");
    }
    else
    {
        Serial.println("[INIT] Owner absent on startup, STAGE_WARNING");
    }

    // Initialize SIM
    bool simOk = initSIM();

    // Send initial data via MQTT
    float batV = readBatteryVoltage();
    int batPct = calcBatteryPercent(batV);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        sysState.batteryVoltage = batV;
        sysState.batteryPercent = batPct;
        xSemaphoreGive(stateMutex);
    }

    if (simOk)
    {
        sendMQTTSafe();
        Serial.println("[INIT] SIM initialized, ready for MQTT");
    }
    else
    {
        Serial.println("[INIT] SIM initialization failed");
    }

    // Initialize Kalman filters
    mpuKalman.reset();
    gpsKalman.reset(0, 0);

    // Attach MPU interrupt
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), IRAM_mpuISR, RISING);

    playBeepPattern(2, 100, 100);
    Serial.println("System started");
    Serial.println("---------------------------------------");

    globalLastSIMActivity = millis();

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(taskMPUHandler, "MPU", 4096, NULL, 4, &taskHandleMPU, 0);
    xTaskCreatePinnedToCore(taskAlarmManager, "Alarm", 4096, NULL, 3, &taskHandleAlarm, 0);
    xTaskCreatePinnedToCore(taskButtonHandler, "Button", 4096, NULL, 3, &taskHandleButton, 0);
    xTaskCreatePinnedToCore(taskSleepManager, "Sleep", 4096, NULL, 3, &taskHandleSleep, 0);
    xTaskCreatePinnedToCore(taskBatteryMonitor, "Battery", 4096, NULL, 1, &taskHandleBattery, 0);
    xTaskCreatePinnedToCore(taskGPSHandler, "GPS", 8192, NULL, 3, &taskHandleGPS, 1);
    xTaskCreatePinnedToCore(taskSIMHandler, "SIM", 8192, NULL, 3, &taskHandleSIM, 1);

    // Initialize watchdog 60s, panic = true
    esp_task_wdt_init(60, true);

    // Add tasks to watchdog
    esp_task_wdt_add(taskHandleMPU);
    esp_task_wdt_add(taskHandleAlarm);
    esp_task_wdt_add(taskHandleButton);
    esp_task_wdt_add(taskHandleSleep);
    esp_task_wdt_add(taskHandleBattery);
    esp_task_wdt_add(taskHandleGPS);
    esp_task_wdt_add(taskHandleSIM);
}

// ================== LOOP ==================
void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}
