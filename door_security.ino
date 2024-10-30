#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "fauxmoESP.h"
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <ESPTelnet.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const int pirPin = 13;
const int ledPin = 2;  // Built-in LED pin for ESP32

int motionDetected = 0;
bool systemActive = true;

unsigned long lastBlinkTime = 0;
unsigned long lastMotionDetectedTime = 0;
int ledState = LOW;
int blinkCount = 0;
const unsigned long motionCooldownPeriod = 30000;
const int WDT_TIMEOUT = 30;

const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

#define BOT_TOKEN ""
#define CHAT_ID ""

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

fauxmoESP fauxmo;

ESPTelnet telnet;
uint16_t telnetPort = 23;

const long gmtOffset_sec = 19800;  // IST is UTC+5:30
const int daylightOffset_sec = 0;

TaskHandle_t otaTask;

void serialPrintln(String message) {
    Serial.println(message);
    telnet.println(message);
}

void serialPrint(String message) {
    Serial.print(message);
    telnet.print(message);
}

void onTelnetInput(String str) {
    str.trim();
  
    serialPrintln("Received command: " + str);

    if (str == "activate") {
        systemActive = true;
        serialPrintln("Security system activated");
    } else if (str == "deactivate") {
        systemActive = false;
        serialPrintln("Security system deactivated");
    } else if (str == "status") {
        serialPrintln("System status: " + String(systemActive ? "Active" : "Inactive"));
        serialPrintln("Wifi strength: " + String(WiFi.RSSI()) + " dBm");
    } else if (str == "time") {
        serialPrintln("Current time: " + getCurrentTime());
    } else {
        serialPrintln("Unknown command");
    }
}

void setupTelnet() {
    telnet.onConnect([](String ip) {
        serialPrintln("Telnet: " + ip + " connected");
        telnet.println("Welcome to ESP32 Serial Monitor");
    });
    telnet.onConnectionAttempt([](String ip) {
        serialPrintln("Telnet: " + ip + " tried to connect");
    });
    telnet.onReconnect([](String ip) {
        serialPrintln("Telnet: " + ip + " reconnected");
    });
    telnet.onDisconnect([](String ip) {
        serialPrintln("Telnet: " + ip + " disconnected");
    });
    telnet.onInputReceived(onTelnetInput);

    serialPrint("Telnet: ");
    if (telnet.begin(telnetPort)) {
        serialPrintln("Running on port " + String(telnetPort));
    } else {
        serialPrintln("Error starting Telnet server");
    }
}

String getCurrentTime() {
    struct tm timeinfo;
    char timeString[30];
    if(!getLocalTime(&timeinfo)){
        return "Time unavailable";
    } else {
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
        return String(timeString);
    }
}

void otaLoop(void * parameter) {
    for(;;) {
        ArduinoOTA.handle();
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to prevent watchdog issues
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(pirPin, INPUT);
    pinMode(ledPin, OUTPUT);
  
    serialPrintln("PIR Motion Sensor initializing...");
    delay(2000); 
    serialPrintln("PIR Motion Sensor ready!");
  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        serialPrintln("Connecting to WiFi...");
    }
    serialPrintln("Connected to WiFi");
    serialPrintln("IP address: " + WiFi.localIP().toString());

    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    ArduinoOTA.setHostname("ESP32-SecuritySystem");
    ArduinoOTA.setPassword("admin");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else
            type = "filesystem";
        serialPrintln("Start updating " + type);
    });
  
    ArduinoOTA.onEnd([]() {
        serialPrintln("\nEnd");
    });
  
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
  
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) serialPrintln("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) serialPrintln("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) serialPrintln("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) serialPrintln("Receive Failed");
        else if (error == OTA_END_ERROR) serialPrintln("End Failed");
    });

    ArduinoOTA.begin();

    fauxmo.createServer(true);
    fauxmo.setPort(80);
    fauxmo.enable(true);

    fauxmo.addDevice("Door Security");

    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
        serialPrintln(String("Device #") + device_id + " (" + device_name + ") state: " + (state ? "ON" : "OFF") + " value: " + value);
        systemActive = state;
        if (systemActive) {
            serialPrintln("Security system activated");
        } else {
            serialPrintln("Security system deactivated");
        }
    });

    setupTelnet();

    xTaskCreatePinnedToCore(
        otaLoop,    /* Task function. */
        "OTA",      /* name of task. */
        10000,      /* Stack size of task */
        NULL,       /* parameter of the task */
        1,          /* priority of the task */
        &otaTask,   /* Task handle to keep track of created task */
        0);         /* pin task to core 0 */

    // Configure secured client for Telegram
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
}

void checkMotion() {
    motionDetected = digitalRead(pirPin);
    
    if (motionDetected == HIGH) {
        unsigned long currentTime = millis();
        if (currentTime - lastMotionDetectedTime > motionCooldownPeriod) {
            lastMotionDetectedTime = currentTime;
            String currentTimeString = getCurrentTime();
            serialPrintln("Motion detected! Human presence.");
            serialPrintln("At time: " + currentTimeString);

            String notificationMessage = "Motion detected! Time: " + currentTimeString;
            sendTelegramMessage(notificationMessage);
        
            blinkCount = 8;  // 4 on-off cycles
        }
    }
}

void loop() {
    unsigned long currentTime = millis();
    fauxmo.handle();
    telnet.loop();
    
    if (systemActive) {
        checkMotion();
        
        if (blinkCount > 0) {
            if (currentTime - lastBlinkTime >= 250) {
                lastBlinkTime = currentTime;
                ledState = !ledState;
                digitalWrite(ledPin, ledState);
                
                if (ledState == LOW) {
                    blinkCount--;
                }
            }
        }
    }
    
    esp_task_wdt_reset();
}

void sendTelegramMessage(const String& message) {
    if (bot.sendMessage(CHAT_ID, message, "")) {
        serialPrintln("Telegram message sent successfully");
    } else {
        serialPrintln("Failed to send Telegram message");
    }
}
