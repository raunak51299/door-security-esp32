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
#include <esp_core_dump.h>

// Pin definitions
const int pirPin = 13;
const int ledPin = 2;  // Built-in LED pin for ESP32

// System state variables
int motionDetected = 0;
bool systemActive = true;
volatile bool wifiConnected = false;

// Timing variables
unsigned long lastBlinkTime = 0;
unsigned long lastMotionDetectedTime = 0;
unsigned long lastMemoryCheck = 0;
unsigned long lastWifiCheck = 0;
int ledState = LOW;
int blinkCount = 0;

// Constants
const unsigned long motionCooldownPeriod = 30000;
const unsigned long MEMORY_CHECK_INTERVAL = 300000;  // 5 minutes
const unsigned long WIFI_CHECK_INTERVAL = 30000;     // 30 seconds
const int WDT_TIMEOUT = 30;
const int MIN_HEAP_SIZE = 20000;  // Minimum acceptable heap size in bytes
const int MAX_TELNET_CONNECTIONS = 2;

// Network credentials
const char* WIFI_SSID = "";  // Fill in your WiFi SSID
const char* WIFI_PASSWORD = "";  // Fill in your WiFi password
#define BOT_TOKEN ""  // Fill in your Telegram bot token
#define CHAT_ID ""    // Fill in your Telegram chat ID

// Global objects
WiFiClientSecure secured_client;
UniversalTelegramBot* bot = nullptr;
fauxmoESP fauxmo;
ESPTelnet telnet;
uint16_t telnetPort = 23;
int telnetConnections = 0;

// Time configuration
const long gmtOffset_sec = 19800;  // IST is UTC+5:30
const int daylightOffset_sec = 0;

// Task handles
TaskHandle_t otaTask = nullptr;
TaskHandle_t telegramTask = nullptr;

// Buffer for string formatting
char logBuffer[150];

// Function declarations
void setupTelegram();
void reconnectWiFi();
void checkHeapMemory();

// Logging functions with static buffers
void serialPrintln(const char* message) {
    Serial.println(message);
    telnet.println(message);
}

void serialPrintln(const String& message) {
    Serial.println(message);
    telnet.println(message);
}

// Time management
String getCurrentTime() {
    static char timeString[30];
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        return "Time unavailable";
    }
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeString);
}

// Telegram setup and management
void setupTelegram() {
    if (bot != nullptr) {
        delete bot;
    }
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    bot = new UniversalTelegramBot(BOT_TOKEN, secured_client);
}

void sendTelegramMessage(const String& message) {
    if (!wifiConnected) {
        snprintf(logBuffer, sizeof(logBuffer), "Cannot send Telegram message: WiFi disconnected");
        serialPrintln(logBuffer);
        return;
    }
    
    if (bot && bot->sendMessage(CHAT_ID, message, "")) {
        snprintf(logBuffer, sizeof(logBuffer), "Telegram message sent successfully");
    } else {
        snprintf(logBuffer, sizeof(logBuffer), "Failed to send Telegram message");
    }
    serialPrintln(logBuffer);
}

// WiFi management
void reconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        snprintf(logBuffer, sizeof(logBuffer), "WiFi disconnected. Reconnecting...");
        serialPrintln(logBuffer);
        
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            snprintf(logBuffer, sizeof(logBuffer), "WiFi reconnected. IP: %s", WiFi.localIP().toString().c_str());
            serialPrintln(logBuffer);
            setupTelegram();
        } else {
            serialPrintln("Failed to reconnect to WiFi");
        }
    }
}

// Memory management
void checkHeapMemory() {
    unsigned long currentTime = millis();
    if (currentTime - lastMemoryCheck > MEMORY_CHECK_INTERVAL) {
        lastMemoryCheck = currentTime;
        
        snprintf(logBuffer, sizeof(logBuffer), 
                "Memory - Free heap: %lu, Largest block: %lu",
                ESP.getFreeHeap(),
                ESP.getMaxAllocHeap());
        serialPrintln(logBuffer);
        
        // Check stack high water mark
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        snprintf(logBuffer, sizeof(logBuffer), "Stack high water mark: %lu", uxHighWaterMark);
        serialPrintln(logBuffer);
        
        // Restart if memory is too low
        if (ESP.getFreeHeap() < MIN_HEAP_SIZE) {
            serialPrintln("Critical memory level reached. Restarting...");
            ESP.restart();
        }
    }
}

// Telnet setup and management
void setupTelnet() {
    telnet.onConnect([](String ip) {
        if (telnetConnections >= MAX_TELNET_CONNECTIONS) {
            telnet.disconnect(ip);
            return;
        }
        telnetConnections++;
        snprintf(logBuffer, sizeof(logBuffer), "Telnet: %s connected", ip.c_str());
        serialPrintln(logBuffer);
        telnet.println("Welcome to ESP32 Serial Monitor");
    });

    telnet.onDisconnect([](String ip) {
        telnetConnections--;
        if (telnetConnections < 0) telnetConnections = 0;
        snprintf(logBuffer, sizeof(logBuffer), "Telnet: %s disconnected", ip.c_str());
        serialPrintln(logBuffer);
    });

    telnet.onInputReceived([](String str) {
        str.trim();
        snprintf(logBuffer, sizeof(logBuffer), "Received command: %s", str.c_str());
        serialPrintln(logBuffer);

        if (str == "activate") {
            systemActive = true;
            serialPrintln("Security system activated");
        } else if (str == "deactivate") {
            systemActive = false;
            serialPrintln("Security system deactivated");
        } else if (str == "status") {
            snprintf(logBuffer, sizeof(logBuffer), 
                    "System status: %s, Wifi strength: %d dBm", 
                    systemActive ? "Active" : "Inactive", 
                    WiFi.RSSI());
            serialPrintln(logBuffer);
        } else if (str == "memory") {
            checkHeapMemory();
        } else if (str == "time") {
            snprintf(logBuffer, sizeof(logBuffer), "Current time: %s", getCurrentTime().c_str());
            serialPrintln(logBuffer);
        } else {
            serialPrintln("Unknown command");
        }
    });

    if (telnet.begin(telnetPort)) {
        snprintf(logBuffer, sizeof(logBuffer), "Telnet server running on port %d", telnetPort);
        serialPrintln(logBuffer);
    } else {
        serialPrintln("Error starting Telnet server");
    }
}

// OTA Task
void otaLoop(void * parameter) {
    for(;;) {
        ArduinoOTA.handle();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Motion detection
void checkMotion() {
    motionDetected = digitalRead(pirPin);
    
    if (motionDetected == HIGH) {
        unsigned long currentTime = millis();
        if (currentTime - lastMotionDetectedTime > motionCooldownPeriod) {
            lastMotionDetectedTime = currentTime;
            String currentTimeString = getCurrentTime();
            
            snprintf(logBuffer, sizeof(logBuffer), 
                    "Motion detected! Time: %s", 
                    currentTimeString.c_str());
            serialPrintln(logBuffer);
            sendTelegramMessage(logBuffer);
            
            blinkCount = 8;  // 4 on-off cycles
        }
    }
}

// Setup
void setup() {
    Serial.begin(115200);
    pinMode(pirPin, INPUT);
    pinMode(ledPin, OUTPUT);
    
    serialPrintln("PIR Motion Sensor initializing...");
    delay(2000);
    serialPrintln("PIR Motion Sensor ready!");
    
    // Initialize WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        serialPrintln("Connecting to WiFi...");
    }
    wifiConnected = true;
    snprintf(logBuffer, sizeof(logBuffer), "Connected to WiFi. IP: %s", WiFi.localIP().toString().c_str());
    serialPrintln(logBuffer);

    // Initialize time
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");

    // Setup watchdog
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    // Setup OTA
    ArduinoOTA.setHostname("ESP32-SecuritySystem");
    ArduinoOTA.setPassword("admin");
    
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        snprintf(logBuffer, sizeof(logBuffer), "Start updating %s", type.c_str());
        serialPrintln(logBuffer);
    });
    
    ArduinoOTA.onEnd([]() {
        serialPrintln("\nEnd");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        snprintf(logBuffer, sizeof(logBuffer), "Progress: %u%%", (progress / (total / 100)));
        Serial.print(logBuffer);
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        if (error == OTA_AUTH_ERROR) serialPrintln("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) serialPrintln("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) serialPrintln("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) serialPrintln("Receive Failed");
        else if (error == OTA_END_ERROR) serialPrintln("End Failed");
    });
    
    ArduinoOTA.begin();

    // Setup Alexa integration
    fauxmo.createServer(true);
    fauxmo.setPort(80);
    fauxmo.enable(true);
    fauxmo.addDevice("Door Security");
    
    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
        snprintf(logBuffer, sizeof(logBuffer), 
                "Device #%d (%s) state: %s value: %d", 
                device_id, device_name, state ? "ON" : "OFF", value);
        serialPrintln(logBuffer);
        
        systemActive = state;
        serialPrintln(systemActive ? "Security system activated" : "Security system deactivated");
    });

    // Setup other services
    setupTelnet();
    setupTelegram();

    // Create OTA task
    xTaskCreatePinnedToCore(
        otaLoop,
        "OTA",
        8192,      // Increased stack size
        NULL,
        1,
        &otaTask,
        0
    );

    // Initialize core dump
    esp_core_dump_init();
    
    // Initial memory check
    checkHeapMemory();
}

// Main loop
void loop() {
    unsigned long currentTime = millis();
    
    // Check WiFi connection periodically
    if (currentTime - lastWifiCheck > WIFI_CHECK_INTERVAL) {
        lastWifiCheck = currentTime;
        reconnectWiFi();
    }
    
    fauxmo.handle();
    telnet.loop();
    
    // Check memory periodically
    checkHeapMemory();
    
    if (systemActive) {
        checkMotion();
        
        // Handle LED blinking
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
    
    // Small delay to prevent watchdog issues
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
