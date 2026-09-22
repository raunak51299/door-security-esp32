#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
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
int lastMotionState = LOW;
bool systemActive = true;

unsigned long lastBlinkTime = 0;
unsigned long lastMotionDetectedTime = 0;
unsigned long lastWiFiReconnectAttempt = 0;
int ledState = LOW;
int blinkCount = 0;
const unsigned long motionCooldownPeriod = 30000;
const unsigned long wifiConnectTimeout = 20000;
const unsigned long wifiReconnectInterval = 10000;
const unsigned long telegramRetryInterval = 5000;
const int maxTelegramSendRetries = 3;
const int WDT_TIMEOUT = 30;
bool lastWiFiConnected = false;

const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

#define BOT_TOKEN ""
#define CHAT_ID ""

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

fauxmoESP fauxmo;

ESPTelnet telnet;
uint16_t telnetPort = 23;

struct TelegramNotification {
    char text[256];
};

QueueHandle_t telegramQueue = NULL;

const long gmtOffset_sec = 19800;  // IST is UTC+5:30
const int daylightOffset_sec = 0;

TaskHandle_t otaTask;
TaskHandle_t telegramTask;

bool hasWiFiCredentials() {
    return WIFI_SSID[0] != '\0' && WIFI_PASSWORD[0] != '\0';
}

bool isTelegramConfigured() {
    return BOT_TOKEN[0] != '\0' && CHAT_ID[0] != '\0';
}

void serialPrintln(String message) {
    Serial.println(message);
    telnet.println(message);
}

void serialPrint(String message) {
    Serial.print(message);
    telnet.print(message);
}

void clearAlertState() {
    blinkCount = 0;
    ledState = LOW;
    digitalWrite(ledPin, ledState);
}

void setSystemActive(bool active) {
    systemActive = active;
    lastMotionState = digitalRead(pirPin);

    if (systemActive) {
        serialPrintln("Security system activated");
    } else {
        clearAlertState();
        serialPrintln("Security system deactivated");
    }
}

void syncTimeWithNtp() {
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
}

bool connectToWiFi(unsigned long timeoutMs) {
    if (!hasWiFiCredentials()) {
        serialPrintln("WiFi credentials missing, continuing in offline mode");
        return false;
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeoutMs) {
        delay(1000);
        serialPrintln("Connecting to WiFi...");
    }

    if (WiFi.status() == WL_CONNECTED) {
        serialPrintln("Connected to WiFi");
        serialPrintln("IP address: " + WiFi.localIP().toString());
        syncTimeWithNtp();
        lastWiFiConnected = true;
        return true;
    }

    serialPrintln("WiFi connection timed out, continuing in offline mode");
    lastWiFiConnected = false;
    lastWiFiReconnectAttempt = millis();
    return false;
}

void ensureWiFiConnection() {
    if (!hasWiFiCredentials()) {
        return;
    }

    bool wifiConnected = WiFi.status() == WL_CONNECTED;

    if (wifiConnected) {
        if (!lastWiFiConnected) {
            serialPrintln("WiFi reconnected");
            serialPrintln("IP address: " + WiFi.localIP().toString());
            syncTimeWithNtp();
        }

        lastWiFiConnected = true;
        return;
    }

    if (lastWiFiConnected) {
        serialPrintln("WiFi connection lost");
        lastWiFiConnected = false;
    }

    unsigned long currentTime = millis();
    if (currentTime - lastWiFiReconnectAttempt >= wifiReconnectInterval) {
        serialPrintln("Attempting WiFi reconnection...");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastWiFiReconnectAttempt = currentTime;
    }
}

bool queueTelegramNotification(const String& message) {
    if (telegramQueue == NULL || !isTelegramConfigured()) {
        return false;
    }

    TelegramNotification notification = {};
    if (message.length() >= sizeof(notification.text)) {
        serialPrintln("Telegram message truncated to fit queue payload");
    }
    message.substring(0, sizeof(notification.text) - 1).toCharArray(notification.text, sizeof(notification.text));

    if (xQueueSend(telegramQueue, &notification, 0) != pdPASS) {
        serialPrintln("Notification queue full, dropping Telegram message");
        return false;
    }

    return true;
}

void telegramLoop(void * parameter) {
    TelegramNotification notification;
    bool waitingForWiFi = false;

    for (;;) {
        if (xQueuePeek(telegramQueue, &notification, portMAX_DELAY) == pdPASS) {
            if (WiFi.status() != WL_CONNECTED) {
                if (!waitingForWiFi) {
                    serialPrintln("Telegram delivery paused: WiFi disconnected");
                    waitingForWiFi = true;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            waitingForWiFi = false;

            if (xQueueReceive(telegramQueue, &notification, 0) != pdPASS) {
                continue;
            }

            bool sent = false;
            int retryCount = 0;

            while (!sent && retryCount < maxTelegramSendRetries) {
                sent = bot.sendMessage(CHAT_ID, notification.text, "");
                if (sent) {
                    serialPrintln("Telegram message sent successfully");
                } else {
                    retryCount++;
                    serialPrintln("Failed to send Telegram message, retrying");
                }

                if (!sent && retryCount < maxTelegramSendRetries) {
                    vTaskDelay(pdMS_TO_TICKS(telegramRetryInterval));
                }
            }

            if (!sent) {
                serialPrintln("Dropping Telegram message after repeated failures");
            }
        }
    }
}

void onTelnetInput(String str) {
    str.trim();

    serialPrintln("Received command: " + str);

    if (str == "activate") {
        setSystemActive(true);
    } else if (str == "deactivate") {
        setSystemActive(false);
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
    lastMotionState = digitalRead(pirPin);

    serialPrintln("PIR Motion Sensor initializing...");
    delay(2000);
    serialPrintln("PIR Motion Sensor ready!");

    connectToWiFi(wifiConnectTimeout);

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
        unsigned int percent = total == 0 ? 0 : (progress * 100U) / total;
        static int lastLoggedBucket = -1;
        int currentBucket = percent / 10;

        if (currentBucket != lastLoggedBucket || percent == 100) {
            serialPrintln("Progress: " + String(percent) + "%");
            lastLoggedBucket = currentBucket;
        }
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
        setSystemActive(state);
    });

    setupTelnet();

    // Configure secured client for Telegram
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

    if (isTelegramConfigured()) {
        telegramQueue = xQueueCreate(5, sizeof(TelegramNotification));
        if (telegramQueue == NULL) {
            serialPrintln("Failed to create Telegram notification queue");
        }
    } else {
        serialPrintln("Telegram notifications disabled: missing BOT_TOKEN or CHAT_ID");
    }

    xTaskCreatePinnedToCore(
        otaLoop,    /* Task function. */
        "OTA",      /* name of task. */
        10000,      /* Stack size of task */
        NULL,       /* parameter of the task */
        1,          /* priority of the task */
        &otaTask,   /* Task handle to keep track of created task */
        0);         /* pin task to core 0 */

    if (telegramQueue != NULL) {
        xTaskCreatePinnedToCore(
            telegramLoop,
            "Telegram",
            10000,
            NULL,
            1,
            &telegramTask,
            1);
    }
}

void checkMotion() {
    motionDetected = digitalRead(pirPin);

    if (motionDetected == HIGH && lastMotionState == LOW) {
        unsigned long currentTime = millis();
        if (currentTime - lastMotionDetectedTime > motionCooldownPeriod) {
            lastMotionDetectedTime = currentTime;
            String currentTimeString = getCurrentTime();
            serialPrintln("Motion detected! Human presence.");
            serialPrintln("At time: " + currentTimeString);

            String notificationMessage = "Motion detected! Time: " + currentTimeString;
            if (isTelegramConfigured() && !queueTelegramNotification(notificationMessage)) {
                serialPrintln("Failed to queue Telegram message");
            }

            blinkCount = 8;  // 4 on-off cycles
        }
    }

    lastMotionState = motionDetected;
}

void loop() {
    unsigned long currentTime = millis();
    ensureWiFiConnection();
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