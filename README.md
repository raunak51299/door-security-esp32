# ESP32 Security System
This project implements a versatile IoT security system using an ESP32 microcontroller. It features motion detection, remote control via Alexa and Telnet, and real-time notifications.
Features

# Motion detection using a PIR sensor
Pushover notifications when motion is detected
Alexa integration for voice control
Telnet server for remote monitoring and control
OTA (Over-The-Air) updates
NTP time synchronization
Watchdog timer for system stability

# Key Components

ESP32 microcontroller
PIR motion sensor
LED for visual alerts

# Functionality

Activates/deactivates the security system via Alexa or Telnet commands
Sends push notifications with timestamps when motion is detected
Blinks the onboard LED when motion is detected
Provides system status and current time via Telnet
Supports OTA updates for easy firmware upgrades

# Libraries Used

WiFi
HTTPClient
fauxmoESP (for Alexa integration)
ArduinoOTA
ESPTelnet
FreeRTOS
