/*
  Wi-Fi connection test for the foyer sensor board.
  Uses the same Wi-Fi setup as src/net.cpp in the main project, with no
  sensors, buzzer, or ntfy. The on-board LED shows the link state:
    rapid blink = connecting (first CONNECTING_SHOWN_MS after each attempt)
    steady on   = connected
    steady off  = not connected and waiting for the next attempt
  The serial port (115200 baud) prints the same status in more detail.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>

#include "secrets.h"

// On-board LED. GPIO2 on most ESP32 dev boards. Set LED_ACTIVE_LOW to
// true if your board's LED lights when the pin is LOW.
const int LED_PIN = 2;
const bool LED_ACTIVE_LOW = false;

// Same retry period as the main project.
const unsigned long RETRY_MS = 30000;

// How long after each WiFi.begin() the LED blinks as "connecting".
const unsigned long CONNECTING_SHOWN_MS = 10000;

// Blink half-period, in milliseconds.
const unsigned long BLINK_MS = 100;

// Serial status line interval while not connected.
const unsigned long LOG_MS = 5000;

unsigned long lastAttempt = 0;
bool wasConnected = false;

void setLed(bool on) {
  digitalWrite(LED_PIN, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

const char *resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software reset";
    case ESP_RST_PANIC:    return "crash";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_EXT:      return "external reset";
    default:               return "other";
  }
}

const char *statusText(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "idle";
    case WL_NO_SSID_AVAIL:   return "no SSID";
    case WL_SCAN_COMPLETED:  return "scan done";
    case WL_CONNECTED:       return "connected";
    case WL_CONNECT_FAILED:  return "connect failed";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED:    return "disconnected";
    default:                 return "?";
  }
}

const char *reasonText(int r) {
  switch (r) {
    case 2:   return "auth expired";
    case 7:   return "not authed";
    case 8:   return "left (assoc leave)";
    case 15:  return "4-way handshake timeout";
    case 200: return "beacon timeout";
    case 201: return "AP not found";
    case 202: return "auth failed";
    case 203: return "assoc failed";
    case 204: return "handshake timeout";
    default:  return "";
  }
}

void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    int r = info.wifi_sta_disconnected.reason;
    Serial.printf("[%lu ms] disconnect reason: %d %s\n", millis(), r,
                  reasonText(r));
  } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    Serial.printf("[%lu ms] associated with AP\n", millis());
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.printf("[%lu ms] got IP\n", millis());
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  Serial.println();
  Serial.printf("Wi-Fi test starting. Reset reason: %s\n", resetReasonText());

  // Same calls and order as netBegin() in the main project.
  WiFi.onEvent(onWifiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastAttempt = millis();
}

void loop() {
  unsigned long now = millis();
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected) {
    Serial.printf("[%lu ms] CONNECTED. IP %s, signal %d dBm\n", now,
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else if (!connected && wasConnected) {
    Serial.printf("[%lu ms] Wi-Fi lost\n", now);
  }
  wasConnected = connected;

  // Same retry rule as the main project.
  if (!connected && now - lastAttempt >= RETRY_MS) {
    lastAttempt = now;
    Serial.printf("[%lu ms] retry: WiFi.begin()\n", now);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  static unsigned long lastLog = 0;
  if (!connected && now - lastLog >= LOG_MS) {
    lastLog = now;
    wl_status_t s = WiFi.status();
    Serial.printf("[%lu ms] not connected. status %d (%s)\n", now, (int)s,
                  statusText(s));
  }

  if (connected) {
    setLed(true);
  } else if (now - lastAttempt < CONNECTING_SHOWN_MS) {
    setLed((now / BLINK_MS) % 2 == 0);
  } else {
    setLed(false);
  }
}
