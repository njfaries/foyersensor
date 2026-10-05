#include "net.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>

#include "secrets.h"

// Long enough for a slow connect to finish before a new attempt.
static const unsigned long WIFI_RETRY_MS = 30000;
static const unsigned long WIFI_LOG_MS = 5000;

// A cold start can fail to join Wi-Fi, and a second start joins. If the
// first connect has not worked by this time, restart. Stop after a few
// tries so a router that is off does not cause endless restarts.
static const unsigned long FIRST_CONNECT_TIMEOUT_MS = 30000;
static const uint8_t MAX_BOOT_RESTARTS = 3;

// Survives a software restart, so the restarts can be counted.
static const uint32_t RESTART_MAGIC = 0xF0E1D2C3;
static RTC_NOINIT_ATTR uint32_t restartMagic;
static RTC_NOINIT_ATTR uint8_t restartCount;
static const int QUEUE_LENGTH = 8;
static const uint16_t HTTP_TIMEOUT_MS = 3000;

struct Message {
  char title[48];
  char body[160];
  char tags[48];
  int priority;
  bool withResetAction;
};

static WebServer server(80);
static QueueHandle_t queue;
static volatile bool resetFlag = false;
static unsigned long lastWifiAttempt = 0;

static const char *resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_SW:        return "software reset";
    case ESP_RST_PANIC:     return "crash";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_EXT:       return "external reset";
    default:                return "other";
  }
}

// Shared with the main loop. Guarded by a short critical section.
static portMUX_TYPE statusLock = portMUX_INITIALIZER_UNLOCKED;
static char statusState[24] = "?";
static bool statusAlarm = false;

void netSetStatus(const char *stateText, bool alarmOn) {
  portENTER_CRITICAL(&statusLock);
  strlcpy(statusState, stateText, sizeof(statusState));
  statusAlarm = alarmOn;
  portEXIT_CRITICAL(&statusLock);
}

bool netResetRequested() {
  bool r = resetFlag;
  resetFlag = false;
  return r;
}

void netNotify(const char *title, const char *message, int priority,
               const char *tags, bool withResetAction) {
  Message m;
  strlcpy(m.title, title, sizeof(m.title));
  strlcpy(m.body, message, sizeof(m.body));
  strlcpy(m.tags, tags, sizeof(m.tags));
  m.priority = priority;
  m.withResetAction = withResetAction;
  if (xQueueSend(queue, &m, 0) != pdTRUE) {
    Serial.println("ntfy queue full. Message dropped.");
  }
}

// Sends one message to ntfy. Runs in the sender task only.
static void sendMessage(const Message &m) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ntfy: no Wi-Fi. Message dropped.");
    return;
  }

  String url = String(NTFY_URL) + "/" + NTFY_TOPIC;
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.begin(url);
  http.addHeader("Title", m.title);
  http.addHeader("Priority", String(m.priority));
  http.addHeader("Tags", m.tags);
  if (strlen(NTFY_TOKEN) > 0) {
    http.addHeader("Authorization", String("Bearer ") + NTFY_TOKEN);
  }
  if (m.withResetAction) {
    String action = String("http, Reset alarm, ") + DEVICE_URL +
                    "/reset, method=POST, clear=true";
    if (strlen(RESET_TOKEN) > 0) {
      action += String(", headers.X-Token=") + RESET_TOKEN;
    }
    http.addHeader("Actions", action);
  }

  int code = http.POST(m.body);
  Serial.printf("ntfy: HTTP %d\n", code);
  http.end();
}

static void senderTask(void *) {
  Message m;
  for (;;) {
    if (xQueueReceive(queue, &m, portMAX_DELAY) == pdTRUE) {
      sendMessage(m);
    }
  }
}

static bool authorized() {
  if (strlen(RESET_TOKEN) == 0)
    return true;
  return server.header("X-Token") == RESET_TOKEN;
}

static void handleReset() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Use POST");
    return;
  }
  if (!authorized()) {
    server.send(401, "text/plain", "Bad token");
    return;
  }
  resetFlag = true;
  server.send(200, "text/plain", "Reset requested");
}

static void handleStatus() {
  char stateCopy[sizeof(statusState)];
  bool alarmCopy;
  portENTER_CRITICAL(&statusLock);
  strlcpy(stateCopy, statusState, sizeof(stateCopy));
  alarmCopy = statusAlarm;
  portEXIT_CRITICAL(&statusLock);

  char json[96];
  snprintf(json, sizeof(json),
           "{\"state\":\"%s\",\"alarm\":%s,\"uptime_s\":%lu}", stateCopy,
           alarmCopy ? "true" : "false", millis() / 1000UL);
  server.send(200, "application/json", json);
}

// Logs why the Wi-Fi link dropped or could not start.
// Reason codes: 2 auth expired, 7 not authed, 15 or 204 handshake timeout,
// 201 AP not found, 202 auth failed.
static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf("Wi-Fi disconnect reason: %d\n",
                  info.wifi_sta_disconnected.reason);
  }
}

void netBegin() {
  queue = xQueueCreate(QUEUE_LENGTH, sizeof(Message));
  xTaskCreate(senderTask, "ntfy", 8192, nullptr, 1, nullptr);

  WiFi.onEvent(onWifiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
  Serial.printf("Reset reason: %s\n", resetReasonText());

  // Keep the count only across our own restarts.
  if (esp_reset_reason() != ESP_RST_SW || restartMagic != RESTART_MAGIC) {
    restartMagic = RESTART_MAGIC;
    restartCount = 0;
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttempt = millis();

  const char *headers[] = {"X-Token"};
  server.collectHeaders(headers, 1);
  server.on("/reset", handleReset);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
}

void netLoop() {
  static bool wasConnected = false;
  static bool everConnected = false;
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected) {
    everConnected = true;
    restartCount = 0;
    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
    char msg[96];
    snprintf(msg, sizeof(msg), "Connected. Reset: %s. Signal: %d dBm.",
             resetReasonText(), WiFi.RSSI());
    Serial.println(msg);
    netNotify("Foyer sensor online", msg, 2, "white_check_mark", false);
  } else if (!connected && wasConnected) {
    Serial.println("Wi-Fi lost.");
  }
  wasConnected = connected;

  static unsigned long lastLog = 0;
  if (!connected && millis() - lastLog >= WIFI_LOG_MS) {
    lastLog = millis();
    Serial.printf("Wi-Fi not connected. Status %d\n", (int)WiFi.status());
  }

  if (!connected && millis() - lastWifiAttempt >= WIFI_RETRY_MS) {
    lastWifiAttempt = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  // Restart only before the first connect, and never during an alarm.
  if (!everConnected && millis() >= FIRST_CONNECT_TIMEOUT_MS &&
      restartCount < MAX_BOOT_RESTARTS) {
    bool alarmNow;
    portENTER_CRITICAL(&statusLock);
    alarmNow = statusAlarm;
    portEXIT_CRITICAL(&statusLock);
    if (!alarmNow) {
      restartCount++;
      Serial.printf("No Wi-Fi after %lu s. Restart %u of %u.\n",
                    FIRST_CONNECT_TIMEOUT_MS / 1000, restartCount,
                    MAX_BOOT_RESTARTS);
      delay(100);
      ESP.restart();
    }
  }

  server.handleClient();
}
