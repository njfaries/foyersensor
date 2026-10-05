#include "net.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>

#include "secrets.h"

static const unsigned long WIFI_RETRY_MS = 10000;
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

void netBegin() {
  queue = xQueueCreate(QUEUE_LENGTH, sizeof(Message));
  xTaskCreate(senderTask, "ntfy", 8192, nullptr, 1, nullptr);

  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
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
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected) {
    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
    netNotify("Foyer sensor online", "Cat detector is connected.", 2,
              "white_check_mark", false);
  } else if (!connected && wasConnected) {
    Serial.println("Wi-Fi lost.");
  }
  wasConnected = connected;

  if (!connected && millis() - lastWifiAttempt >= WIFI_RETRY_MS) {
    lastWifiAttempt = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  server.handleClient();
}
