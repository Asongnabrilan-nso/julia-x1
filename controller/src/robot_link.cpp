#include "robot_link.h"

#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <WiFi.h>

#include "config.h"

namespace robot {
namespace {

SocketIOclient io;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Shared between the link task and the main loop: guarded by `mux`.
Telemetry tel;
float cmdX = 0, cmdY = 0;
volatile bool sending = false;

// Link-task private.
uint32_t lastBalanceMs = 0;
uint32_t lastSendMs = 0;
bool wasActive = false;

bool startsWith(const uint8_t* p, size_t len, const char* prefix) {
  size_t n = strlen(prefix);
  return len >= n && memcmp(p, prefix, n) == 0;
}

void onTelemetry(uint8_t* payload, size_t len) {
  // Only parse the events we use; "params" and "imu" are large and frequent.
  if (!(startsWith(payload, len, "[\"balance\"") || startsWith(payload, len, "[\"tof\"") ||
        startsWith(payload, len, "[\"action\"") || startsWith(payload, len, "[\"calibration_status\"") ||
        startsWith(payload, len, "[\"speed_scale\""))) {
    return;
  }
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* name = doc[0] | "";
  JsonObject d = doc[1];

  portENTER_CRITICAL(&mux);
  if (!strcmp(name, "balance")) {
    tel.mode = d["mode"] | -1;
    tel.state = d["state"] | -1;
    tel.fault = d["fault"] | 0;
    lastBalanceMs = millis();
  } else if (!strcmp(name, "tof")) {
    tel.tofMm = d["mm"] | -1;
    tel.tofStatus = d["status"] | -1;
    tel.tofFactor = d["factor"] | 100;
  } else if (!strcmp(name, "action")) {
    strlcpy(tel.action, d["text"] | "", sizeof(tel.action));
  } else if (!strcmp(name, "calibration_status")) {
    strlcpy(tel.calibration, d["status"] | "", sizeof(tel.calibration));
  } else if (!strcmp(name, "speed_scale")) {
    tel.speedScale = d["scale"] | 1.0f;
  }
  portEXIT_CRITICAL(&mux);
}

void onEvent(socketIOmessageType_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case sIOtype_CONNECT:
      // WebSocket is up; join the default namespace (Socket.IO v4 handshake).
      io.send(sIOtype_CONNECT, "/");
      portENTER_CRITICAL(&mux);
      tel.socket = true;
      portEXIT_CRITICAL(&mux);
      Serial.printf("[ROBOT] connected to %s:%u\n", ROBOT_HOST, ROBOT_PORT);
      break;
    case sIOtype_DISCONNECT: {
      bool was = tel.socket;
      portENTER_CRITICAL(&mux);
      tel = Telemetry{};  // forget stale robot state
      portEXIT_CRITICAL(&mux);
      wasActive = false;
      sending = false;
      if (was) Serial.println("[ROBOT] disconnected");
      break;
    }
    case sIOtype_EVENT:
      onTelemetry(payload, len);
      break;
    default:
      break;
  }
}

void sendCommands() {
  portENTER_CRITICAL(&mux);
  float x = cmdX, y = cmdY;
  bool up = tel.socket;
  portEXIT_CRITICAL(&mux);

  if (!up) {
    wasActive = false;
    sending = false;
    return;
  }
  bool active = x != 0.0f || y != 0.0f;
  uint32_t now = millis();
  if (active && now - lastSendMs >= LINK_SEND_PERIOD_MS) {
    // Same message as the web joystick (assets/app.js), handled by on_joystick().
    char buf[64];
    snprintf(buf, sizeof(buf), "[\"joystick\",{\"x\":%.3f,\"y\":%.3f}]", x, y);
    io.sendEVENT(buf);
    lastSendMs = now;
  } else if (!active && wasActive) {
    io.sendEVENT("[\"stop\",{}]");  // same as releasing the web joystick
  }
  wasActive = active;
  sending = active;
}

void linkTask(void*) {
  bool begun = false;
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!begun) {
        io.onEvent(onEvent);
        io.begin(ROBOT_HOST, ROBOT_PORT, "/socket.io/?EIO=4");
        io.setReconnectInterval(LINK_RECONNECT_MS);
        begun = true;
        Serial.printf("[ROBOT] connecting to %s:%u ...\n", ROBOT_HOST, ROBOT_PORT);
      }
      io.loop();
      sendCommands();
    }
    bool fresh = lastBalanceMs && millis() - lastBalanceMs < LINK_STALE_MS;
    portENTER_CRITICAL(&mux);
    tel.fresh = tel.socket && fresh;
    portEXIT_CRITICAL(&mux);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace

bool configured() { return strlen(ROBOT_HOST) > 0; }

void begin() {
  if (!configured()) {
    Serial.println("[ROBOT] robot_host is empty in platformio.ini - robot link disabled");
    return;
  }
  xTaskCreate(linkTask, "robotLink", 8192, nullptr, 1, nullptr);
}

void setStick(float x, float y) {
  // Same radial dead zone as the robot (DEADZONE in python/main.py), so we don't
  // send commands the robot would zero anyway.
  if (sqrtf(x * x + y * y) < ROBOT_DEADZONE) x = y = 0.0f;
  portENTER_CRITICAL(&mux);
  cmdX = constrain(x, -1.0f, 1.0f);
  cmdY = constrain(y, -1.0f, 1.0f);
  portEXIT_CRITICAL(&mux);
}

Telemetry snapshot() {
  portENTER_CRITICAL(&mux);
  Telemetry t = tel;
  portEXIT_CRITICAL(&mux);
  return t;
}

bool connected() {
  portENTER_CRITICAL(&mux);
  bool up = tel.socket;
  portEXIT_CRITICAL(&mux);
  return up;
}

bool commanding() { return sending; }

void printStatus(Print& out) {
  if (!configured()) {
    out.println("[ROBOT] disabled (robot_host empty)");
    return;
  }
  Telemetry t = snapshot();
  out.printf("[ROBOT] %s:%u %s | mode %d state %d | ToF %d mm (st %d, fwd %u%%) | %s\n", ROBOT_HOST,
             ROBOT_PORT, !t.socket ? "NOT connected" : t.fresh ? "connected" : "connected, no telemetry",
             t.mode, t.state, t.tofMm, t.tofStatus, t.tofFactor, t.action);
}

}  // namespace robot
