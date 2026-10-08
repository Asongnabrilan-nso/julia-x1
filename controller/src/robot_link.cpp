#include "robot_link.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <SocketIOclient.h>
#include <WiFi.h>

#include "config.h"

namespace robot {
namespace {

SocketIOclient io;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ---- Shared between the link task and the main loop: guarded by `mux` ----
Telemetry tel;
float cmdX = 0, cmdY = 0;
volatile bool sending = false;
enum class Request : uint8_t { None, Pair, Choose, Cancel, Forget };
Request request = Request::None;
discovery::Robot chosen;
discovery::Robot results[discovery::kMaxRobots];
uint8_t resultCount = 0;

// ---- Pairing record (flash) ----
struct Paired {
  bool valid = false;
  uint32_t ip = 0;  // last known IP (IPAddress raw, network order)
  uint8_t mac[6] = {0};
  bool hasMac = false;
};
Paired paired;

// ---- Link-task private ----
bool ioBegun = false;
IPAddress target;
uint32_t connectStartMs = 0;
uint32_t lastSearchMs = 0;
uint32_t lastBalanceMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastPingMs = 0;
bool wasActive = false;
int16_t rttWindowMax = -1;
uint32_t rttWindowStartMs = 0;

constexpr char kNvsNs[] = "robot";

void setLink(LinkState s) {
  portENTER_CRITICAL(&mux);
  tel.link = s;
  portEXIT_CRITICAL(&mux);
}

void loadPaired() {
  Preferences p;
  if (!p.begin(kNvsNs, true)) return;
  paired.valid = p.getBool("valid", false);
  paired.ip = p.getUInt("ip", 0);
  paired.hasMac = p.getBytes("mac", paired.mac, 6) == 6;
  p.end();
}

void savePaired() {
  Preferences p;
  if (!p.begin(kNvsNs, false)) return;
  p.putBool("valid", paired.valid);
  p.putUInt("ip", paired.ip);
  if (paired.hasMac) p.putBytes("mac", paired.mac, 6);
  else p.remove("mac");
  p.end();
}

bool startsWith(const uint8_t* p, size_t len, const char* prefix) {
  size_t n = strlen(prefix);
  return len >= n && memcmp(p, prefix, n) == 0;
}

void onTelemetry(uint8_t* payload, size_t len) {
  // Only parse the events we use; "params" and "imu" are large and frequent.
  if (!(startsWith(payload, len, "[\"balance\"") || startsWith(payload, len, "[\"tof\"") ||
        startsWith(payload, len, "[\"action\"") || startsWith(payload, len, "[\"calibration_status\"") ||
        startsWith(payload, len, "[\"speed_scale\"") || startsWith(payload, len, "[\"joy_pong\""))) {
    return;
  }
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* name = doc[0] | "";
  JsonObject d = doc[1];

  if (!strcmp(name, "joy_pong")) {
    // Round trip: controller -> WiFi -> robot Python -> WiFi -> controller.
    uint32_t sent = d["t"] | 0u;
    int16_t rtt = (int16_t)min<uint32_t>(millis() - sent, 9999);
    uint32_t now = millis();
    if (now - rttWindowStartMs > RTT_WINDOW_MS) {
      rttWindowStartMs = now;
      rttWindowMax = -1;
    }
    rttWindowMax = max(rttWindowMax, rtt);
    portENTER_CRITICAL(&mux);
    tel.rttMs = rtt;
    tel.rttMaxMs = rttWindowMax;
    portEXIT_CRITICAL(&mux);
    return;
  }

  int mode = d["mode"] | -1, state = d["state"] | -1, fault = d["fault"] | 0;
  int mm = d["mm"] | -1, status = d["status"] | -1, factor = d["factor"] | 100;
  float scale = d["scale"] | 1.0f;
  const char* text = d["text"] | "";
  const char* calStatus = d["status"] | "";

  portENTER_CRITICAL(&mux);
  if (!strcmp(name, "balance")) {
    tel.mode = mode;
    tel.state = state;
    tel.fault = fault;
    lastBalanceMs = millis();
  } else if (!strcmp(name, "tof")) {
    tel.tofMm = mm;
    tel.tofStatus = status;
    tel.tofFactor = factor;
  } else if (!strcmp(name, "action")) {
    strlcpy(tel.action, text, sizeof(tel.action));
  } else if (!strcmp(name, "calibration_status")) {
    strlcpy(tel.calibration, calStatus, sizeof(tel.calibration));
  } else if (!strcmp(name, "speed_scale")) {
    tel.speedScale = scale;
  }
  portEXIT_CRITICAL(&mux);
}

void resetTelemetry() {
  portENTER_CRITICAL(&mux);
  LinkState keep = tel.link;
  tel = Telemetry{};
  tel.link = keep;
  portEXIT_CRITICAL(&mux);
  wasActive = false;
  sending = false;
}

void onEvent(socketIOmessageType_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case sIOtype_CONNECT:
      // WebSocket is up; join the default namespace (Socket.IO v4 handshake).
      io.send(sIOtype_CONNECT, "/");
      portENTER_CRITICAL(&mux);
      tel.socket = true;
      tel.link = LinkState::Connected;
      portEXIT_CRITICAL(&mux);
      Serial.printf("[ROBOT] connected to %s:%u\n", target.toString().c_str(), ROBOT_PORT);
      break;
    case sIOtype_DISCONNECT:
      if (tel.socket) Serial.println("[ROBOT] disconnected");
      resetTelemetry();
      if (ioBegun) {
        // The library keeps retrying; if the robot stays away, linkTask rescans by MAC.
        setLink(LinkState::Connecting);
        connectStartMs = millis();
      }
      break;
    case sIOtype_EVENT:
      onTelemetry(payload, len);
      break;
    default:
      break;
  }
}

void stopSocket() {
  if (ioBegun) io.disconnect();
  ioBegun = false;
  resetTelemetry();
}

void connectTo(IPAddress ip) {
  stopSocket();
  target = ip;
  io.onEvent(onEvent);
  io.begin(ip.toString(), ROBOT_PORT, "/socket.io/?EIO=4");
  io.setReconnectInterval(LINK_RECONNECT_MS);
  ioBegun = true;
  connectStartMs = millis();
  setLink(LinkState::Connecting);
  Serial.printf("[ROBOT] connecting to %s:%u ...\n", ip.toString().c_str(), ROBOT_PORT);
}

// Paired robot not answering at its last IP: find it again by MAC.
void searchForPaired() {
  setLink(LinkState::Searching);
  stopSocket();
  discovery::Robot found[discovery::kMaxRobots];
  uint8_t n = discovery::scan(found, discovery::kMaxRobots);
  lastSearchMs = millis();
  for (uint8_t i = 0; i < n; i++) {
    bool match = paired.hasMac ? (found[i].hasMac && !memcmp(found[i].mac, paired.mac, 6))
                               : (uint32_t)found[i].ip == paired.ip;
    if (match) {
      if ((uint32_t)found[i].ip != paired.ip) {
        Serial.printf("[ROBOT] paired robot moved to %s - updating\n",
                      found[i].ip.toString().c_str());
        paired.ip = (uint32_t)found[i].ip;
        savePaired();
      }
      connectTo(found[i].ip);
      return;
    }
  }
  Serial.println("[ROBOT] paired robot not found - retrying at its last IP");
  connectTo(IPAddress(paired.ip));
}

void runPairScan() {
  stopSocket();
  setLink(LinkState::PairScanning);
  discovery::Robot found[discovery::kMaxRobots];
  uint8_t n = discovery::scan(found, discovery::kMaxRobots);
  portENTER_CRITICAL(&mux);
  memcpy(results, found, sizeof(found));
  resultCount = n;
  tel.link = LinkState::PairChoose;
  portEXIT_CRITICAL(&mux);
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
  uint32_t now = millis();
  bool active = x != 0.0f || y != 0.0f;
  char buf[64];
  if (active && now - lastSendMs >= LINK_SEND_PERIOD_MS) {
    // Same message as the web joystick (assets/app.js), handled by on_joystick().
    snprintf(buf, sizeof(buf), "[\"joystick\",{\"x\":%.3f,\"y\":%.3f}]", x, y);
    io.sendEVENT(buf);
    lastSendMs = now;
  } else if (!active && wasActive) {
    io.sendEVENT("[\"stop\",{}]");  // same as releasing the web joystick
  }
  wasActive = active;
  sending = active;

  if (now - lastPingMs >= RTT_PING_MS) {
    snprintf(buf, sizeof(buf), "[\"joy_ping\",{\"t\":%lu}]", (unsigned long)now);
    io.sendEVENT(buf);
    lastPingMs = now;
  }
}

void handleRequest() {
  portENTER_CRITICAL(&mux);
  Request r = request;
  request = Request::None;
  discovery::Robot pick = chosen;
  portEXIT_CRITICAL(&mux);

  switch (r) {
    case Request::Pair:
      runPairScan();
      break;
    case Request::Choose:
      paired.valid = true;
      paired.ip = (uint32_t)pick.ip;
      paired.hasMac = pick.hasMac;
      memcpy(paired.mac, pick.mac, 6);
      savePaired();
      Serial.printf("[ROBOT] paired with %s (mac %s)\n", pick.ip.toString().c_str(),
                    pick.hasMac ? discovery::macToString(pick.mac).c_str() : "?");
      connectTo(pick.ip);
      break;
    case Request::Cancel:
      if (paired.valid) connectTo(IPAddress(paired.ip));
      else setLink(LinkState::Unpaired);
      break;
    case Request::Forget:
      stopSocket();
      paired = Paired{};
      savePaired();
      setLink(LinkState::Unpaired);
      Serial.println("[ROBOT] pairing erased");
      break;
    default:
      break;
  }
}

void linkTask(void*) {
  bool fixedHost = strlen(ROBOT_HOST) > 0;
  bool hadWifi = false;
  for (;;) {
    bool wifi = WiFi.status() == WL_CONNECTED;
    if (!wifi) {
      if (hadWifi) stopSocket();
      setLink(LinkState::NoWifi);
    } else {
      if (fixedHost && !ioBegun && (!hadWifi || millis() - lastSearchMs > LINK_SEARCH_EVERY_MS)) {
        lastSearchMs = millis();
        IPAddress ip;
        if (WiFi.hostByName(ROBOT_HOST, ip)) connectTo(ip);
        else Serial.printf("[ROBOT] can't resolve robot_host %s, retrying\n", ROBOT_HOST);
      } else if (!hadWifi && !fixedHost) {
        // WiFi (re)connected: start towards the paired robot, or pair.
        if (paired.valid) {
          connectTo(IPAddress(paired.ip));
        } else {
          Serial.println("[ROBOT] not paired - starting pairing scan");
          runPairScan();
        }
      }
      handleRequest();

      LinkState s = snapshot().link;
      if (ioBegun) {
        io.loop();
        sendCommands();
      }
      // Paired robot silent at its last IP for a while: look for it by MAC.
      if (!fixedHost && paired.valid && s == LinkState::Connecting &&
          millis() - connectStartMs > LINK_SEARCH_AFTER_MS &&
          millis() - lastSearchMs > LINK_SEARCH_EVERY_MS) {
        searchForPaired();
      }
    }
    hadWifi = wifi;

    bool fresh = lastBalanceMs && millis() - lastBalanceMs < LINK_STALE_MS;
    portENTER_CRITICAL(&mux);
    tel.fresh = tel.socket && fresh;
    portEXIT_CRITICAL(&mux);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void post(Request r) {
  portENTER_CRITICAL(&mux);
  request = r;
  portEXIT_CRITICAL(&mux);
}

}  // namespace

void begin() {
  loadPaired();
  if (strlen(ROBOT_HOST) > 0) {
    Serial.printf("[ROBOT] fixed robot_host %s (pairing disabled)\n", ROBOT_HOST);
  } else if (paired.valid) {
    Serial.printf("[ROBOT] paired robot: %s mac %s\n", IPAddress(paired.ip).toString().c_str(),
                  paired.hasMac ? discovery::macToString(paired.mac).c_str() : "?");
  } else {
    Serial.println("[ROBOT] no robot paired yet");
  }
  // Higher priority than loop() so display redraws never delay commands.
  xTaskCreate(linkTask, "robotLink", 10240, nullptr, 2, nullptr);
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

bool connected() { return snapshot().socket; }

bool commanding() { return sending; }

bool isPaired() { return paired.valid || strlen(ROBOT_HOST) > 0; }

IPAddress pairedIp() { return IPAddress(paired.ip); }

void startPairing() { post(Request::Pair); }

uint8_t pairResults(discovery::Robot* out, uint8_t maxOut) {
  portENTER_CRITICAL(&mux);
  uint8_t n = min(resultCount, maxOut);
  memcpy(out, results, n * sizeof(discovery::Robot));
  portEXIT_CRITICAL(&mux);
  return n;
}

void choosePaired(const discovery::Robot& r) {
  portENTER_CRITICAL(&mux);
  chosen = r;
  request = Request::Choose;
  portEXIT_CRITICAL(&mux);
}

void cancelPairing() { post(Request::Cancel); }

void forget() { post(Request::Forget); }

void printStatus(Print& out) {
  static const char* kLink[] = {"no WiFi", "NOT PAIRED", "connecting", "searching by MAC",
                                "connected", "pair scan", "pair choose"};
  Telemetry t = snapshot();
  out.printf("[ROBOT] %s %s | mode %d state %d | ToF %d mm (st %d, fwd %u%%) | rtt %d ms (max %d) | %s\n",
             strlen(ROBOT_HOST) ? ROBOT_HOST : IPAddress(paired.ip).toString().c_str(),
             kLink[(int)t.link], t.mode, t.state, t.tofMm, t.tofStatus, t.tofFactor, t.rttMs,
             t.rttMaxMs, t.action);
}

}  // namespace robot
