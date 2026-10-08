#include "ota.h"

#include <ArduinoOTA.h>
#include <WiFi.h>

#include "display.h"

namespace ota {
namespace {

bool otaStarted = false;
bool wasConnected = false;
volatile bool updating = false;
int lastPct = -1;

void setupArduinoOta() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    updating = true;
    display::showMessage("OTA update...", "do not power off");
    lastPct = -1;
    Serial.printf("[OTA] start (%s)\n",
                  ArduinoOTA.getCommand() == U_FLASH ? "firmware" : "filesystem");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    int pct = total ? (done * 100) / total : 0;
    if (pct / 10 != lastPct / 10) {
      Serial.printf("[OTA] %d%%\n", pct);
      lastPct = pct;
    }
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] done, rebooting");
    updating = false;
  });
  ArduinoOTA.onError([](ota_error_t err) {
    const char* why = err == OTA_AUTH_ERROR      ? "auth failed (check ota_password)"
                    : err == OTA_BEGIN_ERROR     ? "begin failed (no space?)"
                    : err == OTA_CONNECT_ERROR   ? "connect failed (PC firewall?)"
                    : err == OTA_RECEIVE_ERROR   ? "receive failed"
                    : err == OTA_END_ERROR       ? "end failed"
                                                 : "unknown";
    Serial.printf("[OTA] error %u: %s\n", err, why);
    display::showMessage("OTA failed", why);
    updating = false;
  });
}

}  // namespace

void begin() {
  if (strlen(WIFI_SSID) == 0) {
    Serial.println("[WiFi] no SSID configured (secrets.ini) - OTA disabled");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);  // lower, steadier latency for control packets later
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  setupArduinoOta();
  Serial.printf("[WiFi] connecting to \"%s\"...\n", WIFI_SSID);
}

void handle() {
  bool connected = WiFi.status() == WL_CONNECTED;

  if (connected && !wasConnected) {
    Serial.printf("[WiFi] connected  ip=%s  rssi=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    // (Re)start OTA so mDNS and the UDP listener bind to the new link.
    if (otaStarted) ArduinoOTA.end();
    ArduinoOTA.begin();
    otaStarted = true;
    Serial.printf("[OTA] ready: %s.local:3232\n", OTA_HOSTNAME);
  } else if (!connected && wasConnected) {
    Serial.println("[WiFi] disconnected, retrying...");
  }
  wasConnected = connected;

  if (otaStarted && connected) ArduinoOTA.handle();
}

bool wifiConnected() { return WiFi.status() == WL_CONNECTED; }

bool inProgress() { return updating; }

void printStatus(Print& out) {
  out.printf("fw %s  host %s\n", FW_VERSION, OTA_HOSTNAME);
  if (wifiConnected()) {
    out.printf("WiFi \"%s\"  ip %s  rssi %d dBm  OTA %s\n", WIFI_SSID,
               WiFi.localIP().toString().c_str(), WiFi.RSSI(),
               otaStarted ? "ready" : "off");
  } else {
    out.printf("WiFi \"%s\" not connected (status %d)\n", WIFI_SSID, WiFi.status());
  }
}

void printStatusLine(Print& out) {
  if (wifiConnected()) {
    out.printf("[STATUS] fw %s | WiFi \"%s\" ip %s %d dBm | OTA %s.local %s\n", FW_VERSION,
               WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI(), OTA_HOSTNAME,
               otaStarted ? "ready" : "off");
  } else if (strlen(WIFI_SSID) == 0) {
    out.printf("[STATUS] fw %s | WiFi disabled (no SSID)\n", FW_VERSION);
  } else {
    out.printf("[STATUS] fw %s | WiFi \"%s\" NOT connected (status %d) - send 'w' to scan\n",
               FW_VERSION, WIFI_SSID, WiFi.status());
  }
}

void scan(Print& out) {
  out.println("[WiFi] scanning (~3 s)...");
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    out.println("[WiFi] no networks found - is the antenna attached?");
    return;
  }
  bool found = false;
  for (int i = 0; i < n; i++) {
    bool match = WiFi.SSID(i) == WIFI_SSID;
    found |= match;
    out.printf("  %c %-32s ch %2d  %4d dBm  %s\n", match ? '*' : ' ', WiFi.SSID(i).c_str(),
               WiFi.channel(i), WiFi.RSSI(i),
               WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
  }
  out.printf("[WiFi] %d networks; \"%s\" %s\n", n, WIFI_SSID,
             found ? "FOUND (*)" : "NOT FOUND - check spelling/case, 2.4 GHz, antenna");
  WiFi.scanDelete();
}

}  // namespace ota
