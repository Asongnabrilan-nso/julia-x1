#include "ota.h"

#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiManager.h>

#include "config.h"
#include "display.h"

namespace ota {
namespace {

bool otaStarted = false;
bool wasConnected = false;
volatile bool updating = false;
int lastPct = -1;

// WiFi credentials: a network saved from the setup portal (flash) takes
// priority over the compiled-in secrets.ini values, so end users never need
// to rebuild the firmware to change networks.
String ssid, pass;
WiFiManager wm;
bool portal = false;

constexpr char kNvsNs[] = "wifi";

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

void loadCredentials() {
  Preferences p;
  if (p.begin(kNvsNs, true)) {
    ssid = p.getString("ssid", "");
    pass = p.getString("pass", "");
    p.end();
  }
  if (ssid.length()) {
    Serial.printf("[WiFi] using network saved from the setup portal: \"%s\"\n", ssid.c_str());
  } else if (strlen(WIFI_SSID)) {
    ssid = WIFI_SSID;
    pass = WIFI_PASSWORD;
    Serial.printf("[WiFi] using network from secrets.ini: \"%s\"\n", ssid.c_str());
  }
}

void connectStation() {
  WiFi.mode(portal ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);  // no modem sleep: lower, steadier latency for control packets
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.printf("[WiFi] connecting to \"%s\"...\n", ssid.c_str());
}

void onPortalSaved() {
  ssid = wm.getWiFiSSID();
  pass = wm.getWiFiPass();
  Preferences p;
  if (p.begin(kNvsNs, false)) {
    p.putString("ssid", ssid);
    p.putString("pass", pass);
    p.end();
  }
  Serial.printf("[WiFi] setup portal saved network \"%s\"\n", ssid.c_str());
}

}  // namespace

void begin() {
  setupArduinoOta();
  loadCredentials();
  if (ssid.length() == 0) {
    Serial.println("[WiFi] no network configured - opening the setup portal");
    startPortal();
    return;
  }
  connectStation();
}

void startPortal() {
  if (portal) return;
  portal = true;
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT_S);
  wm.setSaveConfigCallback(onPortalSaved);
  wm.setBreakAfterConfig(true);  // close the portal after saving, even if the join fails
  wm.setTitle("Julia-x2 Joy WiFi setup");
  WiFi.mode(WIFI_AP_STA);
  wm.startConfigPortal(PORTAL_SSID, PORTAL_PASSWORD);
  Serial.printf("[WiFi] setup portal open: join \"%s\" (password %s), then open http://192.168.4.1\n",
                PORTAL_SSID, PORTAL_PASSWORD);
}

void stopPortal() {
  if (!portal) return;
  wm.stopConfigPortal();
  portal = false;
  Serial.println("[WiFi] setup portal closed");
  if (ssid.length()) connectStation();
}

bool portalActive() { return portal; }

void forgetWifi() {
  Preferences p;
  if (p.begin(kNvsNs, false)) {
    p.clear();
    p.end();
  }
  Serial.println("[WiFi] saved network erased (secrets.ini value, if any, applies after reboot)");
}

String currentSsid() { return ssid; }

void handle() {
  if (portal) {
    wm.process();
    if (!wm.getConfigPortalActive()) {  // saved, timed out, or closed from the page
      portal = false;
      Serial.println("[WiFi] setup portal finished");
      if (ssid.length()) connectStation();
    }
  }

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
    out.printf("WiFi \"%s\"  ip %s  rssi %d dBm  OTA %s\n", ssid.c_str(),
               WiFi.localIP().toString().c_str(), WiFi.RSSI(), otaStarted ? "ready" : "off");
  } else {
    out.printf("WiFi \"%s\" not connected (status %d)%s\n", ssid.c_str(), WiFi.status(),
               portal ? "  [setup portal open]" : "");
  }
}

void printStatusLine(Print& out) {
  if (wifiConnected()) {
    out.printf("[STATUS] fw %s | WiFi \"%s\" ip %s %d dBm | OTA %s.local %s\n", FW_VERSION,
               ssid.c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI(), OTA_HOSTNAME,
               otaStarted ? "ready" : "off");
  } else if (portal) {
    out.printf("[STATUS] fw %s | WiFi setup portal open: join \"%s\" (pw %s), http://192.168.4.1\n",
               FW_VERSION, PORTAL_SSID, PORTAL_PASSWORD);
  } else {
    out.printf("[STATUS] fw %s | WiFi \"%s\" NOT connected (status %d) - send 'w' to scan\n",
               FW_VERSION, ssid.c_str(), WiFi.status());
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
    bool match = WiFi.SSID(i) == ssid;
    found |= match;
    out.printf("  %c %-32s ch %2d  %4d dBm  %s\n", match ? '*' : ' ', WiFi.SSID(i).c_str(),
               WiFi.channel(i), WiFi.RSSI(i),
               WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
  }
  out.printf("[WiFi] %d networks; \"%s\" %s\n", n, ssid.c_str(),
             found ? "FOUND (*)" : "NOT FOUND - check spelling/case, 2.4 GHz, antenna");
  WiFi.scanDelete();
}

}  // namespace ota
