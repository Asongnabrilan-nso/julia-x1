#include "discovery.h"

#include <WiFi.h>
#include <fcntl.h>
#include <lwip/etharp.h>
#include <lwip/netif.h>
#include <lwip/sockets.h>

#include "config.h"

namespace discovery {
namespace {

// Opens non-blocking connections to a batch of hosts and reports which accepted.
void probeBatch(uint32_t netBase, int first, int count, bool* open) {
  int fds[DISCOVERY_BATCH];
  for (int i = 0; i < count; i++) {
    open[i] = false;
    fds[i] = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fds[i] < 0) continue;
    lwip_fcntl(fds[i], F_SETFL, lwip_fcntl(fds[i], F_GETFL, 0) | O_NONBLOCK);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(ROBOT_PORT);
    a.sin_addr.s_addr = htonl(netBase | (uint32_t)(first + i));
    lwip_connect(fds[i], (sockaddr*)&a, sizeof(a));  // EINPROGRESS expected
  }

  uint32_t start = millis();
  int pending = count;
  while (pending > 0 && millis() - start < DISCOVERY_CONNECT_MS) {
    fd_set wset;
    FD_ZERO(&wset);
    int maxfd = -1;
    for (int i = 0; i < count; i++) {
      if (fds[i] >= 0) {
        FD_SET(fds[i], &wset);
        maxfd = max(maxfd, fds[i]);
      }
    }
    if (maxfd < 0) break;
    timeval tv{0, 50 * 1000};
    if (lwip_select(maxfd + 1, nullptr, &wset, nullptr, &tv) <= 0) continue;
    for (int i = 0; i < count; i++) {
      if (fds[i] < 0 || !FD_ISSET(fds[i], &wset)) continue;
      int err = 0;
      socklen_t len = sizeof(err);
      lwip_getsockopt(fds[i], SOL_SOCKET, SO_ERROR, &err, &len);
      open[i] = err == 0;
      lwip_close(fds[i]);
      fds[i] = -1;
      pending--;
    }
  }
  for (int i = 0; i < count; i++) {
    if (fds[i] >= 0) lwip_close(fds[i]);
  }
}

}  // namespace

String macToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
           mac[4], mac[5]);
  return String(buf);
}

bool macOf(IPAddress ip, uint8_t mac[6]) {
  // Read-only lookup in lwIP's ARP cache; we've just exchanged packets with
  // this host, so it has a stable entry.
  for (size_t i = 0; i < ARP_TABLE_SIZE; i++) {
    ip4_addr_t* eip;
    struct netif* nif;
    struct eth_addr* eth;
    if (etharp_get_entry(i, &eip, &nif, &eth) && eip->addr == (uint32_t)ip) {
      memcpy(mac, eth->addr, 6);
      return true;
    }
  }
  return false;
}

bool probe(IPAddress ip, uint8_t mac[6], bool* hasMac) {
  WiFiClient c;
  if (!c.connect(ip, ROBOT_PORT, DISCOVERY_HTTP_MS)) return false;
  c.print("GET / HTTP/1.0\r\nHost: robot\r\nConnection: close\r\n\r\n");
  // The page title is near the top: <title>Julia X2 Remote</title>
  char buf[1200];
  size_t n = 0;
  uint32_t start = millis();
  while (n < sizeof(buf) - 1 && millis() - start < DISCOVERY_HTTP_MS) {
    int avail = c.available();
    if (avail > 0) {
      n += c.read((uint8_t*)buf + n, min((size_t)avail, sizeof(buf) - 1 - n));
    } else if (!c.connected()) {
      break;
    } else {
      delay(5);
    }
  }
  c.stop();
  buf[n] = 0;
  if (!strstr(buf, ROBOT_PAGE_MARKER)) return false;
  *hasMac = macOf(ip, mac);
  return true;
}

uint8_t scan(Robot* out, uint8_t maxOut) {
  if (WiFi.status() != WL_CONNECTED) return 0;
  uint32_t self = ntohl((uint32_t)WiFi.localIP());
  uint32_t mask = ntohl((uint32_t)WiFi.subnetMask());
  // Only /24 (or smaller) networks are scanned: a /16 would take minutes.
  uint32_t base = self & 0xFFFFFF00u;
  if (mask < 0xFFFFFF00u) {
    Serial.println("[DISC] network larger than /24 - scanning own /24 only");
  }

  Serial.printf("[DISC] scanning %u.%u.%u.1-254 port %u ...\n", base >> 24, (base >> 16) & 255,
                (base >> 8) & 255, ROBOT_PORT);
  uint32_t t0 = millis();
  uint8_t found = 0;
  bool open[DISCOVERY_BATCH];
  for (int first = 1; first <= 254 && found < maxOut; first += DISCOVERY_BATCH) {
    int count = min(DISCOVERY_BATCH, 255 - first);
    probeBatch(base, first, count, open);
    for (int i = 0; i < count && found < maxOut; i++) {
      if (!open[i]) continue;
      IPAddress ip(htonl(base | (uint32_t)(first + i)));
      if (ip == WiFi.localIP()) continue;
      Robot r{ip, {0}, false};
      if (probe(ip, r.mac, &r.hasMac)) {
        Serial.printf("[DISC] julia-X2 at %s  mac %s\n", ip.toString().c_str(),
                      r.hasMac ? macToString(r.mac).c_str() : "?");
        out[found++] = r;
      } else {
        Serial.printf("[DISC] %s has port %u open but is not a julia-X2\n", ip.toString().c_str(),
                      ROBOT_PORT);
      }
    }
  }
  Serial.printf("[DISC] done in %lu ms, %u robot(s)\n", millis() - t0, found);
  return found;
}

}  // namespace discovery
