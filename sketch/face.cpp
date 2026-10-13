// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// julia-X2 face -- see face.h for the interface.
//
// Look: a port of Grobot_Animations (github.com/tanmaywankar/Grobot_Animations, MIT): round "ring"
// eyes whose lids, tilt, pupil and size are driven by damped springs, so every mood change, blink
// and glance overshoots and settles organically. That library is written for TFT_eSPI on an ESP32,
// which does not exist on the UNO Q's Zephyr core, so its shapes and physics are re-implemented here.
//
// Rendering: each frame is drawn into a 1-bit 320x240 canvas in RAM (9.6 KB). present() compares it
// with what is already on the glass and sends only the rectangles that changed, in the eye colour.
//
// Transport: the core's SPI driver runs SPI2 interrupt-driven, one IRQ per byte, which is both slow
// and steals time from the 200 Hz balance thread. Pixel data therefore bypasses the driver and is
// written straight into SPI2's FIFO by polling (FAST_SPI). Polling runs in this low-priority thread
// and raises no interrupts, so the balance thread preempts it at any moment and never waits on it.
// The driver is still used for display init; if the fast path ever stalls, the face falls back to
// the driver (slow, chunked) and says so on the serial log.

#include "face.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <zephyr/devicetree.h>
#include <stm32u5xx.h>

namespace face {
namespace {

// ---------------------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------------------
const int16_t SCREEN_W = 320, SCREEN_H = 240;  // landscape
const uint8_t ROTATION = 1;              // 1 or 3: use 3 if the face is upside down
const uint32_t SPI_HZ = 24000000;        // rounded down to the nearest SPI clock divider. 40 MHz is
                                         // ~10% more fps on short wires; 12 MHz if pixels come out wrong
const bool FAST_SPI = true;              // false = always use the (slow) interrupt-driven driver
const uint16_t COLOR_BG = ILI9341_BLACK;
const int THREAD_STACK = 3072;
const int THREAD_PRIORITY = 14;          // = loop(); the balance thread (3) always preempts it

const uint32_t FRAME_US = 20000;         // 50 fps cap; small changes (most frames) reach it
const size_t FAST_CHUNK_BYTES = 1024;    // fast path: yield to loop() between chunks (~0.4 ms)
const size_t SLOW_CHUNK_BYTES = 256;     // driver path: burst size...
const uint32_t SLOW_CHUNK_GAP_US = 500;  // ...then sleep so the balance thread can run
const uint32_t STATS_MS = 10000;         // fps / throughput line on the serial log

// Eye geometry in Grobot "design units" (its 320x120 reference canvas), scaled onto the screen.
const float SCALE = 1.2f;
const float EYE_RADIUS = 45, PUPIL_RADIUS = 30, EYE_GAP = 20;
const float EYE_OFFSET = EYE_RADIUS + EYE_GAP;  // eye centre to screen centre
const float LID_CLOSED = 100;                   // top lid height that covers the whole eye

// Spring constants (Grobot defaults): stiffness / damping. Underdamped on purpose: it gives the bounce.
const float EYE_K = 180, EYE_C = 15;
const float PUPIL_K = 180, PUPIL_C = 22;
const float GAZE_K = 140, GAZE_C = 18;
const float MAX_DT = 0.05f;  // a stalled frame must not make the springs explode

const float GAZE_X = 18, GAZE_Y = 18;   // design units: how far the eyes travel when looking around
const float CURIOUS_GROW = 4, CURIOUS_SHRINK = 3;  // looking sideways: near eye grows, far eye shrinks
const uint32_t BLINK_MIN_MS = 2500, BLINK_MAX_MS = 6000;
const uint32_t BLINK_HOLD_MIN_MS = 120, BLINK_HOLD_MAX_MS = 220;
const float LAUGH_BOUNCE = 7;   // design units, toggled every LAUGH_STEP_MS
const uint32_t LAUGH_STEP_MS = 110;
const float CONFUSED_SHAKE = 10;
const uint32_t CONFUSED_STEP_MS = 130;
const uint32_t WANDER_MIN_MS = 1200, WANDER_MAX_MS = 3500;
const float NOD_DEPTH = 12;
const uint32_t NOD_STEP_MS = 180;
const float DIZZY_RADIUS = 14;      // design units: the gaze circles this far around the centre
const float DIZZY_TURNS_PER_S = 1.6f;
const uint32_t LOVE_BEAT_MS = 900;  // heartbeat period: two quick beats, then a pause
const uint32_t SHOWCASE_MOOD_MS = 3500;

const bool MIRROR_GAZE = false;  // true = eyes move opposite to the joystick's left/right
const float DRIVE_DEADBAND = 0.05f;
const float TURN_LOOK = 0.25f;  // |steer| above this -> look into the turn
const uint32_t IDLE_WANDER_MS = 4000;    // no driving this long -> eyes wander around
const uint32_t GREET_COOLDOWN_MS = 8000;

// RGB565 eye colours. The robot's state is readable from across the room by colour alone.
const uint16_t COLOR_NEUTRAL = 0x07FF;    // cyan
const uint16_t COLOR_HAPPY = 0x5FEC;      // mint
const uint16_t COLOR_TIRED = 0x3B5F;      // soft blue
const uint16_t COLOR_ANGRY = ILI9341_RED;
const uint16_t COLOR_ATTENTION = 0xFD20;  // amber: armed, waiting to be held upright
const uint16_t COLOR_FAULT = 0xFA60;      // red-orange
const uint16_t COLOR_SAD = 0x5D1F;        // light blue
const uint16_t COLOR_SURPRISED = 0xFFE0;  // yellow
const uint16_t COLOR_SUSPICIOUS = 0xA81F; // purple
const uint16_t COLOR_LOVE = 0xFA14;       // pink

// Eye shapes, design units (Grobot's MoodData). top/bot: lid heights; tilt > 0 slants the top lid
// down towards the nose (angry), < 0 towards the outside (sad).
struct Shape {
  float top, bot, tilt, pupil, radius;
};
const Shape SHAPE_NEUTRAL = {0, 0, 0, 30, 45};
const Shape SHAPE_HAPPY = {0, 50, 0, 30, 45};
const Shape SHAPE_ANGRY = {0, 0, 60, 30, 45};
const Shape SHAPE_SAD = {0, 0, -60, 27, 45};
const Shape SHAPE_TIRED = {30, 0, -22, 26, 43};
const Shape SHAPE_SURPRISED = {0, 0, 0, 16, 52};   // wide open, tiny pupil
const Shape SHAPE_SQUINT = {36, 30, 0, 0, 45};      // suspicious: one eye narrowed to a slit...
const Shape SHAPE_RAISED = {8, 0, 22, 30, 47};      // ...the other one with a cocked brow
const Shape SHAPE_LOVE = {0, 0, 0, 0, 44};          // drawn as a heart (pupil unused)

// ---------------------------------------------------------------------------------------
// SPI with a GPIO chip-select on D10
// ---------------------------------------------------------------------------------------
// SPI2's pin mux also claims D10 (PB9) as the hardware NSS line, and SPI.begin() re-applies that
// mux. Adafruit sets D10 up as a GPIO chip-select *before* calling begin(), which would leave its CS
// writes without effect. This wrapper hands D10 back to GPIO right after SPI.begin(); everything else
// goes straight to the board's SPI object.
class CsSafeSpi : public HardwareSPI {
 public:
  uint8_t transfer(uint8_t data) override { return SPI.transfer(data); }
  uint16_t transfer16(uint16_t data) override { return SPI.transfer16(data); }
  void transfer(void* buf, size_t count) override { SPI.transfer(buf, count); }
  void usingInterrupt(int n) override { SPI.usingInterrupt(n); }
  void notUsingInterrupt(int n) override { SPI.notUsingInterrupt(n); }
  void beginTransaction(SPISettings s) override { SPI.beginTransaction(s); }
  void endTransaction() override { SPI.endTransaction(); }
  void attachInterrupt() override { SPI.attachInterrupt(); }
  void detachInterrupt() override { SPI.detachInterrupt(); }
  void begin() override {
    SPI.begin();
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
  }
  void end() override { SPI.end(); }
};

CsSafeSpi spi;
Adafruit_ILI9341 tft(&spi, PIN_DC, PIN_CS, PIN_RST);

// ---------------------------------------------------------------------------------------
// Fast path: polled writes straight into SPI2 (the controller behind the core's `SPI`)
// ---------------------------------------------------------------------------------------
// The core's SPI driver leaves SPI2 configured (master, mode 0, MSB first, clock divider) after
// tft.begin(). A fast session keeps that, forces 8-bit frames with a 1-frame FIFO threshold, and
// starts an endless (TSIZE = 0) master transfer; writes then feed TXDR and drain RXDR by polling.
// close() suspends the transfer, disables the SPI and restores CFG1, as the driver expects.
SPI_TypeDef* const SPI_REGS = reinterpret_cast<SPI_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(spi2)));
const uint32_t SPIN_LIMIT = 200000;  // polls without progress (> 1 ms) before the path is given up
const uint32_t IFCR_ALL = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC |
                          SPI_IFCR_CRCEC | SPI_IFCR_TIFREC | SPI_IFCR_MODFC | SPI_IFCR_SUSPC;
const size_t MAX_IN_FLIGHT = 8;  // keeps the 16-byte RX FIFO from ever overflowing

bool fastOk = FAST_SPI;
bool fastOpen = false;
uint32_t savedCfg1 = 0;

bool spiStop() {
  SPI_TypeDef* s = SPI_REGS;
  bool ok = true;
  if (s->CR1 & SPI_CR1_CSTART) {
    s->CR1 |= SPI_CR1_CSUSP;
    uint32_t spins = 0;
    while ((s->CR1 & SPI_CR1_CSTART) && ++spins < SPIN_LIMIT) {}
    ok = !(s->CR1 & SPI_CR1_CSTART);
  }
  s->CR1 &= ~SPI_CR1_SPE;
  while (s->SR & SPI_SR_RXP) (void)*reinterpret_cast<volatile uint8_t*>(&s->RXDR);
  s->IFCR = IFCR_ALL;
  return ok;
}

void fastBegin() {
  SPI_TypeDef* s = SPI_REGS;
  spiStop();
  savedCfg1 = s->CFG1;
  s->CFG1 = (savedCfg1 & ~(SPI_CFG1_DSIZE | SPI_CFG1_FTHLV)) | (7u << SPI_CFG1_DSIZE_Pos);
  s->CFG2 |= SPI_CFG2_AFCNTR;  // keep SCK/MOSI driven (not floating) whenever SPE is off
  s->CR2 = 0;                  // TSIZE = 0: endless transfer, ended by CSUSP
  s->IER = 0;
  s->CR1 |= SPI_CR1_SPE;
  s->CR1 |= SPI_CR1_CSTART;
  fastOpen = true;
}

void fastEnd() {
  if (!fastOpen) return;
  fastOpen = false;
  if (!spiStop()) fastOk = false;
  SPI_REGS->CFG1 = savedCfg1;
}

// Sends n bytes and returns once all of them are on the wire. False = the controller stopped
// responding; the caller then abandons the fast path.
bool fastWrite(const uint8_t* p, size_t n) {
  SPI_TypeDef* s = SPI_REGS;
  volatile uint8_t* tx = reinterpret_cast<volatile uint8_t*>(&s->TXDR);  // 8-bit access = 1 frame
  volatile uint8_t* rx = reinterpret_cast<volatile uint8_t*>(&s->RXDR);
  size_t sent = 0, received = 0;
  uint32_t spins = 0;
  while (received < n) {
    const uint32_t sr = s->SR;
    bool progress = false;
    if (sent < n && sent - received < MAX_IN_FLIGHT && (sr & SPI_SR_TXP)) {
      *tx = p[sent++];
      progress = true;
    }
    if (sr & SPI_SR_RXP) {
      (void)*rx;
      ++received;
      progress = true;
    }
    if (progress) spins = 0;
    else if (++spins > SPIN_LIMIT) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------------------
// Display writes: fast path, or the driver as fallback
// ---------------------------------------------------------------------------------------
uint8_t chunk[FAST_CHUNK_BYTES];
size_t chunkLen = 0;
uint32_t bytesSent = 0;  // for the stats line

void sendBytes(const uint8_t* p, size_t n) {
  bytesSent += n;
  if (fastOpen) {
    if (fastWrite(p, n)) return;
    fastEnd();
    fastOk = false;  // the frame is lost; the next frame repaints everything via the driver
    return;
  }
  spi.transfer(const_cast<uint8_t*>(p), n);  // overwrites the buffer with received bytes; fine
}

size_t chunkLimit() { return fastOpen ? FAST_CHUNK_BYTES : SLOW_CHUNK_BYTES; }

void flushChunk() {
  if (chunkLen == 0) return;
  sendBytes(chunk, chunkLen);
  chunkLen = 0;
  if (fastOpen) k_yield();  // let loop() run; the balance thread never needs this, it preempts
  else k_usleep(SLOW_CHUNK_GAP_US);
}

inline void pushPixel(uint8_t hi, uint8_t lo) {
  chunk[chunkLen++] = hi;
  chunk[chunkLen++] = lo;
  if (chunkLen >= chunkLimit()) flushChunk();
}

void fastCommand(uint8_t cmd, const uint8_t* data, size_t n) {
  digitalWrite(PIN_DC, LOW);
  sendBytes(&cmd, 1);
  digitalWrite(PIN_DC, HIGH);
  if (n && fastOpen) sendBytes(data, n);
}

void beginWrite() {
  tft.startWrite();  // CS low (+ driver transaction settings)
  if (fastOk) fastBegin();
}

void endWrite() {
  flushChunk();
  fastEnd();
  tft.endWrite();
}

// Pixels pushed after this fill the rectangle row by row.
void setWindow(int16_t x, int16_t y, int16_t w, int16_t h) {
  flushChunk();
  if (fastOpen) {
    const uint16_t x1 = x + w - 1, y1 = y + h - 1;
    const uint8_t cols[4] = {uint8_t(x >> 8), uint8_t(x), uint8_t(x1 >> 8), uint8_t(x1)};
    const uint8_t rows[4] = {uint8_t(y >> 8), uint8_t(y), uint8_t(y1 >> 8), uint8_t(y1)};
    fastCommand(ILI9341_CASET, cols, 4);
    fastCommand(ILI9341_PASET, rows, 4);
    fastCommand(ILI9341_RAMWR, nullptr, 0);
    if (fastOpen) return;
    // The fast path died during the commands: redo them through the driver.
  }
  tft.setAddrWindow(x, y, w, h);
}

void clearGlass() {
  beginWrite();
  setWindow(0, 0, SCREEN_W, SCREEN_H);
  for (uint32_t i = 0; i < (uint32_t)SCREEN_W * SCREEN_H; ++i) pushPixel(COLOR_BG >> 8, COLOR_BG & 0xFF);
  endWrite();
}

// ---------------------------------------------------------------------------------------
// 1-bit canvas with diff rendering
// ---------------------------------------------------------------------------------------
class Canvas {
 public:
  void clear() { memset(cur_, 0, sizeof(cur_)); }

  // Eye colour for the next present(). A change repaints every lit pixel once.
  void setInk(uint16_t color) {
    if (color == ink_) return;
    ink_ = color;
    repaintAll_ = true;
  }
  // The glass content is unknown (e.g. a frame was lost): repaint everything next time.
  void invalidate() {
    memset(shown_, 0xFF, sizeof(shown_));
    repaintAll_ = true;
  }

  void fillRect(float x, float y, float w, float h, bool on) {
    if (w <= 0 || h <= 0) return;
    const int x0 = (int)lroundf(x), x1 = (int)lroundf(x + w) - 1;
    const int y0 = (int)lroundf(y), y1 = (int)lroundf(y + h) - 1;
    for (int r = y0; r <= y1; ++r) span(r, x0, x1, on);
  }

  // Pixel centres at integer coordinates: (x, y) is inside if within r of (cx, cy).
  void fillCircle(float cx, float cy, float r, bool on) {
    if (r <= 0) return;
    for (int y = (int)ceilf(cy - r); y <= (int)floorf(cy + r); ++y) {
      const float dy = y - cy;
      const float dx = sqrtf(fmaxf(r * r - dy * dy, 0.0f));
      span(y, (int)ceilf(cx - dx), (int)floorf(cx + dx), on);
    }
  }

  void fillTriangle(float x0, float y0, float x1, float y1, float x2, float y2, bool on) {
    const float top = fminf(y0, fminf(y1, y2)), bottom = fmaxf(y0, fmaxf(y1, y2));
    for (int y = (int)ceilf(top); y <= (int)floorf(bottom); ++y) {
      float lo = 1e9f, hi = -1e9f;
      edge(x0, y0, x1, y1, y, lo, hi);
      edge(x1, y1, x2, y2, y, lo, hi);
      edge(x2, y2, x0, y0, y, lo, hi);
      if (lo <= hi) span(y, (int)ceilf(lo), (int)floorf(hi), on);
    }
  }

  void present();

 private:
  static const int16_t STRIDE = SCREEN_W / 8;
  static const int MAX_WINS = 12;
  static const int RUN_MERGE_GAP = 2;  // bytes; closer runs are sent as one (fewer address windows)

  struct Win {
    int16_t b0, b1, y0, y1;  // byte columns and rows, inclusive
    bool open, touched;
  };

  static void edge(float xa, float ya, float xb, float yb, int y, float& lo, float& hi) {
    if ((y < ya && y < yb) || (y > ya && y > yb)) return;
    float x;
    if (ya == yb) {
      lo = fminf(lo, fminf(xa, xb));
      hi = fmaxf(hi, fmaxf(xa, xb));
      return;
    }
    x = xa + (xb - xa) * (y - ya) / (yb - ya);
    lo = fminf(lo, x);
    hi = fmaxf(hi, x);
  }

 public:
  // Horizontal run of pixels x0..x1, clipped to the screen.
  void span(int y, int x0, int x1, bool on) {
    if (y < 0 || y >= SCREEN_H) return;
    if (x0 < 0) x0 = 0;
    if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
    if (x0 > x1) return;
    uint8_t* row = cur_ + y * STRIDE;
    const int b0 = x0 >> 3, b1 = x1 >> 3;
    const uint8_t m0 = 0xFF >> (x0 & 7);
    const uint8_t m1 = (uint8_t)(0xFF << (7 - (x1 & 7)));
    if (b0 == b1) {
      set(row[b0], m0 & m1, on);
      return;
    }
    set(row[b0], m0, on);
    if (b1 - b0 > 1) memset(row + b0 + 1, on ? 0xFF : 0x00, b1 - b0 - 1);
    set(row[b1], m1, on);
  }

 private:
  static void set(uint8_t& b, uint8_t m, bool on) {
    if (on) b |= m; else b &= ~m;
  }

  bool changed(int i) const { return repaintAll_ ? ((cur_[i] | shown_[i]) != 0) : (cur_[i] != shown_[i]); }
  void addRun(int16_t y, int16_t b0, int16_t b1);
  void sendWin(Win& w);

  uint8_t cur_[STRIDE * SCREEN_H];    // frame being drawn
  uint8_t shown_[STRIDE * SCREEN_H];  // what the glass shows (all background after clearGlass())
  Win wins_[MAX_WINS];
  uint16_t ink_ = COLOR_NEUTRAL;
  bool repaintAll_ = false;
};

// Changed bytes are grouped into rectangles: a run of changed bytes in one row extends any open
// rectangle it overlaps in the row above, so a moving eye edge becomes one tall, narrow window.
void Canvas::addRun(int16_t y, int16_t b0, int16_t b1) {
  for (Win& w : wins_) {
    if (w.open && w.y1 == y - 1 && b0 <= w.b1 + 1 && b1 >= w.b0 - 1) {
      if (b0 < w.b0) w.b0 = b0;
      if (b1 > w.b1) w.b1 = b1;
      w.y1 = y;
      w.touched = true;
      return;
    }
  }
  for (Win& w : wins_) {
    if (!w.open) {
      w = {b0, b1, y, y, true, true};
      return;
    }
  }
  // All slots busy (very unusual): close everything and start over with this run.
  for (Win& w : wins_) sendWin(w);
  wins_[0] = {b0, b1, y, y, true, true};
}

void Canvas::sendWin(Win& w) {
  if (!w.open) return;
  w.open = false;
  const int16_t x0 = w.b0 * 8, x1 = w.b1 * 8 + 7;
  setWindow(x0, w.y0, x1 - x0 + 1, w.y1 - w.y0 + 1);
  const uint8_t inkHi = ink_ >> 8, inkLo = ink_ & 0xFF;
  const uint8_t bgHi = COLOR_BG >> 8, bgLo = COLOR_BG & 0xFF;
  for (int16_t y = w.y0; y <= w.y1; ++y) {
    const uint8_t* row = cur_ + y * STRIDE;
    for (int16_t b = w.b0; b <= w.b1; ++b) {
      const uint8_t bits = row[b];
      for (uint8_t m = 0x80; m; m >>= 1) {
        if (bits & m) pushPixel(inkHi, inkLo);
        else pushPixel(bgHi, bgLo);
      }
    }
  }
}

void Canvas::present() {
  bool started = false;
  for (int16_t y = 0; y < SCREEN_H; ++y) {
    const int base = y * STRIDE;
    for (Win& w : wins_) w.touched = false;
    int16_t b = 0;
    while (b < STRIDE) {
      if (!changed(base + b)) { ++b; continue; }
      int16_t r0 = b, r1 = b;
      for (int16_t k = b + 1; k < STRIDE && k <= r1 + 1 + RUN_MERGE_GAP; ++k) {
        if (changed(base + k)) r1 = k;
      }
      if (!started) {
        beginWrite();
        started = true;
      }
      addRun(y, r0, r1);
      b = r1 + 1;
    }
    for (Win& w : wins_) {
      if (w.open && !w.touched) sendWin(w);
    }
  }
  for (Win& w : wins_) sendWin(w);
  if (started) endWrite();
  memcpy(shown_, cur_, sizeof(cur_));
  repaintAll_ = false;
}

Canvas canvas;

// Adafruit_GFX front end for the canvas, used for the boot intro's text.
class TextPen : public Adafruit_GFX {
 public:
  TextPen() : Adafruit_GFX(SCREEN_W, SCREEN_H) {}
  void drawPixel(int16_t x, int16_t y, uint16_t c) override { canvas.span(y, x, x, c != 0); }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override {
    for (int16_t r = y; r < y + h; ++r) canvas.span(r, x, x + w - 1, c != 0);
  }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override { fillRect(x, y, w, 1, c); }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override { fillRect(x, y, 1, h, c); }
};

TextPen pen;

// ---------------------------------------------------------------------------------------
// Spring-driven eyes (Grobot_Animations' model)
// ---------------------------------------------------------------------------------------
struct Spring {
  float pos = 0, vel = 0;
  // Hooke's law with damping, semi-implicit Euler: frame-rate independent thanks to dt.
  void step(float target, float dt, float k, float c) {
    vel += ((target - pos) * k - vel * c) * dt;
    pos += vel * dt;
  }
};

struct Eye {
  Spring top, bot, tilt, pupil, radius;
  void snap(const Shape& s) {
    top.pos = s.top; bot.pos = s.bot; tilt.pos = s.tilt; pupil.pos = s.pupil; radius.pos = s.radius;
  }
  void step(const Shape& t, float dt) {
    top.step(t.top, dt, EYE_K, EYE_C);
    bot.step(t.bot, dt, EYE_K, EYE_C);
    tilt.step(t.tilt, dt, EYE_K, EYE_C);
    pupil.step(t.pupil, dt, PUPIL_K, PUPIL_C);
    radius.step(t.radius, dt, EYE_K, EYE_C);
  }
};

Eye eyeL, eyeR;  // viewer's left / right
Spring gazeX, gazeY;
uint32_t lastStepUs = 0;

// Draws one eye: a filled disc with the pupil cut out (or a solid heart), then the lids painted
// over in background. Arguments are in pixels. isLeft = the eye on the viewer's left.
void drawEye(float ex, float ey, const Eye& e, bool isLeft, bool heart) {
  const float r = e.radius.pos * SCALE, pr = e.pupil.pos * SCALE;
  const float lid = e.top.pos * SCALE, bot = e.bot.pos * SCALE, tilt = e.tilt.pos * SCALE;
  if (heart) {
    // Two round lobes on top of a downward-pointing triangle.
    const float lobe = r * 0.52f, lobeDx = r * 0.46f, lobeY = ey - r * 0.22f;
    canvas.fillCircle(ex - lobeDx, lobeY, lobe, true);
    canvas.fillCircle(ex + lobeDx, lobeY, lobe, true);
    canvas.fillTriangle(ex - r * 0.94f, ey - r * 0.02f, ex + r * 0.94f, ey - r * 0.02f, ex, ey + r * 0.98f, true);
  } else {
    canvas.fillCircle(ex, ey, r, true);
    canvas.fillCircle(ex, ey, pr, false);
  }
  const float pad = 5 * SCALE;  // lids reach a bit past the eye so no edge pixels peek out
  const float lidY = ey - r - pad + lid;
  canvas.fillRect(ex - r - pad, ey - r - pad, 2 * r + 2 * pad, lid + pad, false);
  if (fabsf(tilt) >= 0.5f) {
    // The lid drops towards the nose for tilt > 0 (angry) or the outside for tilt < 0 (sad).
    const bool lowSideRight = isLeft == (tilt > 0);
    const float px = lowSideRight ? ex + r + pad : ex - r - pad;
    canvas.fillTriangle(ex - r - pad, lidY, ex + r + pad, lidY, px, lidY + fabsf(tilt), false);
  }
  if (bot > 0) canvas.fillRect(ex - r - pad, ey + r + pad - bot, 2 * r + 2 * pad, bot + pad, false);
}

void renderEyes(const Shape& left, const Shape& right, float gx, float gy, bool hearts) {
  const uint32_t nowUs = micros();
  float dt = (nowUs - lastStepUs) * 1e-6f;
  lastStepUs = nowUs;
  if (dt > MAX_DT) dt = MAX_DT;
  if (dt < 0.0001f) dt = 0.0001f;

  eyeL.step(left, dt);
  eyeR.step(right, dt);
  gazeX.step(gx, dt, GAZE_K, GAZE_C);
  gazeY.step(gy, dt, GAZE_K, GAZE_C);

  const float cx = SCREEN_W / 2 + gazeX.pos * SCALE, cy = SCREEN_H / 2 + gazeY.pos * SCALE;
  canvas.clear();
  drawEye(roundf(cx - EYE_OFFSET * SCALE), roundf(cy), eyeL, true, hearts);
  drawEye(roundf(cx + EYE_OFFSET * SCALE), roundf(cy), eyeR, false, hearts);
  canvas.present();
}

// ---------------------------------------------------------------------------------------
// Shared state (written by loop() / the Bridge thread, read by the face thread)
// ---------------------------------------------------------------------------------------
volatile uint8_t obsState = ROBOT_IDLE;
volatile float obsFwd = 0.0f, obsSteer = 0.0f;
volatile bool obsNear = false, obsBlocked = false;
volatile uint8_t overrideMood = MOOD_AUTO;
volatile uint8_t pendingAnim = ANIM_NONE;

char logBuf[200];
volatile bool logReady = false;

void postLog() { logReady = true; }  // logBuf filled; loop() prints it via takeLog()

// ---------------------------------------------------------------------------------------
// Expression logic (face thread only)
// ---------------------------------------------------------------------------------------
uint8_t animKind = ANIM_NONE;
uint32_t animStartMs = 0;
uint32_t nextBlinkMs = 0, blinkEndMs = 0;
bool blinking = false;
uint32_t nextWanderMs = 0;
float wanderX = 0, wanderY = 0;
uint32_t lastActiveMs = 0, lastGreetMs = 0;
uint8_t prevState = ROBOT_IDLE;
bool prevNear = false, prevBlocked = false;
bool introActive = false;
uint32_t introStartMs = 0;
uint8_t showcaseIdx = 0xFF;

uint32_t animDuration(uint8_t a) {
  switch (a) {
    case ANIM_WINK: return 450;
    case ANIM_LOOK_AROUND: return 1800;
    case ANIM_NOD: return 1100;
    case ANIM_DIZZY: return 1900;
    default: return 900;  // laugh, confused, surprise
  }
}

void startBlink(uint32_t now) {
  blinking = true;
  blinkEndMs = now + random(BLINK_HOLD_MIN_MS, BLINK_HOLD_MAX_MS);
}

void updateBlink(uint32_t now) {
  if (blinking && (int32_t)(now - blinkEndMs) >= 0) {
    blinking = false;
    nextBlinkMs = now + random(BLINK_MIN_MS, BLINK_MAX_MS);
  } else if (!blinking && (int32_t)(now - nextBlinkMs) >= 0) {
    startBlink(now);
  }
}

void startAnim(uint8_t a, uint32_t now) {
  if (a == ANIM_BLINK) {
    startBlink(now);
  } else if (a == ANIM_INTRO) {
    introActive = true;
    introStartMs = now;
    animKind = ANIM_NONE;
  } else if (a > ANIM_NONE && a < ANIM_COUNT) {
    animKind = a;
    animStartMs = now;
  }
}

// Showcase mode: cycles through these moods, each introduced by its matching animation.
const uint8_t SHOWCASE_MOODS[] = {MOOD_NEUTRAL, MOOD_HAPPY, MOOD_SURPRISED, MOOD_SUSPICIOUS,
                                  MOOD_LOVE, MOOD_SAD, MOOD_ANGRY, MOOD_TIRED};
const uint8_t SHOWCASE_ANIMS[] = {ANIM_LOOK_AROUND, ANIM_LAUGH, ANIM_SURPRISE, ANIM_NONE,
                                  ANIM_WINK, ANIM_NONE, ANIM_NONE, ANIM_NOD};
const uint8_t SHOWCASE_COUNT = sizeof(SHOWCASE_MOODS);

void moodShapes(uint8_t m, Shape& left, Shape& right) {
  switch (m) {
    case MOOD_HAPPY: left = right = SHAPE_HAPPY; break;
    case MOOD_TIRED: left = right = SHAPE_TIRED; break;
    case MOOD_ANGRY: left = right = SHAPE_ANGRY; break;
    case MOOD_SAD: left = right = SHAPE_SAD; break;
    case MOOD_SURPRISED: left = right = SHAPE_SURPRISED; break;
    case MOOD_SUSPICIOUS: left = SHAPE_SQUINT; right = SHAPE_RAISED; break;
    case MOOD_LOVE: left = right = SHAPE_LOVE; break;
    default: left = right = SHAPE_NEUTRAL; break;
  }
}

uint16_t moodColor(uint8_t m) {
  switch (m) {
    case MOOD_HAPPY: return COLOR_HAPPY;
    case MOOD_TIRED: return COLOR_TIRED;
    case MOOD_ANGRY: return COLOR_ANGRY;
    case MOOD_SAD: return COLOR_SAD;
    case MOOD_SURPRISED: return COLOR_SURPRISED;
    case MOOD_SUSPICIOUS: return COLOR_SUSPICIOUS;
    case MOOD_LOVE: return COLOR_LOVE;
    default: return COLOR_NEUTRAL;
  }
}

// ---- boot intro: "JULIA-X2" title card, then the screen closes like eyelids and the eyes open ----
const uint32_t INTRO_LINE_MS = 600;      // a line powers on in the middle
const uint32_t INTRO_REVEAL_MS = 1300;   // the line splits into rails; the title is revealed
const uint32_t INTRO_BOOT_MS = 2500;     // subtitle types out, boot bar fills
const uint32_t INTRO_MS = 2900;          // the screen closes from top and bottom
const char* const INTRO_TITLE = "JULIA-X2";
const char* const INTRO_SUBTITLE = "self-balancing robot";

float easeOut(float p) {
  p = p < 0 ? 0 : (p > 1 ? 1 : p);
  return 1 - (1 - p) * (1 - p) * (1 - p);
}

// Prints text centred on (cx, cy) using the font's real ink bounds; only the first `chars` letters.
void printCentred(const char* text, const GFXfont* font, int16_t cx, int16_t cy, size_t chars) {
  int16_t x1, y1;
  uint16_t w, h;
  pen.setFont(font);
  pen.setTextSize(1);
  pen.setTextWrap(false);
  pen.setTextColor(1);
  pen.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  pen.setCursor(cx - (int16_t)w / 2 - x1, cy - (int16_t)h / 2 - y1);
  for (size_t i = 0; text[i] && i < chars; ++i) pen.write(text[i]);
}

void drawIntro(uint32_t t) {
  const int cx = SCREEN_W / 2, cy = SCREEN_H / 2 - 12;
  canvas.clear();
  canvas.setInk(COLOR_NEUTRAL);
  if (t < INTRO_LINE_MS) {
    const float half = 130 * easeOut((float)t / INTRO_LINE_MS);
    canvas.fillRect(cx - half, cy - 1, 2 * half, 3, true);
    canvas.present();
    return;
  }
  // Title, revealed from the centre outwards while the rails move apart.
  const float p = easeOut((float)(t - INTRO_LINE_MS) / (INTRO_REVEAL_MS - INTRO_LINE_MS));
  printCentred(INTRO_TITLE, &FreeSansBold24pt7b, cx, cy, strlen(INTRO_TITLE));
  const int reveal = (int)(150 * p);
  for (int y = cy - 30; y <= cy + 30; ++y) {
    canvas.span(y, 0, cx - reveal - 1, false);
    canvas.span(y, cx + reveal, SCREEN_W - 1, false);
  }
  const float rail = 2 + 34 * p;
  canvas.fillRect(cx - 130, cy - rail - 1, 260, 3, true);
  canvas.fillRect(cx - 130, cy + rail - 1, 260, 3, true);

  if (t >= INTRO_REVEAL_MS) {
    // Subtitle types out, boot bar fills.
    const float q = fminf(1.0f, (float)(t - INTRO_REVEAL_MS) / (INTRO_BOOT_MS - INTRO_REVEAL_MS));
    printCentred(INTRO_SUBTITLE, &FreeSans9pt7b, cx, cy + 58,
                 (size_t)(strlen(INTRO_SUBTITLE) * fminf(1.0f, q * 1.6f)));
    const int barY = cy + 82, barW = 160;
    canvas.fillRect(cx - barW / 2, barY, barW, 8, true);
    canvas.fillRect(cx - barW / 2 + 2, barY + 2, barW - 4, 4, false);
    canvas.fillRect(cx - barW / 2 + 2, barY + 2, (barW - 4) * q, 4, true);
  }
  if (t >= INTRO_BOOT_MS) {
    // Close like eyelids: black comes in from the top and the bottom.
    const float lid = (SCREEN_H / 2 + 2) * easeOut((float)(t - INTRO_BOOT_MS) / (INTRO_MS - INTRO_BOOT_MS));
    canvas.fillRect(0, 0, SCREEN_W, lid, false);
    canvas.fillRect(0, SCREEN_H - lid, SCREEN_W, lid, false);
  }
  canvas.present();
}

void endIntro(uint32_t now) {
  introActive = false;
  eyeL.snap(SHAPE_NEUTRAL);
  eyeR.snap(SHAPE_NEUTRAL);
  eyeL.top.pos = eyeR.top.pos = LID_CLOSED;  // the eyes spring open...
  gazeX.pos = gazeY.pos = gazeX.vel = gazeY.vel = 0;
  lastStepUs = micros();
  blinking = false;
  nextBlinkMs = now + BLINK_MAX_MS;
  lastActiveMs = now;
  startAnim(ANIM_LOOK_AROUND, now);          // ...and look around the room
}

void step(uint32_t now) {
  const uint8_t st = obsState;
  const float fwd = obsFwd, steer = obsSteer;
  const bool near = obsNear, blocked = obsBlocked;
  const uint8_t forced = overrideMood;
  const bool fault = st == ROBOT_FAULT, armed = st == ROBOT_ARMED;
  const bool driving = fabsf(fwd) > DRIVE_DEADBAND || fabsf(steer) > DRIVE_DEADBAND;
  if (driving) lastActiveMs = now;

  // ---- one-shot reactions ----
  uint8_t req = pendingAnim;
  if (req != ANIM_NONE) {
    pendingAnim = ANIM_NONE;
    startAnim(req, now);
  } else if (st != prevState && st == ROBOT_BALANCING) {
    startAnim(ANIM_LAUGH, now);     // proud to be standing
  } else if (st != prevState && fault) {
    startAnim(ANIM_DIZZY, now);     // fell over: sees stars
  } else if (blocked && !prevBlocked) {
    startAnim(ANIM_CONFUSED, now);  // wants to go forward but something is in the way
  } else if (near && !prevNear && !driving && !fault && now - lastGreetMs > GREET_COOLDOWN_MS) {
    lastGreetMs = now;
    startAnim(random(2) ? ANIM_SURPRISE : ANIM_LAUGH, now);  // greets someone walking up to it
  }
  prevState = st;
  prevNear = near;
  prevBlocked = blocked;

  if (introActive) {
    const uint32_t t = now - introStartMs;
    if (t < INTRO_MS) {
      drawIntro(t);
      return;
    }
    endIntro(now);
  }
  const uint32_t animT = now - animStartMs;
  if (animKind != ANIM_NONE && animT >= animDuration(animKind)) animKind = ANIM_NONE;
  updateBlink(now);

  // ---- mood + colour ----
  uint8_t mood;
  uint16_t color;
  if (forced == MOOD_SHOWCASE) {
    const uint8_t idx = (now / SHOWCASE_MOOD_MS) % SHOWCASE_COUNT;
    if (idx != showcaseIdx) {
      showcaseIdx = idx;
      if (SHOWCASE_ANIMS[idx] != ANIM_NONE) startAnim(SHOWCASE_ANIMS[idx], now);
    }
    mood = SHOWCASE_MOODS[idx];
    color = moodColor(mood);
  } else if (forced != MOOD_AUTO) {
    mood = forced;
    color = moodColor(forced);
  } else if (fault) {
    mood = MOOD_TIRED;
    color = COLOR_FAULT;
  } else if (armed) {
    mood = MOOD_NEUTRAL;
    color = COLOR_ATTENTION;
  } else if (st == ROBOT_BALANCING) {
    mood = MOOD_HAPPY;
    color = COLOR_HAPPY;
  } else {
    mood = MOOD_NEUTRAL;
    color = COLOR_NEUTRAL;
  }
  if (forced != MOOD_SHOWCASE) showcaseIdx = 0xFF;
  canvas.setInk(color);
  Shape left, right;
  moodShapes(mood, left, right);
  const bool hearts = mood == MOOD_LOVE;
  if (hearts) {
    // Heartbeat: two quick pulses, then a pause.
    const uint32_t ph = now % LOVE_BEAT_MS;
    if (ph < 120 || (ph >= 220 && ph < 340)) left.radius = right.radius = SHAPE_LOVE.radius + 6;
  }

  // ---- gaze (design units; +x = viewer's right, +y = down) ----
  float gx = 0, gy = 0;
  if (fault) gy = GAZE_Y;                                    // hangs its head
  else if (armed) gy = -GAZE_Y;                              // looks up at whoever holds it
  else if (fabsf(steer) > TURN_LOOK) gx = ((steer > 0) == MIRROR_GAZE) ? -GAZE_X : GAZE_X;
  else if (fwd < -DRIVE_DEADBAND) gy = GAZE_Y;               // watches where it backs up
  else if (driving) {}                                       // straight ahead
  else if (near) gy = -GAZE_Y;                               // someone is standing in front of it
  else if (now - lastActiveMs > IDLE_WANDER_MS || forced == MOOD_SHOWCASE) {
    if ((int32_t)(now - nextWanderMs) >= 0) {
      nextWanderMs = now + random(WANDER_MIN_MS, WANDER_MAX_MS);
      const bool centre = random(4) == 0;
      wanderX = centre ? 0 : random(-100, 101) * GAZE_X / 100;
      wanderY = centre ? 0 : random(-100, 101) * GAZE_Y / 100;
    }
    gx = wanderX;
    gy = wanderY;
  }

  // ---- animation overlays ----
  switch (animKind) {
    case ANIM_LAUGH:
      if (!hearts) left = right = SHAPE_HAPPY;
      gy += ((animT / LAUGH_STEP_MS) & 1) ? LAUGH_BOUNCE : -LAUGH_BOUNCE;
      break;
    case ANIM_CONFUSED:
      left = SHAPE_HAPPY;  // Grobot's "confused": one happy eye, one sad eye
      right = SHAPE_SAD;
      gx += ((animT / CONFUSED_STEP_MS) & 1) ? CONFUSED_SHAKE : -CONFUSED_SHAKE;
      break;
    case ANIM_WINK:
      right.top = LID_CLOSED;
      left.bot += 14;  // the open eye smiles a little
      break;
    case ANIM_LOOK_AROUND:
      gx = animT < 500 ? -GAZE_X : (animT < 1000 ? GAZE_X : 0);
      gy = (animT >= 1000 && animT < 1400) ? -GAZE_Y : 0;
      break;
    case ANIM_NOD:
      gy += ((animT / NOD_STEP_MS) & 1) ? -NOD_DEPTH / 3 : NOD_DEPTH;
      break;
    case ANIM_DIZZY: {
      const float a = animT * 0.001f * DIZZY_TURNS_PER_S * 2 * (float)M_PI;
      gx = cosf(a) * DIZZY_RADIUS;
      gy = sinf(a) * DIZZY_RADIUS;
      left.pupil = 18;  // mismatched pupils look woozy
      right.pupil = 34;
      break;
    }
    case ANIM_SURPRISE:
      left = right = SHAPE_SURPRISED;
      gy -= 6;
      break;
    default:
      break;
  }

  // Curiosity: the eye on the side it looks to grows, the other one shrinks.
  if (fabsf(gx) > GAZE_X / 2) {
    Shape& nearEye = gx > 0 ? right : left;
    Shape& farEye = gx > 0 ? left : right;
    nearEye.pupil *= (nearEye.radius + CURIOUS_GROW) / nearEye.radius;  // keeps the ring even
    nearEye.radius += CURIOUS_GROW;
    farEye.pupil *= (farEye.radius - CURIOUS_SHRINK) / farEye.radius;
    farEye.radius -= CURIOUS_SHRINK;
  }
  if (blinking) left.top = right.top = LID_CLOSED;

  renderEyes(left, right, gx, gy, hearts);
}

// ---------------------------------------------------------------------------------------
// Startup + run loop
// ---------------------------------------------------------------------------------------
void setupDisplay() {
  tft.begin(SPI_HZ);
  tft.setRotation(ROTATION);
  // Only meaningful with MISO (D12) wired: 0x00/0xFF means the display did not answer.
  uint8_t power = tft.readcommand8(ILI9341_RDMODE);
  uint8_t diag = tft.readcommand8(ILI9341_RDSELFDIAG);
  clearGlass();
  if (FAST_SPI && !fastOk) clearGlass();  // the fast path failed during the first clear: redo it

  startAnim(ANIM_INTRO, millis());  // boot intro first; the eyes open when it ends

  snprintf(logBuf, sizeof(logBuf),
           "TFT face ready (ILI9341 on D10/D9/D7, %s SPI): power mode 0x%02X, self-diag 0x%02X "
           "(typically 0x94 / 0xC0; 0x00 or 0xFF = no answer on MISO D12)",
           fastOk ? "fast polled" : "driver", power, diag);
  postLog();
}

void run() {
  static uint32_t frames = 0, statsStartMs = 0, frameStartUs = 0;
  static bool wasFast = FAST_SPI;
  const uint32_t nowUs = micros();
  if (nowUs - frameStartUs < FRAME_US) return;
  frameStartUs = nowUs;

  step(millis());
  ++frames;
  if (wasFast && !fastOk) {
    // The fast path stalled mid-frame: the glass is in an unknown state.
    wasFast = false;
    canvas.invalidate();
    if (!logReady) {
      snprintf(logBuf, sizeof(logBuf), "TFT face: fast SPI stopped responding, using the slow driver");
      postLog();
    }
  }
  const uint32_t nowMs = millis();
  if (nowMs - statsStartMs >= STATS_MS) {
    if (statsStartMs != 0 && !logReady) {
      const uint32_t s = nowMs - statsStartMs;
      snprintf(logBuf, sizeof(logBuf), "TFT face: %lu fps, %lu KB/s to the display (%s SPI)",
               (unsigned long)(frames * 1000 / s), (unsigned long)(bytesSent / s),
               fastOk ? "fast" : "driver");
      postLog();
    }
    statsStartMs = nowMs;
    frames = 0;
    bytesSent = 0;
  }
}

void threadEntry(void*, void*, void*) {
  setupDisplay();
  for (;;) {
    run();
    k_msleep(1);
  }
}

}  // namespace

bool begin() {
  k_thread_stack_t* stack = k_thread_stack_alloc(THREAD_STACK, 0);
  if (!stack) return false;
  static struct k_thread thread;
  k_tid_t tid = k_thread_create(&thread, stack, THREAD_STACK, threadEntry, NULL, NULL, NULL,
                                THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(tid, "face");
  return true;
}

void service() {
  static bool ready = false;
  if (!ready) {
    setupDisplay();
    ready = true;
  }
  run();
}

void observe(uint8_t robotState, float fwd, float steer, bool near, bool blocked) {
  obsState = robotState;
  obsFwd = fwd;
  obsSteer = steer;
  obsNear = near;
  obsBlocked = blocked;
}

void setMood(uint8_t mood) {
  if (mood < MOOD_COUNT) overrideMood = mood;
}

void play(uint8_t anim) {
  if (anim > ANIM_NONE && anim < ANIM_COUNT) pendingAnim = anim;
}

const char* takeLog() {
  if (!logReady) return nullptr;
  logReady = false;
  return logBuf;
}

}  // namespace face
