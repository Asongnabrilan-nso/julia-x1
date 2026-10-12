// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// julia-X2 face -- see face.h for the interface.
//
// This is a .cpp (unlike the header-only modules next to it) because FluxGarage_RoboEyes.h #defines
// short global macros (N, E, S, W, ON, OFF, DEFAULT, HAPPY, ...) and defines global variables. Kept
// in its own translation unit, none of that can leak into sketch.ino.
//
// Rendering: RoboEyes draws each frame into a 1-bit 320x240 canvas in RAM (9.6 KB). display()
// compares it with what is already on the glass and sends only the rectangles that changed. That
// matters here: on the UNO Q, SPI2 runs interrupt-driven with no DMA and no FIFO, so every byte
// costs the CPU an interrupt. A full 16-bit frame is 150 KB, and pushing that in one go would starve
// the 200 Hz balance thread. Changed pixels are therefore sent in small chunks with a short sleep
// between chunks, which bounds how long the balance thread can be held off.

#include "face.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <FluxGarage_RoboEyes.h>  // last: it defines N/E/S/W/ON/OFF/DEFAULT/... as macros

namespace face {
namespace {

// ---------------------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------------------
const int16_t SCREEN_W = 320, SCREEN_H = 240;          // landscape
const uint8_t ROTATION = 1;              // 1 or 3: use 3 if the face is upside down
const uint32_t SPI_HZ = 24000000;        // lower to 12 MHz if the image shows noise / wrong pixels
const size_t CHUNK_BYTES = 256;          // SPI burst size (128 pixels)...
const uint32_t CHUNK_GAP_US = 500;       // ...then sleep this long so the balance thread can run
const uint16_t COLOR_BG = ILI9341_BLACK;
const int THREAD_STACK = 3072;
const int THREAD_PRIORITY = 14;          // = loop(); the balance thread (3) always preempts it

const uint8_t FPS = 25;                  // upper bound; large changes render slower (see above)
const uint8_t EYE_W = 96, EYE_H = 96, EYE_RADIUS = 22;
const int EYE_SPACE = 40;
const uint8_t LAUGH_AMPL_PX = 10;        // RoboEyes' own amplitudes are sized for a 128x64 OLED
const uint8_t CONFUSED_AMPL_PX = 24;
const uint32_t ANIM_MS = 600;

const bool MIRROR_GAZE = true;  // the public faces the robot: its right is the viewer's left
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
// Pixel streaming (call between tft.startWrite() and tft.endWrite())
// ---------------------------------------------------------------------------------------
uint8_t chunk[CHUNK_BYTES];
size_t chunkLen = 0;

void flushChunk() {
  if (chunkLen == 0) return;
  spi.transfer(chunk, chunkLen);  // overwrites chunk with received bytes; it is refilled anyway
  chunkLen = 0;
  k_usleep(CHUNK_GAP_US);
}

inline void pushPixel(uint8_t hi, uint8_t lo) {
  chunk[chunkLen++] = hi;
  chunk[chunkLen++] = lo;
  if (chunkLen >= CHUNK_BYTES) flushChunk();
}

void clearGlass() {
  tft.startWrite();
  tft.setAddrWindow(0, 0, SCREEN_W, SCREEN_H);
  for (uint32_t i = 0; i < (uint32_t)SCREEN_W * SCREEN_H; ++i) pushPixel(COLOR_BG >> 8, COLOR_BG & 0xFF);
  flushChunk();
  tft.endWrite();
}

// ---------------------------------------------------------------------------------------
// 1-bit canvas with the clearDisplay()/display() interface RoboEyes expects
// ---------------------------------------------------------------------------------------
class EyeCanvas : public Adafruit_GFX {
 public:
  EyeCanvas() : Adafruit_GFX(SCREEN_W, SCREEN_H) {}

  // RoboEyes calls these around every frame.
  void clearDisplay() { memset(cur_, 0, sizeof(cur_)); }
  void display();

  // Eye colour for the next display(). A change repaints every lit pixel once.
  void setInk(uint16_t color) {
    if (color == ink_) return;
    ink_ = color;
    repaintAll_ = true;
  }

  void drawPixel(int16_t x, int16_t y, uint16_t c) override {
    if (x < 0 || y < 0 || x >= SCREEN_W || y >= SCREEN_H) return;
    uint8_t& b = cur_[y * STRIDE + (x >> 3)];
    uint8_t m = 0x80 >> (x & 7);
    if (c) b |= m; else b &= ~m;
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override {
    if (w < 0) { x += w + 1; w = -w; }
    if (h < 0) { y += h + 1; h = -h; }
    int16_t x1 = x + w - 1, y1 = y + h - 1;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
    if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
    if (w == 0 || h == 0 || x > x1 || y > y1) return;
    for (int16_t r = y; r <= y1; ++r) span(r, x, x1, c != 0);
  }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) override { fillRect(x, y, w, 1, c); }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) override { fillRect(x, y, 1, h, c); }
  void fillScreen(uint16_t c) override { memset(cur_, c ? 0xFF : 0x00, sizeof(cur_)); }

 private:
  static const int16_t STRIDE = SCREEN_W / 8;
  static const int MAX_WINS = 12;
  static const int RUN_MERGE_GAP = 2;  // bytes; closer runs are sent as one (fewer address windows)

  struct Win {
    int16_t b0, b1, y0, y1;  // byte columns and rows, inclusive
    bool open, touched;
  };

  void span(int16_t y, int16_t x0, int16_t x1, bool on) {
    uint8_t* row = cur_ + y * STRIDE;
    int16_t b0 = x0 >> 3, b1 = x1 >> 3;
    uint8_t m0 = 0xFF >> (x0 & 7);
    uint8_t m1 = (uint8_t)(0xFF << (7 - (x1 & 7)));
    if (b0 == b1) {
      set(row[b0], m0 & m1, on);
      return;
    }
    set(row[b0], m0, on);
    if (b1 - b0 > 1) memset(row + b0 + 1, on ? 0xFF : 0x00, b1 - b0 - 1);
    set(row[b1], m1, on);
  }
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
void EyeCanvas::addRun(int16_t y, int16_t b0, int16_t b1) {
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

void EyeCanvas::sendWin(Win& w) {
  if (!w.open) return;
  w.open = false;
  int16_t x0 = w.b0 * 8, x1 = w.b1 * 8 + 7;
  tft.setAddrWindow(x0, w.y0, x1 - x0 + 1, w.y1 - w.y0 + 1);
  const uint8_t inkHi = ink_ >> 8, inkLo = ink_ & 0xFF;
  const uint8_t bgHi = COLOR_BG >> 8, bgLo = COLOR_BG & 0xFF;
  for (int16_t y = w.y0; y <= w.y1; ++y) {
    const uint8_t* row = cur_ + y * STRIDE;
    for (int16_t x = x0; x <= x1; ++x) {
      if (row[x >> 3] & (0x80 >> (x & 7))) pushPixel(inkHi, inkLo);
      else pushPixel(bgHi, bgLo);
    }
  }
}

void EyeCanvas::display() {
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
        tft.startWrite();
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
  if (started) {
    flushChunk();
    tft.endWrite();
  }
  memcpy(shown_, cur_, sizeof(cur_));
  repaintAll_ = false;
}

EyeCanvas canvas;
RoboEyes<EyeCanvas> eyes(canvas);

// ---------------------------------------------------------------------------------------
// Shared state (written by loop() / the Bridge thread, read by the face thread)
// ---------------------------------------------------------------------------------------
volatile uint8_t obsState = ROBOT_IDLE;
volatile float obsFwd = 0.0f, obsSteer = 0.0f;
volatile bool obsNear = false, obsBlocked = false;
volatile uint8_t overrideMood = MOOD_AUTO;
volatile uint8_t pendingAnim = ANIM_NONE;

char logBuf[160];
volatile bool logReady = false;

// ---------------------------------------------------------------------------------------
// Expression logic (face thread only)
// ---------------------------------------------------------------------------------------
uint8_t animKind = ANIM_NONE;
uint32_t animEndMs = 0;
uint32_t lastActiveMs = 0, lastGreetMs = 0;
uint8_t prevState = ROBOT_IDLE;
bool prevNear = false, prevBlocked = false;

void stopAnim() {
  eyes.setVFlicker(false, 0);
  eyes.setHFlicker(false, 0);
  animKind = ANIM_NONE;
}

void startAnim(uint8_t a, uint32_t now) {
  if (a == ANIM_BLINK) {
    eyes.blink();
    return;
  }
  stopAnim();
  if (a == ANIM_LAUGH) eyes.setVFlicker(true, LAUGH_AMPL_PX);
  else if (a == ANIM_CONFUSED) eyes.setHFlicker(true, CONFUSED_AMPL_PX);
  else return;
  animKind = a;
  animEndMs = now + ANIM_MS;
}

unsigned char roboMood(uint8_t m) {
  switch (m) {
    case MOOD_HAPPY: return HAPPY;
    case MOOD_TIRED: return TIRED;
    case MOOD_ANGRY: return ANGRY;
    default: return DEFAULT;
  }
}

uint16_t moodColor(uint8_t m) {
  switch (m) {
    case MOOD_HAPPY: return COLOR_HAPPY;
    case MOOD_TIRED: return COLOR_TIRED;
    case MOOD_ANGRY: return COLOR_ANGRY;
    default: return COLOR_NEUTRAL;
  }
}

void updateExpression(uint32_t now) {
  const uint8_t st = obsState;
  const float fwd = obsFwd, steer = obsSteer;
  const bool near = obsNear, blocked = obsBlocked;
  const uint8_t forced = overrideMood;
  const bool fault = st == ROBOT_FAULT, armed = st == ROBOT_ARMED;
  const bool driving = fabsf(fwd) > DRIVE_DEADBAND || fabsf(steer) > DRIVE_DEADBAND;
  if (driving) lastActiveMs = now;

  // ---- mood + colour ----
  uint8_t mood;
  uint16_t color;
  if (forced != MOOD_AUTO) {
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
  eyes.setMood(roboMood(mood));
  eyes.setSweat(forced == MOOD_AUTO && fault);
  canvas.setInk(color);

  // ---- gaze ----
  const unsigned char CENTER = 0;  // RoboEyes: any value other than N..NW means middle centre
  bool wander = false;
  unsigned char look = CENTER;
  if (fault) look = S;                                       // hangs its head
  else if (armed) look = N;                                  // looks up at whoever holds it
  else if (fabsf(steer) > TURN_LOOK) look = ((steer > 0) == MIRROR_GAZE) ? W : E;
  else if (fwd < -DRIVE_DEADBAND) look = S;                  // watches where it backs up
  else if (driving) look = CENTER;
  else if (near) look = N;                                   // someone is standing in front of it
  else if (now - lastActiveMs > IDLE_WANDER_MS) wander = true;
  eyes.setCuriosity(look == W || look == E);
  if (wander) {
    eyes.setIdleMode(true, 2, 3);
  } else {
    eyes.setIdleMode(false);
    eyes.setPosition(look);
  }

  // ---- one-shot reactions ----
  uint8_t req = pendingAnim;
  if (req != ANIM_NONE) {
    pendingAnim = ANIM_NONE;
    startAnim(req, now);
  } else if (st != prevState && st == ROBOT_BALANCING) {
    startAnim(ANIM_LAUGH, now);     // proud to be standing
  } else if (st != prevState && fault) {
    startAnim(ANIM_CONFUSED, now);  // what just happened?
  } else if (blocked && !prevBlocked) {
    startAnim(ANIM_CONFUSED, now);  // wants to go forward but something is in the way
  } else if (near && !prevNear && !driving && !fault && now - lastGreetMs > GREET_COOLDOWN_MS) {
    lastGreetMs = now;
    startAnim(ANIM_LAUGH, now);     // greets someone walking up to it
  }
  prevState = st;
  prevNear = near;
  prevBlocked = blocked;

  if (animKind != ANIM_NONE && (int32_t)(now - animEndMs) >= 0) stopAnim();
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

  eyes.begin(SCREEN_W, SCREEN_H, FPS);
  eyes.setWidth(EYE_W, EYE_W);
  eyes.setHeight(EYE_H, EYE_H);
  eyes.setBorderradius(EYE_RADIUS, EYE_RADIUS);
  eyes.setSpacebetween(EYE_SPACE);
  eyes.setAutoblinker(true, 3, 4);
  // RoboEyes' start position assumes a 128x64 screen (and reads spaceBetweenDefault before it is
  // initialised -- the compiler warning from inside the library): jump straight to the centre.
  eyes.setPosition(0);
  eyes.eyeLx = eyes.eyeLxNext;
  eyes.eyeLy = eyes.eyeLyNext;
  eyes.eyeRx = eyes.eyeLxNext + EYE_W + EYE_SPACE;
  eyes.eyeRy = eyes.eyeLyNext;
  lastActiveMs = millis();

  snprintf(logBuf, sizeof(logBuf),
           "TFT face ready (ILI9341 on D10/D9/D7): power mode 0x%02X, self-diag 0x%02X "
           "(typically 0x94 / 0xC0; 0x00 or 0xFF = no answer on MISO D12)",
           power, diag);
  logReady = true;
}

void step() {
  updateExpression(millis());
  eyes.update();  // draws + pushes a frame when the frame interval has elapsed
}

void threadEntry(void*, void*, void*) {
  setupDisplay();
  for (;;) {
    step();
    k_msleep(5);
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
  step();
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
