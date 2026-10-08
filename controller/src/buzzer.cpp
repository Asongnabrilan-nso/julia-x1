#include "buzzer.h"

#include "config.h"

namespace buzzer {
namespace {

struct Note {
  uint16_t freq;  // 0 = silent gap
  uint16_t ms;
};

constexpr uint8_t kQueueLen = 8;
Note queue[kQueueLen];
uint8_t head = 0, count = 0;
bool playing = false;
uint32_t noteEndMs = 0;

void output(uint16_t freq) {
  if (BUZZER_ACTIVE) {
    digitalWrite(PIN_BUZZER, freq ? HIGH : LOW);  // active buzzer has its own oscillator
  } else {
    ledcWriteTone(BUZZER_LEDC_CHANNEL, freq);     // 0 Hz = off
  }
}

void push(uint16_t freq, uint16_t ms) {
  if (count == kQueueLen) return;  // drop rather than block
  queue[(head + count) % kQueueLen] = {freq, ms};
  count++;
}

}  // namespace

void begin() {
  if (BUZZER_ACTIVE) {
    pinMode(PIN_BUZZER, OUTPUT);
  } else {
    ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQ_HZ, 8);
    ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CHANNEL);
  }
  output(0);
}

void update() {
  uint32_t now = millis();
  if (playing && (int32_t)(now - noteEndMs) < 0) return;
  if (count == 0) {
    if (playing) output(0);
    playing = false;
    return;
  }
  Note n = queue[head];
  head = (head + 1) % kQueueLen;
  count--;
  output(n.freq);
  noteEndMs = now + n.ms;
  playing = true;
}

void beep(uint16_t freqHz, uint16_t ms) { push(freqHz ? freqHz : BUZZER_FREQ_HZ, ms); }

void chirpUp() {
  push(1800, 70);
  push(0, 30);
  push(2700, 90);
}

void chirpDown() {
  push(2700, 70);
  push(0, 30);
  push(1500, 140);
}

void doubleBeep() {
  push(BUZZER_FREQ_HZ, 50);
  push(0, 60);
  push(BUZZER_FREQ_HZ, 50);
}

}  // namespace buzzer
