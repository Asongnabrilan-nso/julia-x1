#pragma once
#include <Arduino.h>

// Non-blocking buzzer: queue short tones, update() plays them out.
namespace buzzer {

void begin();
void update();  // call every loop
void beep(uint16_t freqHz = 0, uint16_t ms = 60);  // 0 = BUZZER_FREQ_HZ
void chirpUp();    // e.g. robot link connected
void chirpDown();  // e.g. robot link lost
void doubleBeep(); // e.g. TEST mode entered

}  // namespace buzzer
