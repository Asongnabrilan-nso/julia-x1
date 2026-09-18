// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// Minimal raw-register MPU6050 driver.
//
// Why not MPU6050_light: the balancer needs (a) a fixed, known sample path, (b) the chip's
// on-board low-pass configured for our loop rate, (c) raw gyro/accel so the attitude filter in
// balance.h uses the *measured* loop dt (micros()) instead of a coarse millis() timestamp,
// and (d) one 14-byte burst read at 400 kHz (~0.45 ms) instead of several transactions.

#pragma once

#include <Arduino.h>
#include <Wire.h>

struct ImuSample {
  float ax, ay, az;  // g
  float gx, gy, gz;  // deg/s, offset NOT removed
  float tempC;
};

class Mpu6050 {
 public:
  explicit Mpu6050(TwoWire& wire, uint8_t address = 0x68) : wire_(wire), addr_(address) {}

  // Returns true if the chip answered and was configured. whoAmI() has the raw ID for logging
  // (genuine MPU6050 = 0x68; some clones report 0x70/0x71/0x72/0x98 and work identically).
  bool begin() {
    wire_.begin();
    wire_.setClock(400000);

    uint8_t id = 0;
    if (!readRegs(0x75, &id, 1) || id == 0x00 || id == 0xFF) {
      return false;
    }
    who_ = id;

    // Reset, then wake with the gyro X PLL as clock source (more stable than the internal RC).
    if (!writeReg(0x6B, 0x80)) return false;
    delay(100);
    if (!writeReg(0x6B, 0x01)) return false;
    delay(10);

    bool ok = true;
    ok &= writeReg(0x19, 0x00);  // SMPLRT_DIV = 0 -> 1 kHz internal rate, we poll the newest sample
    ok &= writeReg(0x1A, 0x02);  // DLPF_CFG = 2 -> ~94 Hz gyro / 98 Hz accel, ~3 ms delay: kills
                                 //   motor/step vibration without adding noticeable phase lag
    ok &= writeReg(0x1B, 0x08);  // gyro  +-500 deg/s  (65.5 LSB per deg/s)
    ok &= writeReg(0x1C, 0x08);  // accel +-4 g        (8192 LSB per g)
    return ok;
  }

  uint8_t whoAmI() const { return who_; }

  bool read(ImuSample& out) {
    uint8_t b[14];
    if (!readRegs(0x3B, b, sizeof(b))) return false;
    auto s16 = [&](int i) -> int16_t { return (int16_t)((b[i] << 8) | b[i + 1]); };
    out.ax = s16(0) / 8192.0f;
    out.ay = s16(2) / 8192.0f;
    out.az = s16(4) / 8192.0f;
    out.tempC = s16(6) / 340.0f + 36.53f;
    out.gx = s16(8) / 65.5f;
    out.gy = s16(10) / 65.5f;
    out.gz = s16(12) / 65.5f;
    return true;
  }

 private:
  bool writeReg(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
  }

  bool readRegs(uint8_t reg, uint8_t* dst, size_t len) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;
    if (wire_.requestFrom(addr_, len) != len) return false;
    for (size_t i = 0; i < len; ++i) {
      dst[i] = (uint8_t)wire_.read();
    }
    return true;
  }

  TwoWire& wire_;
  uint8_t addr_;
  uint8_t who_ = 0;
};
