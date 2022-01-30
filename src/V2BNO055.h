// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "bno055/bno055_support.h"
#include <Arduino.h>
#include <Wire.h>

class V2BNO055 {
public:
  constexpr V2BNO055(TwoWire *i2c, uint8_t pin_reset, uint8_t pin_interrupt) :
    _pin_reset(pin_reset),
    _pin_interrupt(pin_interrupt),
    _i2c(i2c) {}

  void begin(uint8_t mode = BNO055_OPERATION_MODE_NDOF);
  bool readEuler(float &heading, float &roll, float &pitch);
  bool readQuaternion(float &w, float &x, float &y, float &z);
  bool readGravity(float &x, float &y, float &z);
  bool readAcceleration(float &x, float &y, float &z);
  float readTemperature();

private:
  uint8_t _pin_reset;
  uint8_t _pin_interrupt;
  TwoWire *_i2c;
};
