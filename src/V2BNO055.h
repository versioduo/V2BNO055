#pragma once

#include "Arduino.h"
#include "Wire.h"

class V2BNO055 {
public:
  constexpr V2BNO055(TwoWire *i2c, uint8_t pin_reset, uint8_t pin_interrupt) :
    _pin_reset(pin_reset),
    _pin_interrupt(pin_interrupt),
    _i2c(i2c) {}

  void begin();
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
