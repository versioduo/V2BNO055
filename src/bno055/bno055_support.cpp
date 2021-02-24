#include "bno055_support.h"

#include <Arduino.h>
#include <Wire.h>

static TwoWire *_i2c;

BNO055_RETURN_FUNCTION_TYPE BNO_Init(struct bno055_t *bno055, void *userdata) {
  BNO055_RETURN_FUNCTION_TYPE comres = BNO055_SUCCESS;

  _i2c = (TwoWire *)userdata;

  bno055->dev_addr   = BNO055_I2C_ADDR2;
  bno055->bus_read   = BNO055_I2C_bus_read;
  bno055->bus_write  = BNO055_I2C_bus_write;
  bno055->delay_msec = _delay;
  comres             = bno055_init(bno055);

  return comres;
}

BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_read(unsigned char dev_addr,
                                                unsigned char reg_addr,
                                                unsigned char *reg_data,
                                                unsigned char cnt) {
  BNO055_RETURN_FUNCTION_TYPE comres = BNO055_SUCCESS;

  _i2c->beginTransmission(dev_addr);
  _i2c->write(reg_addr);
  _i2c->endTransmission();

  _i2c->requestFrom(dev_addr, cnt);
  while (_i2c->available()) {
    *reg_data = _i2c->read();
    reg_data++;
  }

  return comres;
}

BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_write(unsigned char dev_addr,
                                                 unsigned char reg_addr,
                                                 unsigned char *reg_data,
                                                 unsigned char cnt) {
  BNO055_RETURN_FUNCTION_TYPE comres = BNO055_SUCCESS;

  _i2c->beginTransmission(dev_addr);
  _i2c->write(reg_addr);
  for (uint8_t i = 0; i < cnt; i++) {
    _i2c->write(*reg_data);
    reg_data++;
  }
  _i2c->endTransmission();

  return comres;
}

void _delay(unsigned int period) {
  delay(long(period));
}
