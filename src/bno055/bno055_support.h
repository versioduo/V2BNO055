#pragma #once

extern "C" {
#include "bno055.h"
}

BNO055_RETURN_FUNCTION_TYPE BNO_Init(struct bno055_t *, void *userdata);
BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_read(unsigned char, unsigned char, unsigned char *, unsigned char);
BNO055_RETURN_FUNCTION_TYPE BNO055_I2C_bus_write(unsigned char, unsigned char, unsigned char *, unsigned char);
void _delay(unsigned int);
