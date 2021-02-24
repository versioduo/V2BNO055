#include "V2BNO055.h"
#include "bno055/bno055_support.h"

static struct bno055_t _sensor;

void V2BNO055::begin() {
  BNO_Init(&_sensor, _i2c);

  // Reset sensor.
  bno055_set_sys_rst(1);
  delay(700);

  // Wait for: idle.
  for (;;) {
    uint8_t status;
    bno055_get_sys_stat_code(&status);
    if (status == 0)
      break;

    delay(1);
  }

  // Enable external crystal.
  bno055_set_clk_src(1);

  // Wait for: clock status cleared.
  for (;;) {
    uint8_t status;
    bno055_get_stat_main_clk(&status);
    if ((status & 1) == 0)
      break;

    delay(1);
  }

  // Nine degrees of freedom - reads accel, mag, gyro and fusion data.
  bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  // Wait for: fusion algorithm running.
  for (;;) {
    uint8_t status;
    bno055_get_sys_stat_code(&status);
    if (status == 5)
      break;

    delay(1);
  }
}

bool V2BNO055::readEuler(float &heading, float &roll, float &pitch) {
  struct bno055_euler_t sensorData;
  if (bno055_read_euler_hrp(&sensorData) != BNO055_SUCCESS)
    return false;

  heading = (float)sensorData.h / (float)BNO055_EULER_DIV_DEG;
  roll    = (float)sensorData.r / (float)BNO055_EULER_DIV_DEG;
  pitch   = (float)sensorData.p / (float)BNO055_EULER_DIV_DEG;
  return true;
}

bool V2BNO055::readQuaternion(float &w, float &x, float &y, float &z) {
  struct bno055_quaternion_t sensorData;
  if (bno055_read_quaternion_wxyz(&sensorData) != BNO055_SUCCESS)
    return false;

  w = (float)sensorData.w / (float)(1 << 14);
  x = (float)sensorData.x / (float)(1 << 14);
  y = (float)sensorData.y / (float)(1 << 14);
  z = (float)sensorData.z / (float)(1 << 14);
  return true;
}

bool V2BNO055::readGravity(float &x, float &y, float &z) {
  struct bno055_gravity_t sensorData;
  if (bno055_read_gravity_xyz(&sensorData) != BNO055_SUCCESS)
    return false;

  x = (float)sensorData.x / (float)BNO055_GRAVITY_DIV_MSQ;
  y = (float)sensorData.y / (float)BNO055_GRAVITY_DIV_MSQ;
  z = (float)sensorData.z / (float)BNO055_GRAVITY_DIV_MSQ;
  return true;
}

bool V2BNO055::readAcceleration(float &x, float &y, float &z) {
  struct bno055_linear_accel_t sensorData;
  if (bno055_read_linear_accel_xyz(&sensorData) != BNO055_SUCCESS)
    return false;

  x = (float)sensorData.x / (float)BNO055_LINEAR_ACCEL_DIV_MSQ;
  y = (float)sensorData.y / (float)BNO055_LINEAR_ACCEL_DIV_MSQ;
  z = (float)sensorData.z / (float)BNO055_LINEAR_ACCEL_DIV_MSQ;
  return true;
}

float V2BNO055::readTemperature() {
  int8_t temp;
  if (bno055_read_temp_data(&temp) != BNO055_SUCCESS)
    return 0;

  return (float)temp / (float)BNO055_TEMP_DIV_CELSIUS;
}
