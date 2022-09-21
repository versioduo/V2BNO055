// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2BNO055.h"

static struct bno055_t _sensor;

enum class State {
  Idle,
  Reset,
  Delay,
  WaitForIdle,
  Crystal,
  WaitForStatus,
  Mode,
  WaitForRunning,
  Running,
  Ready,
  _count
};

static struct {
  State state;
  uint8_t status;
  unsigned long usec;
} _reset{};

void V2BNO055::begin() {
  BNO_Init(&_sensor, _i2c);
}

void V2BNO055::reset(uint8_t mode) {
  handleReset();
  _mode        = mode;
  _reset.state = State::Reset;
}

void V2BNO055::loop() {
  switch (_reset.state) {
    case State::Idle:
    case State::Ready:
      return;

    case State::Reset:
      // Reset sensor.
      bno055_set_sys_rst(1);
      _reset.state = State::Delay;
      _reset.usec  = micros();
      break;

    case State::Delay:
      if ((unsigned long)(micros() - _reset.usec) < 600 * 1000)
        break;

      _reset.state = State::WaitForIdle;
      _reset.usec  = micros();
      break;

    case State::WaitForIdle:
      if ((unsigned long)(micros() - _reset.usec) < 1000)
        break;

      bno055_get_sys_stat_code(&_reset.status);
      _reset.usec = micros();

      if (_reset.status == 0)
        _reset.state = State::Crystal;
      break;

    case State::Crystal:
      // Enable external crystal.
      bno055_set_clk_src(1);
      _reset.state = State::WaitForStatus;
      break;

    case State::WaitForStatus:
      bno055_get_stat_main_clk(&_reset.status);
      if ((_reset.status & 1) == 0)
        _reset.state = State::Mode;
      break;

    case State::Mode:
      bno055_set_operation_mode(_mode);
      _reset.state = State::WaitForRunning;
      _reset.usec  = 0;
      break;

    case State::WaitForRunning:
      if ((unsigned long)(micros() - _reset.usec) < 1000)
        break;

      bno055_get_sys_stat_code(&_reset.status);
      _reset.usec = micros();

      if (_reset.status == 5)
        _reset.state = State::Running;
      break;

    case State::Running:
      if ((unsigned long)(micros() - _reset.usec) < 1000)
        break;

      _reset.state = State::Ready;
      break;
  }
}

bool V2BNO055::readEulerData(float &yaw, float &pitch, float &roll) {
  if (_reset.state != State::Ready)
    return false;

  struct bno055_euler_t sensorData;
  if (bno055_read_euler_hrp(&sensorData) != BNO055_SUCCESS)
    return false;

  yaw   = (float)sensorData.h / (float)BNO055_EULER_DIV_DEG;
  pitch = (float)sensorData.p / (float)BNO055_EULER_DIV_DEG;
  roll  = (float)sensorData.r / (float)BNO055_EULER_DIV_DEG;
  return true;
}

bool V2BNO055::readQuaternionData(float &w, float &x, float &y, float &z) {
  if (_reset.state != State::Ready)
    return false;

  struct bno055_quaternion_t sensorData;
  if (bno055_read_quaternion_wxyz(&sensorData) != BNO055_SUCCESS)
    return false;

  w = (float)sensorData.w / (float)(1 << 14);
  x = (float)sensorData.x / (float)(1 << 14);
  y = (float)sensorData.y / (float)(1 << 14);
  z = (float)sensorData.z / (float)(1 << 14);
  return true;
}

bool V2BNO055::readGravityData(float &x, float &y, float &z) {
  if (_reset.state != State::Ready)
    return false;

  struct bno055_gravity_t sensorData;
  if (bno055_read_gravity_xyz(&sensorData) != BNO055_SUCCESS)
    return false;

  x = (float)sensorData.x / (float)BNO055_GRAVITY_DIV_MSQ;
  y = (float)sensorData.y / (float)BNO055_GRAVITY_DIV_MSQ;
  z = (float)sensorData.z / (float)BNO055_GRAVITY_DIV_MSQ;
  return true;
}

bool V2BNO055::readAccelerationData(float &x, float &y, float &z) {
  if (_reset.state != State::Ready)
    return false;

  struct bno055_linear_accel_t sensorData;
  if (bno055_read_linear_accel_xyz(&sensorData) != BNO055_SUCCESS)
    return false;

  x = (float)sensorData.x / (float)BNO055_LINEAR_ACCEL_DIV_MSQ;
  y = (float)sensorData.y / (float)BNO055_LINEAR_ACCEL_DIV_MSQ;
  z = (float)sensorData.z / (float)BNO055_LINEAR_ACCEL_DIV_MSQ;
  return true;
}

bool V2BNO055::readMagnetometerData(float &x, float &y, float &z) {
  if (_reset.state != State::Ready)
    return false;

  struct bno055_mag_t sensorData;
  if (bno055_read_mag_xyz(&sensorData) != BNO055_SUCCESS)
    return false;

  x = (float)sensorData.x / (float)BNO055_MAG_DIV_UT;
  y = (float)sensorData.y / (float)BNO055_MAG_DIV_UT;
  z = (float)sensorData.z / (float)BNO055_MAG_DIV_UT;
  return true;
}

bool V2BNO055::readTemperature(float &t) {
  if (_reset.state != State::Ready)
    return false;

  int8_t temp;
  if (bno055_read_temp_data(&temp) != BNO055_SUCCESS)
    return false;

  t = (float)temp / (float)BNO055_TEMP_DIV_CELSIUS;
  return true;
}
