#include "IMU.h"

#include <Wire.h>

namespace {

// The ST library takes the 8-bit address form (LSM6DSV16BX_I2C_ADD_L = 0xD5 -> 0x6A).
LSM6DSV16BXSensor s_imu(&Wire, LSM6DSV16BX_I2C_ADD_L);

//Sets up the new data interupt
volatile bool s_dataReady = false;
void IRAM_ATTR onImuInt1() { s_dataReady = true; }

int s_intPin = -1;

// Latest values computed by update()
float s_quat[4] = {0, 0, 0, 1};  // x, y, z, w (identity = no rotation)
float s_yaw = 0, s_pitch = 0, s_roll = 0;  // degrees, raw from the quaternion
float s_rates[3] = {0, 0, 0};    // gyro, deg/s
float s_headingOffset = 0;       // yaw captured by zeroHeading()
float s_gyroSens = 0;            // gyro mdps per LSB at the configured full scale

constexpr float RAD_TO_DEGF = 57.2957795f;

// Wraps an angle in degrees to [-180, 180).
float wrap180(float deg) {
  while (deg >= 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// Quaternion (x, y, z, w) -> roll/pitch/yaw in degrees (Z-Y-X / aerospace order).
void quatToEuler(const float q[4], float &roll, float &pitch, float &yaw) {
  const float x = q[0], y = q[1], z = q[2], w = q[3];
  roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEGF;
  // Clamp: rounding can push the argument slightly past ±1 at ±90° pitch.
  pitch = asinf(constrain(2.0f * (w * y - z * x), -1.0f, 1.0f)) * RAD_TO_DEGF;
  yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEGF;
}

}  // namespace

namespace IMU {

bool begin(int intPin) {
  //Sets up the I2C bus for the IMU
  uint8_t id = 0;
  if (s_imu.ReadID(&id) != LSM6DSV16BX_OK || id != LSM6DSV16BX_ID) {
    Serial.printf("IMU not found (WHO_AM_I = 0x%02X, expected 0x%02X)\n", id, LSM6DSV16BX_ID);
    return false;
  }

  // Power down both sensors before the reset below (ST recommends this).
  s_imu.Write_Reg(LSM6DSV16BX_CTRL1, 0x00);
  s_imu.Write_Reg(LSM6DSV16BX_CTRL2, 0x00);

  // Enable_Sensor_Fusion() software-resets the control registers, then
  // configures the on-chip fusion engine (SFLP)
  if (s_imu.Enable_Sensor_Fusion() != LSM6DSV16BX_OK) return false;

  // begin() disables the I3C interface, enables BDU + register auto-increment,
  // and puts the FIFO in bypass (overridden below: fusion needs stream mode).
  // BDU means Block Data Update, which prevents reading partially updated data.
  // Auto-increment allows reading multiple registers in a row automaticaly
  // It also turns both sensors off and sets ±2 g / ±2000 dps, writing the same
  // values to the chip and the library's cached state so the two agree after
  // the reset. The settings below then change them through the library.
  if (s_imu.begin() != LSM6DSV16BX_OK) return false;

  //Sets and enables the accelerometer and gyroscope
  // ±4 g, ±2000 dps, 480 Hz (high-performance mode)
  if (s_imu.Set_X_FS(4) != LSM6DSV16BX_OK) return false;
  if (s_imu.Set_G_FS(2000) != LSM6DSV16BX_OK) return false;
  if (s_imu.Set_X_ODR(480.0f) != LSM6DSV16BX_OK) return false;
  if (s_imu.Set_G_ODR(480.0f) != LSM6DSV16BX_OK) return false;
  if (s_imu.Enable_X() != LSM6DSV16BX_OK) return false;
  if (s_imu.Enable_G() != LSM6DSV16BX_OK) return false;

  // Sensor fusion: game rotation vector (6-axis quaternion, no magnetometer)
  // at 480 Hz. SFLP ODR must not exceed the accel/gyro ODR.
  // Fusion output only comes out through the FIFO, so put it in stream mode
  // (continuous, oldest entry overwritten when full) with a watermark of 1
  // entry so each new orientation is available immediately (no batching delay).
  if (s_imu.Set_SFLP_ODR(480.0f) != LSM6DSV16BX_OK) return false;
  if (s_imu.Enable_Rotation_Vector() != LSM6DSV16BX_OK) return false;
  if (s_imu.FIFO_Set_Watermark_Level(1) != LSM6DSV16BX_OK) return false;
  if (s_imu.FIFO_Set_Mode(LSM6DSV16BX_STREAM_MODE) != LSM6DSV16BX_OK) return false;

  // Cache the gyro sensitivity so update() can convert raw counts to deg/s.
  if (s_imu.Get_G_Sensitivity(&s_gyroSens) != LSM6DSV16BX_OK) return false;

  // Route the FIFO watermark interrupt to INT1. With a watermark of 1, INT1
  // goes high as soon as a fusion result is in the FIFO and stays high until
  // the FIFO is emptied
  lsm6dsv16bx_int1_ctrl_t int1 = {};
  int1.int1_fifo_th = 1;
  if (s_imu.Write_Reg(LSM6DSV16BX_INT1_CTRL, *(uint8_t *)&int1) != LSM6DSV16BX_OK) return false;

  //On INT1 interrupt, send the data-ready signal to the IMU interrupt handler on the ESP32
  s_intPin = intPin;
  pinMode(intPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin), onImuInt1, RISING);
  s_dataReady = digitalRead(intPin) == HIGH;  // catch a sample that's already waiting
  return true;
}

bool update() {
  // Only touch the I2C bus when INT1 says there is data. Checking the pin
  // level as well as the edge flag means a missed edge can never leave INT1
  // stuck high with nobody draining the FIFO.
  if (s_intPin < 0) return false;
  if (!s_dataReady && digitalRead(s_intPin) == LOW) return false;
  s_dataReady = false;

  uint16_t n = 0;
  if (s_imu.FIFO_Get_Num_Samples(&n) != LSM6DSV16BX_OK) return false;

  // Drain every entry so INT1 drops back low. Each entry is a tag byte plus 6
  // data bytes; the data must be read even for entries we skip, otherwise the
  // FIFO does not advance. Only the newest quaternion is kept.
  bool gotQuat = false;
  for (uint16_t i = 0; i < n; i++) {
    uint8_t tag = 0;
    if (s_imu.FIFO_Get_Tag(&tag) != LSM6DSV16BX_OK) return false;

    // The tag enum is nested in a struct in ST's C header, hence the qualifier.
    if (tag == lsm6dsv16bx_fifo_out_raw_t::LSM6DSV16BX_SFLP_GAME_ROTATION_VECTOR_TAG) {
      if (s_imu.FIFO_Get_Rotation_Vector(s_quat) != LSM6DSV16BX_OK) return false;
      gotQuat = true;
    } else {
      uint8_t discard[6];
      if (s_imu.FIFO_Get_Data(discard) != LSM6DSV16BX_OK) return false;
    }
  }
  if (!gotQuat) return false;

  quatToEuler(s_quat, s_roll, s_pitch, s_yaw);

  // Raw gyro for the rate (D) term of stabilization: read alongside the new
  // orientation so both describe the same moment.
  int16_t raw[3];
  if (s_imu.Get_G_AxesRaw(raw) == LSM6DSV16BX_OK) {
    for (int i = 0; i < 3; i++) s_rates[i] = raw[i] * s_gyroSens / 1000.0f;
  }
  return true;
}

float heading() { return wrap180(s_yaw - s_headingOffset); }
float pitch() { return s_pitch; }
float roll() { return s_roll; }

void rates(float &gx, float &gy, float &gz) {
  gx = s_rates[0];
  gy = s_rates[1];
  gz = s_rates[2];
}

void quaternion(float q[4]) {
  for (int i = 0; i < 4; i++) q[i] = s_quat[i];
}

void zeroHeading() { s_headingOffset = s_yaw; }

//Creates a new IMUData object and fill it with the latest data from the IMU
bool read(IMUData &out) {
  int32_t acc[3], gyr[3];
  if (s_imu.Get_X_Axes(acc) != LSM6DSV16BX_OK) return false;
  if (s_imu.Get_G_Axes(gyr) != LSM6DSV16BX_OK) return false;
  if (s_imu.Get_Temp(&out.tempC) != LSM6DSV16BX_OK) return false;

  out.ax = acc[0]; out.ay = acc[1]; out.az = acc[2];
  out.gx = gyr[0]; out.gy = gyr[1]; out.gz = gyr[2];
  return true;
}

LSM6DSV16BXSensor &sensor() { return s_imu; }

}  // namespace IMU
