#pragma once

#include <Arduino.h>
#include <LSM6DSV16BXSensor.h>

// Thin wrapper around ST's LSM6DSV16BX library (stm32duino/STM32duino LSM6DSV16BX).
// The IMU sits on the default Wire bus at 0x6A (SDO/SA0 tied to GND).

struct IMUData {
  int32_t ax, ay, az;  // acceleration, mg
  int32_t gx, gy, gz;  // angular rate, mdps
  float tempC;         // die temperature, °C
};

namespace IMU {

// Checks WHO_AM_I, resets the chip, configures ±4 g / ±2000 dps / 480 Hz,
// enables on-chip sensor fusion (game rotation vector -> FIFO, stream mode)
// and routes the FIFO watermark interrupt to INT1 on intPin.
// Wire must already be started with Wire.begin(SDA, SCL).
bool begin(int intPin);

// Call every loop. When INT1 signals new data, drains the FIFO, keeps the
// newest orientation and samples the gyro. Returns true if the orientation
// was updated (~480 Hz), false if there was nothing new.
bool update();

// Orientation from the latest update(), in degrees. All angles are in the
// IMU's own axes (remap here if the IMU is mounted rotated on the robot).
float heading();  // yaw relative to the last zeroHeading(), wrapped to [-180, 180)
float pitch();    // absolute (gravity-referenced)
float roll();     // absolute (gravity-referenced)

// Raw gyro angular rate from the latest update(), deg/s.
// Use for the rate (D) term of stabilization instead of differentiating angles.
void rates(float &gx, float &gy, float &gz);

// Latest orientation quaternion, x, y, z, w.
void quaternion(float q[4]);

// Makes the current direction heading 0 ("this way is forward").
// Only offsets yaw; pitch/roll and the fusion itself are untouched.
void zeroHeading();

// Reads accel, gyro and temperature in physical units, directly from the
// output registers (independent of update()).
bool read(IMUData &out);

// Direct access to the library object for features not wrapped here
// (FIFO, sensor fusion, tap detection, ...).
LSM6DSV16BXSensor &sensor();

}  // namespace IMU
