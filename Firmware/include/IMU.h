#pragma once

#include <Arduino.h>
#include <Wire.h>

// Driver for the ST LSM6DSV16BX 6-axis IMU over I2C
// As a general note the register addresses and bit positions come from the datasheet

struct IMUData {
  float ax, ay, az;  // acceleration, g
  float gx, gy, gz;  // angular rate, degrees/s
  float tempC;       // die temperature, °C
};

struct IMURaw {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
};

/* 
Some terminology for class IMU:
  - CTRL_1 CTRL_2, ... are registers in the IMU that we read/write from
  - FS means "full scale" and is the same as "range" (±2 g, ±4 g, etc.)
  - XL means accelerometer, G means gyroscope, ODR means output data rate, LPF means low-pass filter
  - [a : b] means bits a down to b (inclusive) of a register byte
*/

class IMU {
 public:
  // CTRL8 FS_XL[1:0]
  enum AccelRange : uint8_t { 
    ACCEL_2G = 0x0, 
    ACCEL_4G = 0x1, 
    ACCEL_8G = 0x2, 
    ACCEL_16G = 0x3
  };

  // CTRL6 FS_G[3:0]
  enum GyroRange : uint8_t {
    GYRO_125DPS  = 0x0,
    GYRO_250DPS  = 0x1,
    GYRO_500DPS  = 0x2,
    GYRO_1000DPS = 0x3,
    GYRO_2000DPS = 0x4,
    GYRO_4000DPS = 0xC,
  };

  // CTRL1 ODR_XL[3:0] / CTRL2 ODR_G[3:0] in high-performance mode.
  enum DataRate : uint8_t {
    ODR_OFF     = 0x0,
    ODR_7_5HZ   = 0x2,
    ODR_15HZ    = 0x3,
    ODR_30HZ    = 0x4,
    ODR_60HZ    = 0x5,
    ODR_120HZ   = 0x6,
    ODR_240HZ   = 0x7,
    ODR_480HZ   = 0x8,
    ODR_960HZ   = 0x9,
    ODR_1920HZ  = 0xA,
    ODR_3840HZ  = 0xB,
    ODR_7680HZ  = 0xC,
  };

  // These are the 2 possible I2C addresses of the chip depending on what the pin is tied to
  static constexpr uint8_t ADDR_SA0_LOW  = 0x6A;  // SDO/SA0 tied to GND
  static constexpr uint8_t ADDR_SA0_HIGH = 0x6B;  // SDO/SA0 tied to VDDIO

  // Probes the given address (or both, if address == 0), checks WHO_AM_I,
  // soft-resets the chip and applies a default config (±4 g, ±500 dps, 120 Hz).
  // Wire must already be started with Wire.begin(SDA, SCL).
  //TwoWire is the default I2C class given by arduino
  bool begin(TwoWire &wire = Wire, uint8_t address = 0);

  bool setAccelRange(AccelRange range);
  bool setGyroRange(GyroRange range);
  bool setAccelDataRate(DataRate odr);
  bool setGyroDataRate(DataRate odr);

  // Routes "accelerometer data ready" to INT1 (push-pull, active-high, latched
  // until the data is read) and attaches a rising-edge interrupt on intPin.
  bool enableDataReadyInterrupt(int intPin);
  void disableDataReadyInterrupt();

  // True when new data is available. Uses the INT1 flag if the interrupt is
  // enabled, otherwise polls STATUS_REG over I2C.
  bool dataReady();

  // Burst-reads temperature, gyro and accel (14 bytes) in a single transaction.
  bool readRaw(IMURaw &out);
  bool read(IMUData &out);

  // Reads WHO_AM_I live from the last probed address. Useful when begin() fails.
  uint8_t whoAmI();
  const char *chipName() const;
  uint8_t address() const { return _addr; }

  bool readRegister(uint8_t reg, uint8_t &value);
  bool readRegisters(uint8_t reg, uint8_t *buf, size_t len);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool updateRegister(uint8_t reg, uint8_t mask, uint8_t value);

 private:
  // Internal steps of begin(); not meant to be called on their own.
  bool probe(uint8_t address);
  bool softReset();

  TwoWire *_wire = nullptr;  // I2C bus to use; nullptr = begin() not called yet
  uint8_t _addr = 0;         // I2C address of the chip (0x6A or 0x6B)
  uint8_t _whoAmI = 0;       // ID byte read during probe(), used by chipName()
  int _intPin = -1;          // GPIO wired to INT1; -1 = interrupt not in use
  float _accelScale = 0.0f;  // g per LSB (raw count), set by setAccelRange()
  float _gyroScale = 0.0f;   // dps per LSB, set by setGyroRange()
};
