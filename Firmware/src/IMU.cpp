#include "IMU.h"

namespace {
/*
Acronym Key:
  -OP_MODE_XL[6:4] = accelerometer operating mode bits in CTRL1
  -OP_MODE_G[6:4] = gyroscope operating mode bits in CTRL2
  -FS_XL[1:0] = accelerometer full-scale range bits in CTRL8
  -FS_G[3:0] = gyroscope full-scale range bits in CTRL6
  -ODR_XL[3:0] = accelerometer output data rate bits in CTRL1
  -ODR_G[3:0] = gyroscope output data rate bits in CTRL2
  -LPF1_G_BW[6:4] = gyroscope low-pass filter 1 bandwidth bits in CTRL6
  -HP_LPF2_XL_BW[7:5] = accelerometer high-pass filter / low-pass filter 2 bandwidth bits in CTRL8
  -DRDY_PULSED = data-ready interrupt pulse mode bit in CTRL4
  -BDU = block data update bit in CTRL3
  -IF_INC = auto-increment address on burst reads bit in CTRL3
  -SW_RESET = software reset bit in CTRL3
  -XLDA = accelerometer data-ready bit in STATUS_REG
  -GDA = gyroscope data-ready bit in STATUS_REG
  -INT1_DRDY_XL = accelerometer data-ready routed to INT1 bit in INT1_CTRL      
*/
// LSM6DSV16BX register map (subset)
constexpr uint8_t REG_INT1_CTRL  = 0x0D;
constexpr uint8_t REG_WHO_AM_I   = 0x0F;
constexpr uint8_t REG_CTRL1      = 0x10;  // OP_MODE_XL[6:4], ODR_XL[3:0]
constexpr uint8_t REG_CTRL2      = 0x11;  // OP_MODE_G[6:4],  ODR_G[3:0]
constexpr uint8_t REG_CTRL3      = 0x12;  // BOOT, BDU, IF_INC, SW_RESET
constexpr uint8_t REG_CTRL6      = 0x15;  // LPF1_G_BW[6:4], FS_G[3:0]
constexpr uint8_t REG_CTRL8      = 0x17;  // HP_LPF2_XL_BW[7:5], FS_XL[1:0]
constexpr uint8_t REG_STATUS     = 0x1E;
constexpr uint8_t REG_OUT_TEMP_L = 0x20;  // followed by gyro X/Y/Z, then accel X/Y/Z

constexpr uint8_t WHO_AM_I_LSM6DSV16BX = 0x71;
constexpr uint8_t WHO_AM_I_LSM6DSV16X  = 0x70;  // LSM6DSV / LSM6DSV16X, same core registers

// CTRL3 bits
constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_IF_INC   = 0x04;  // auto-increment address on burst reads
constexpr uint8_t CTRL3_BDU      = 0x40;  // block data update: no torn high/low bytes

// STATUS_REG bits
constexpr uint8_t STATUS_XLDA = 0x01;
constexpr uint8_t STATUS_GDA  = 0x02;

// INT1_CTRL bits
constexpr uint8_t INT1_DRDY_XL = 0x01;

constexpr float TEMP_LSB_PER_C = 256.0f;

volatile bool s_dataReadyFlag = false;

void IRAM_ATTR onImuInt1() { s_dataReadyFlag = true; }

}  // namespace

bool IMU::begin(TwoWire &wire, uint8_t address) {
  _wire = &wire;

  //If address is provided, probe that address. If no address is provided, probe both possible addresses and use the one that responds.
  if (address != 0) {
    if (!probe(address)) return false;
  } else if (!probe(ADDR_SA0_LOW) && !probe(ADDR_SA0_HIGH)) {
    return false;
  }

  // Reset the chip and configure it with a default config: ±4 g, ±500 dps, 120 Hz.
  if (!softReset()) return false;
  if (!writeRegister(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC)) return false;

  // Default config: ±4 g, ±500 dps, 120 Hz
  return setAccelRange(ACCEL_4G) && setGyroRange(GYRO_500DPS) &&
         setAccelDataRate(ODR_120HZ) && setGyroDataRate(ODR_120HZ);
}

bool IMU::probe(uint8_t address) {
  _addr = address;
  uint8_t id = 0;

  //Read the WHO_AM_I register and check if it matches the expected value for the LSM6DSV16BX or LSM6DSV16X. 
  //If it does, store the ID and return true. Otherwise, return false.
  if (!readRegister(REG_WHO_AM_I, id)) return false;
  if (id != WHO_AM_I_LSM6DSV16BX && id != WHO_AM_I_LSM6DSV16X) return false;
  _whoAmI = id;
  return true;
}

bool IMU::softReset() {
  // Power down both sensors before resetting, as ST recommends.
  if (!writeRegister(REG_CTRL1, 0x00) || !writeRegister(REG_CTRL2, 0x00)) return false;
  if (!updateRegister(REG_CTRL3, CTRL3_SW_RESET, CTRL3_SW_RESET)) return false;

  // SW_RESET self-clears when the reset completes.
  for (int i = 0; i < 10; i++) {
    delay(1);
    uint8_t ctrl3 = 0;
    if (readRegister(REG_CTRL3, ctrl3) && !(ctrl3 & CTRL3_SW_RESET)) return true;
  }
  return false;
}

//In the set___Range functions we are updating the IMU's registers to match the scale factor we chose
//Wee are also updating the code's internal variables to match the scale factor as well (in the actual units)

bool IMU::setAccelRange(AccelRange range) {
  if (!updateRegister(REG_CTRL8, 0x03, range)) return false;
  switch (range) {
    case ACCEL_2G:  _accelScale = 0.061e-3f; break;
    case ACCEL_4G:  _accelScale = 0.122e-3f; break;
    case ACCEL_8G:  _accelScale = 0.244e-3f; break;
    case ACCEL_16G: _accelScale = 0.488e-3f; break;
  }
  return true;
}

bool IMU::setGyroRange(GyroRange range) {
  float scale;
  switch (range) {
    case GYRO_125DPS:  scale = 4.375e-3f; break;
    case GYRO_250DPS:  scale = 8.75e-3f;  break;
    case GYRO_500DPS:  scale = 17.5e-3f;  break;
    case GYRO_1000DPS: scale = 35.0e-3f;  break;
    case GYRO_2000DPS: scale = 70.0e-3f;  break;
    case GYRO_4000DPS: scale = 140.0e-3f; break;
    default: return false;
  }
  if (!updateRegister(REG_CTRL6, 0x0F, range)) return false;
  _gyroScale = scale;
  return true;
}

bool IMU::setAccelDataRate(DataRate odr) {
  // OP_MODE_XL is left at 000 (high-performance).
  return updateRegister(REG_CTRL1, 0x0F, odr);
}

bool IMU::setGyroDataRate(DataRate odr) {
  // OP_MODE_G is left at 000 (high-performance).
  return updateRegister(REG_CTRL2, 0x0F, odr);
}

//Configures the IMU to interupt when new data is ready
//Also sets up ESP32 to listen for that interrupt and set a flag when it happens
bool IMU::enableDataReadyInterrupt(int intPin) {
  // INT1 is push-pull, active-high by default, so the MCU pin needs no pull-up.
  // Data-ready is latched (CTRL4 DRDY_PULSED = 0): INT1 stays high until the
  // output registers are read.
  if (!updateRegister(REG_INT1_CTRL, INT1_DRDY_XL, INT1_DRDY_XL)) return false;

  _intPin = intPin;
  pinMode(_intPin, INPUT);
  s_dataReadyFlag = digitalRead(_intPin) == HIGH;  // catch a sample that's already waiting
  attachInterrupt(digitalPinToInterrupt(_intPin), onImuInt1, RISING);
  return true;
}

//Resets all the interupt setup on the IMU and detaches the ESP32 interrupt
void IMU::disableDataReadyInterrupt() {
  if (_intPin < 0) return;
  detachInterrupt(digitalPinToInterrupt(_intPin));
  updateRegister(REG_INT1_CTRL, INT1_DRDY_XL, 0);
  _intPin = -1;
  s_dataReadyFlag = false;
}


bool IMU::dataReady() {
  if (_intPin >= 0) {
    if (!s_dataReadyFlag) return false;
    s_dataReadyFlag = false;
    return true;
  }
  uint8_t status = 0;
  if (!readRegister(REG_STATUS, status)) return false;
  return (status & (STATUS_XLDA | STATUS_GDA)) != 0;
}

//Checks if the IMU has new data ready, and if it does, 
//reads all the data in one burst read and returns it in the out parameter
bool IMU::readRaw(IMURaw &out) {
  uint8_t b[14];
  if (!readRegisters(REG_OUT_TEMP_L, b, sizeof(b))) return false;

  auto s16 = [&](int i) { return (int16_t)((uint16_t)b[i + 1] << 8 | b[i]); };
  out.temp = s16(0);
  out.gx = s16(2);
  out.gy = s16(4);
  out.gz = s16(6);
  out.ax = s16(8);
  out.ay = s16(10);
  out.az = s16(12);
  return true;
}

//Reads the raw data from the IMU and converts it to actual 
//units using the scale factors set by the set___Range functions
bool IMU::read(IMUData &out) {
  IMURaw raw;
  if (!readRaw(raw)) return false;

  out.ax = raw.ax * _accelScale;
  out.ay = raw.ay * _accelScale;
  out.az = raw.az * _accelScale;
  out.gx = raw.gx * _gyroScale;
  out.gy = raw.gy * _gyroScale;
  out.gz = raw.gz * _gyroScale;
  out.tempC = 25.0f + raw.temp / TEMP_LSB_PER_C;
  return true;
}


//Gets the WHO_AM_I register from the IMU and returns it
uint8_t IMU::whoAmI() {
  uint8_t id = 0;
  readRegister(REG_WHO_AM_I, id);
  return id;
}


//Checks the WHO_AM_I register and returns a string with the chip name
const char *IMU::chipName() const {
  switch (_whoAmI) {
    case WHO_AM_I_LSM6DSV16BX: return "LSM6DSV16BX";
    case WHO_AM_I_LSM6DSV16X:  return "LSM6DSV16X";
    default:                   return "unknown";
  }
}


//Reads a single register from the IMU and returns it in value
//Note that it calls readRegisters with a length of 1, so it is a wrapper function for that
bool IMU::readRegister(uint8_t reg, uint8_t &value) {
  return readRegisters(reg, &value, 1);
}

bool IMU::readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
  //Start a message to the IMU (at address _addr) and add the register we want to read from
  _wire->beginTransmission(_addr);
  _wire->write(reg);

  //Tells the IMU to not pass STOP and to keep the I2C bus open for read
  if (_wire->endTransmission(false) != 0) return false;  // repeated start

  //If the IMU doesn't respond with the expected number of bytes, return false
  if (_wire->requestFrom(_addr, (uint8_t)len) != len) return false;

  //Read the bytes from the IMU into the buffer
  for (size_t i = 0; i < len; i++) buf[i] = _wire->read();
  return true;
}

bool IMU::writeRegister(uint8_t reg, uint8_t value) {
  //Start a message to the IMU (at address _addr) and add the register we want to write to
  //Then add the value we want to write to that register and send the message
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

bool IMU::updateRegister(uint8_t reg, uint8_t mask, uint8_t value) {
  //Get the current value of the register
  uint8_t current = 0;
  if (!readRegister(reg, current)) return false;

  //Mask off the bits we want to update and OR in the new value, then write it back to the register
  return writeRegister(reg, (current & ~mask) | (value & mask));
}
