#include <Arduino.h>
#include <Wire.h>

#include "IMU.h"

// Pin definitions
#define PIN_SDA  12
#define PIN_SCL  13
#define PIN_INT1 11

void setup() {
  Serial.begin(115200);
  delay(2000);  // allow the native USB port to enumerate
  Serial.println("ESP32-S3 alive");
  Serial.printf("Flash size: %lu bytes\n", (unsigned long)ESP.getFlashChipSize());

  Wire.begin(PIN_SDA, PIN_SCL);
  Serial.println(IMU::begin(PIN_INT1) ? "IMU ready" : "IMU init failed");
}

void loop() {
  IMU::update();

  // Send 'z' over serial to make the current direction heading 0.
  if (Serial.available() && Serial.read() == 'z') {
    IMU::zeroHeading();
    Serial.println("Heading zeroed");
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;  // print at 10 Hz
  lastPrint = millis();

  float gx, gy, gz;
  IMU::rates(gx, gy, gz);
  Serial.printf("Hdg %7.2f  Pitch %7.2f  Roll %7.2f | Rate[dps] %7.1f %7.1f %7.1f\n",
                IMU::heading(), IMU::pitch(), IMU::roll(), gx, gy, gz);
}
