#include <Arduino.h>
#include <Wire.h>

// Pin definitions
#define PIN_SDA  12
#define PIN_SCL  13
#define PIN_INT1 11

void setup() {
  Serial.begin(115200);
  delay(2000);  // allow the native USB port to enumerate
  Serial.println("ESP32-S3 alive");
  Serial.printf("Flash size: %lu bytes\n", (unsigned long)ESP.getFlashChipSize());
}

void loop() {
  
}
