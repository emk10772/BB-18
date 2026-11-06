# The stm32duino LSM6DSV16BX library relies on the STM32 core including
# Arduino.h implicitly; on the ESP32 core its .cpp misses MSBFIRST and Serial.
# Force-include Arduino.h for C++ files only (the library's C sources don't need it).
Import("env")
env.Append(CXXFLAGS=["-include", "Arduino.h"])
