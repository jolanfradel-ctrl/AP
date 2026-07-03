#ifndef BMI088V1_H
#define BMI088V1_H

#include <Arduino.h>
#include <Wire.h>

struct IMUData {
    float ax, ay, az, gx, gy, gz;
};

struct IMUPry {
    float pitch, roll, yaw;
};

class Bmi088V1 { // <-- Vérifie ce nom précisément
private:
    uint8_t _accAddr;
    uint8_t _gyrAddr;
    void writeRegister(uint8_t addr, uint8_t reg, uint8_t data);
    void readRegisters(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);

public:
    IMUPry _pry; 
    Bmi088V1(uint8_t accAddr = 0x19, uint8_t gyrAddr = 0x69);
    bool begin();
    IMUData readAll();
    IMUPry  calculatePRY();
};

#endif