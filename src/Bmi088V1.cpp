#include "Bmi088V1.h"

// Constructeur : initialise les adresses I2C
Bmi088V1::Bmi088V1(uint8_t accAddr, uint8_t gyrAddr) {
    _accAddr = accAddr;
    _gyrAddr = gyrAddr;
}

// Initialisation du capteur
bool Bmi088V1::begin() {
    // Activer l'accéléromètre (Power ON + Active mode)
    writeRegister(_accAddr, 0x7D, 0x04); 
    delay(50);
    writeRegister(_accAddr, 0x7C, 0x00); 
    delay(10);
    return true; 
}

// Lecture des registres via I2C (Méthode privée)
void Bmi088V1::readRegisters(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len; i++) {
        if (Wire.available()) {
            buf[i] = Wire.read();
        }
    }
}

// Écriture dans un registre (Méthode privée)
void Bmi088V1::writeRegister(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// Lecture et conversion de toutes les données IMU
IMUData Bmi088V1::readAll() {
    uint8_t buffer[6];
    IMUData data;

    // --- Lecture Accéléromètre (registre 0x12) ---
    readRegisters(_accAddr, 0x12, buffer, 6);
    data.ax = (int16_t)((buffer[1] << 8) | buffer[0]) * (6.0 / 32768.0);
    data.ay = (int16_t)((buffer[3] << 8) | buffer[2]) * (6.0 / 32768.0);
    data.az = (int16_t)((buffer[5] << 8) | buffer[4]) * (6.0 / 32768.0);

    // --- Lecture Gyroscope (registre 0x02) ---
    readRegisters(_gyrAddr, 0x02, buffer, 6);
    data.gx = (int16_t)((buffer[1] << 8) | buffer[0]) * (2000.0 / 32768.0);
    data.gy = (int16_t)((buffer[3] << 8) | buffer[2]) * (2000.0 / 32768.0);
    data.gz = (int16_t)((buffer[5] << 8) | buffer[4]) * (2000.0 / 32768.0);

    return data;
}

IMUPry Bmi088V1::calculatePRY() {
    IMUData data = readAll();

    // Calcul du pitch et du roll à partir des données de l'accéléromètre
    _pry.pitch = atan2(data.ay, sqrt(data.ax * data.ax + data.az * data.az)) * 180.0 / M_PI;
    _pry.roll = atan2(-data.ax, data.az) * 180.0 / M_PI;

    // Calcul du yaw à partir des données du gyroscope
    _pry.yaw = atan2(data.gz, sqrt(data.gx * data.gx + data.gy * data.gy)) * 180.0 / M_PI;

    return _pry;
}
