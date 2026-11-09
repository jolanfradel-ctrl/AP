#include "GY273.h"

// Registres HMC5883L
#define REG_CONFIG_A   0x00
#define REG_CONFIG_B   0x01
#define REG_MODE       0x02
#define REG_DATA_X_MSB 0x03
#define REG_STATUS     0x09
#define REG_ID_A       0x0A

// Table gain -> échelle (mGauss/LSB), cf. datasheet HMC5883L table 3
static const float GAIN_SCALE[8] = {
    0.73f, 0.92f, 1.22f, 1.52f, 2.27f, 2.56f, 3.03f, 4.35f
};

GY273::GY273(uint8_t addr) {
    _addr = addr;
    _scale = GAIN_SCALE[1]; // gain par défaut du HMC5883L au power-on
    _declination = 0.0f;
}

bool GY273::begin() {
    // Config A : 8 échantillons moyennés, 15Hz, mode normal
    writeRegister(REG_CONFIG_A, 0x70);
    delay(10);

    // Config B : gain par défaut (setting 1)
    setGain(1);
    delay(10);

    // Mode continu : le capteur mesure en boucle sans qu'on ait à le redéclencher
    writeRegister(REG_MODE, 0x00);
    delay(10);

    // Vérifie que le capteur répond correctement
    return getDeviceID() == "H43";
}

void GY273::setGain(uint8_t gainSetting) {
    if (gainSetting > 7) gainSetting = 7;
    writeRegister(REG_CONFIG_B, gainSetting << 5);
    _scale = GAIN_SCALE[gainSetting];
}

void GY273::setDeclination(float declinationDeg) {
    _declination = declinationDeg;
}

void GY273::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, len);
    for (uint8_t i = 0; i < len; i++) {
        if (Wire.available()) {
            buf[i] = Wire.read();
        }
    }
}

void GY273::writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

bool GY273::isDataReady() {
    uint8_t status;
    readRegisters(REG_STATUS, &status, 1);
    return (status & 0x01) != 0;
}

String GY273::getDeviceID() {
    uint8_t idBuf[3];
    readRegisters(REG_ID_A, idBuf, 3);
    char id[4];
    id[0] = (char)idBuf[0];
    id[1] = (char)idBuf[1];
    id[2] = (char)idBuf[2];
    id[3] = '\0';
    return String(id);
}

MagData GY273::readAll() {
    uint8_t buffer[6];
    MagData data;

    data.dataReady = isDataReady();

    // Important : le HMC5883L renvoie les axes dans l'ordre X, Z, Y (pas X,Y,Z !)
    readRegisters(REG_DATA_X_MSB, buffer, 6);

    data.raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.raw_z = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.raw_y = (int16_t)((buffer[4] << 8) | buffer[5]);

    data.x = data.raw_x * _scale;
    data.y = data.raw_y * _scale;
    data.z = data.raw_z * _scale;

    // Cap magnétique (dans le plan horizontal X/Y), corrigé de la déclinaison locale
    float heading = atan2(data.y, data.x) * 180.0 / M_PI;
    heading += _declination;
    if (heading < 0) heading += 360.0;
    if (heading >= 360.0) heading -= 360.0;
    data.heading = heading;

    return data;
}

float GY273::getTiltCompensatedHeading(const MagData& mag, float rollDeg, float pitchDeg) {
    float roll  = rollDeg  * M_PI / 180.0f;
    float pitch = pitchDeg * M_PI / 180.0f;

    // Reprojection du champ magnétique dans le plan horizontal réel
    float Xh = mag.x * cos(pitch) + mag.z * sin(pitch);
    float Yh = mag.x * sin(roll) * sin(pitch)
             + mag.y * cos(roll)
             - mag.z * sin(roll) * cos(pitch);

    float heading = atan2(Yh, Xh) * 180.0 / M_PI;
    heading += _declination;

    if (heading < 0)     heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;

    return heading;
}