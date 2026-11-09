#include "EepromManager.h"
#include <Arduino.h>

void EepromManager::writeBytes(uint16_t addr, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(addr >> 8));    // Adresse haute
    Wire.write((uint8_t)(addr & 0xFF));  // Adresse basse
    for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
    Wire.endTransmission();
    delay(6); // Délai d'écriture EEPROM
}

void EepromManager::readBytes(uint16_t addr, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)EEPROM_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.available() ? Wire.read() : 0;
    }
}

bool EepromManager::save(float P, float Q, float Qbias, float R, float lock) {
    GainData g;
    g.magic = EEPROM_MAGIC;
    g.gain_P = P;
    g.gain_Q = Q;
    g.gain_Qbias = Qbias;
    g.gain_R = R;
    g.lock_sensitivity = lock;

    writeBytes(0x00, (uint8_t*)&g, sizeof(GainData));
    Serial.printf("[EEPROM] Gains sauvegardés : P=%.4f Q=%.5f Qbias=%.5f R=%.4f LOCK=%.3f\n", P, Q, Qbias, R, lock);
    return true;
}

bool EepromManager::load(float &P, float &Q, float &Qbias, float &R, float &lock) {
    GainData g;
    readBytes(0x00, (uint8_t*)&g, sizeof(GainData));

    if (g.magic != EEPROM_MAGIC) {
        Serial.println("[EEPROM] Aucune donnée valide, valeurs par défaut utilisées.");
        return false;
    }

    P    = g.gain_P;
    Q    = g.gain_Q;
    Qbias = g.gain_Qbias;
    R    = g.gain_R;
    lock = g.lock_sensitivity;
    Serial.printf("[EEPROM] Gains chargés : P=%.4f Q=%.5f Qbias=%.5f R=%.4f LOCK=%.3f\n", P, Q, Qbias, R, lock);
    return true;
}