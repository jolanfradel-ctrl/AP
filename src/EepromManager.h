#pragma once
#include <Wire.h>

#define EEPROM_ADDR 0x50  // AT24C32 classique, à adapter
#define EEPROM_MAGIC 0xAB // Signature pour détecter si l'EEPROM est initialisée

class EepromManager {
public:

    struct GainData {
    uint8_t magic;
    float gain_P;
    float gain_Q;
    float gain_Qbias;   // ← AJOUTÉ (juste après gain_Q)
    float gain_R;
    float lock_sensitivity;
    };
    bool save(float P, float Q, float Qbias, float R, float lock);
    bool load(float &P, float &Q, float &Qbias, float &R, float &lock);

private:
    void writeBytes(uint16_t addr, uint8_t* data, uint8_t len);
    void readBytes(uint16_t addr, uint8_t* data, uint8_t len);
};