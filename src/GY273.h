#ifndef GY273_H
#define GY273_H

#include <Arduino.h>
#include <Wire.h>

struct MagData {
    int16_t raw_x, raw_y, raw_z;   // valeurs brutes du capteur
    float x, y, z;                 // en milligauss, échelle appliquée
    float heading;                 // cap en degrés (0-360°), corrigé de la déclinaison
    bool dataReady;                // registre de statut : nouvelle mesure dispo
};

class GY273 {
public:
    explicit GY273(uint8_t addr = 0x1E); // 0x1E = adresse par défaut du HMC5883L

    bool begin();

    // Lecture complète : brut + converti + cap calculé
    MagData readAll();

    // Change le gain (plage de mesure). Valeurs valides : 0 à 7 (cf. datasheet HMC5883L)
    // 0 = ±0.88 Ga (le plus sensible) ... 7 = ±8.1 Ga (le moins sensible)
    void setGain(uint8_t gainSetting);

    // Déclinaison magnétique locale en degrés, pour corriger le cap vers le nord vrai
    // (positive à l'Est, négative à l'Ouest — cherche la valeur pour ta position)
    void setDeclination(float declinationDeg);

    // Vérifie que le capteur répond bien (lit les registres d'identification)
    // Doit renvoyer "H43" si le capteur est bien un HMC5883L
    String getDeviceID();

    // Lit uniquement le registre de statut (bit 0 = nouvelle donnée prête)
    bool isDataReady();

    float getTiltCompensatedHeading(const MagData& mag, float rollDeg, float pitchDeg);


private:
    uint8_t _addr;
    float _scale;          // mGauss par LSB, dépend du gain
    float _declination;    // en degrés

    void readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t data);
};

#endif