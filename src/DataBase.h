#ifndef DATABASE_H
#define DATABASE_H

#include <Arduino.h>

// Types de données supportés
enum ParamType { TYPE_FLOAT, TYPE_UINT8 ,TYPE_DOUBLE};

// Structure interne pour stocker le lien entre une clé et une adresse mémoire
struct Parameter {
    const char* key;
    void* valuePtr;
    ParamType type;
};

class DataBase {
private:
    static const int MAX_PARAMS = 100; // Taille max du tableau (blindé)
    Parameter _registry[MAX_PARAMS];
    int _paramCount;

public:
    DataBase();

    // Lier un float (ex: gain PID)
    void link(const char* key, float* var);

    // Lier un entier 8 bits (ex: mode, état, switch)
    void link(const char* key, uint8_t* var);

    // Lier un double (ex: coordonnées GPS)
    void link(const char* key, double* var);

    // Analyse et mise à jour depuis un message "CLE=VALEUR"
    void updateFromPayload(String payload);

    // (Optionnel) Pour débug : affiche tout ce qui est dans la base
    void printAll();

    String getValueAsString(const char* key);
};

#endif