#include "DataBase.h"

DataBase::DataBase() : _paramCount(0) {
    // Le tableau est prêt
}

// --- MÉTHODES DE LIAISON (LINK) ---

void DataBase::link(const char* key, float* var) {
    if (_paramCount < MAX_PARAMS) {
        _registry[_paramCount] = {key, (void*)var, TYPE_FLOAT};
        _paramCount++;
    } else {
        Serial.println("ERREUR DB: Capacité maximale atteinte !");
    }
}

void DataBase::link(const char* key, uint8_t* var) {
    if (_paramCount < MAX_PARAMS) {
        _registry[_paramCount] = {key, (void*)var, TYPE_UINT8};
        _paramCount++;
    } else {
        Serial.println("ERREUR DB: Capacité maximale atteinte !");
    }
}

void DataBase::link(const char* key, double* var) {
    if (_paramCount < MAX_PARAMS) {
        _registry[_paramCount] = {key, (void*)var, TYPE_DOUBLE};
        _paramCount++;
    } else {
        Serial.println("ERREUR DB: Capacité maximale atteinte !");
    }
}

// --- GESTION DES MESSAGES ENTRANTS ---

void DataBase::updateFromPayload(String payload) {
    int sep = payload.indexOf('=');
    if (sep == -1) return;

    String key = payload.substring(0, sep);
    String valStr = payload.substring(sep + 1);

    for (int i = 0; i < _paramCount; i++) {
        if (key == _registry[i].key) {
            if (_registry[i].type == TYPE_FLOAT) {
                *((float*)_registry[i].valuePtr) = valStr.toFloat();
                Serial.printf("DB UPDATE: %s (Float) -> %.3f\n", _registry[i].key, *((float*)_registry[i].valuePtr));
            } 
            else if (_registry[i].type == TYPE_UINT8) {
                *((uint8_t*)_registry[i].valuePtr) = (uint8_t)valStr.toInt();
                Serial.printf("DB UPDATE: %s (Uint8) -> %u\n", _registry[i].key, *((uint8_t*)_registry[i].valuePtr));
            }
            else if (_registry[i].type == TYPE_DOUBLE) {
                *((double*)_registry[i].valuePtr) = valStr.toDouble();
                Serial.printf("DB UPDATE: %s (Double) -> %.8f\n", _registry[i].key, *((double*)_registry[i].valuePtr));
            }
            return; 
        }
    }
    Serial.printf("DB WARN: Clé '%s' inconnue.\n", key.c_str());
}

// --- LECTURE DES VALEURS ---

String DataBase::getValueAsString(const char* key) {
    for (int i = 0; i < _paramCount; i++) {
        if (strcmp(key, _registry[i].key) == 0) {
            if (_registry[i].type == TYPE_FLOAT)  return String(*((float*)_registry[i].valuePtr), 3);
            if (_registry[i].type == TYPE_UINT8)  return String(*((uint8_t*)_registry[i].valuePtr));
            if (_registry[i].type == TYPE_DOUBLE) return String(*((double*)_registry[i].valuePtr), 8); // Précision GPS
        }
    }
    return "NOT_FOUND";
}

void DataBase::printAll() {
    Serial.println("--- Current DataBase Registry ---");
    for (int i = 0; i < _paramCount; i++) {
        Serial.printf("- %s: %s\n", _registry[i].key, getValueAsString(_registry[i].key).c_str());
    }
    Serial.println("---------------------------------");
}