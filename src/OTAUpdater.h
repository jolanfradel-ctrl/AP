#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <Update.h>
#include <TinyGsmClient.h>
#include "IFirmwareSource.h"
#include "esp_task_wdt.h"

class OTAUpdater {
private:
    IFirmwareSource& _source;
    TinyGsm* _modem = nullptr;
    HardwareSerial* _serialAT = nullptr;

public:
    OTAUpdater(IFirmwareSource& source) : _source(source) {}

    void attachModem(TinyGsm& modem) { _modem = &modem; }
    void attachSerial(HardwareSerial& serialAT) { _serialAT = &serialAT; } 

    // Attend la confirmation réelle de détachement réseau avant de couper quoi que ce soit
    void waitForDetachConfirmation(HardwareSerial& serialAT, uint32_t timeout = 15000) {
        unsigned long t0 = millis();
        String buf = "";
        while (millis() - t0 < timeout) {
            while (serialAT.available()) {
                buf += (char)serialAT.read();
            }
            if (buf.indexOf("DETACH") != -1) {
                Serial.println(">>> Détachement réseau confirmé.");
                delay(200); // laisse passer le reste de la ligne
                return;
            }
            esp_task_wdt_reset();
        }
        Serial.println(">>> Timeout confirmation détachement (on continue quand même).");
    }
    void gracefulModemShutdown() {
        if (!_modem || !_serialAT) {
            Serial.println(">>> OTA: pas de modem attaché, skip shutdown propre.");
            return;
        }

        Serial.println(">>> OTA: détachement GPRS...");
        _modem->gprsDisconnect();
        delay(500);
        esp_task_wdt_reset();

        Serial.println(">>> OTA: envoi AT+CPOF...");
        _serialAT->println("AT+CPOF");

        unsigned long t0 = millis();
        String buf = "";
        while (millis() - t0 < 8000) {
            while (_serialAT->available()) {
                buf += (char)_serialAT->read();
            }
            if (buf.indexOf("DETACH") != -1 || buf.indexOf("OK") != -1) {
                break;
            }
            esp_task_wdt_reset();
        }
        Serial.println(">>> OTA: modem éteint proprement.");
        delay(1000);
    }

    bool run(const String& url) {
        Serial.println(">>> OTA: connexion à la source...");
        if (!_source.begin(url)) {
            Serial.println(">>> OTA: échec d'ouverture de la source.");
            return false;
        }

        int size = _source.totalSize();
        Serial.printf(">>> OTA: taille annoncée = %d octets\n", size);

        if (!Update.begin(size > 0 ? size : UPDATE_SIZE_UNKNOWN)) {
            Serial.println(">>> OTA: Update.begin() a échoué.");
            Update.printError(Serial);
            _source.end();
            return false;
        }

        uint8_t buffer[512];
        size_t totalWritten = 0;
        unsigned long lastData = millis();
        const unsigned long stallTimeout = 15000;

        while (true) {
            esp_task_wdt_reset();

            // --- Arrêt propre si on a déjà tout reçu (taille connue) ---
            if (size > 0 && totalWritten >= (size_t)size) {
                Serial.println("\n>>> OTA: taille annoncée atteinte, fin du téléchargement.");
                break;
            }

            int n = _source.readChunk(buffer, sizeof(buffer));

            if (n > 0) {
                Update.write(buffer, n);
                totalWritten += n;
                lastData = millis();
                Serial.printf("\r>>> OTA: %u octets écrits", (unsigned)totalWritten);
            } else if (n == 0) {
                if (millis() - lastData > stallTimeout) {
                    Serial.println("\n>>> OTA: timeout, aucune donnée reçue.");
                    Update.abort();
                    _source.end();
                    return false;
                }
                delay(20);
            } else {
                break; // n == -1 : terminé côté source
            }
        }

        _source.end();
        Serial.println();

        if (!Update.end(true)) {
            Serial.println(">>> OTA: Update.end() a échoué.");
            Update.printError(Serial);
            return false;
        }

        if (!Update.isFinished()) {
            Serial.println(">>> OTA: update incomplet.");
            return false;
        }
        Serial.println(">>> OTA: succès");

        

        delay(15000);
        Serial.println(">>> OTA: succès, déconnexion propre avant redémarrage...");

        gracefulModemShutdown();

        Serial.println(">>> OTA: redémarrage ESP32...");
        delay(500);
        ESP.restart();
        return true;
    }
};

#endif