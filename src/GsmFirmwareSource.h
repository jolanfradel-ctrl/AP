#ifndef GSM_FIRMWARE_SOURCE_H
#define GSM_FIRMWARE_SOURCE_H

#include "IFirmwareSource.h"
#include "esp_task_wdt.h"

class GsmFirmwareSource : public IFirmwareSource {
private:
    HardwareSerial& _serial;
    int _totalSize = -1;
    int _offset = 0;
    static const size_t READ_CHUNK = 512;

    // --- Helper générique, identique à celui du firmware de test ---
    String sendATSimple(const String& cmd, uint32_t timeout = 8000) {
        while (_serial.available()) _serial.read();
        _serial.println(cmd);

        unsigned long t0 = millis();
        String resp = "";
        while (millis() - t0 < timeout) {
            while (_serial.available()) resp += (char)_serial.read();
            if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) {
                delay(100);
                while (_serial.available()) resp += (char)_serial.read();
                break;
            }
            esp_task_wdt_reset();
        }
        return resp;
    }

    // --- Attend le vrai résultat asynchrone de AT+HTTPACTION ---
    String waitForHttpAction(uint32_t timeout) {
        unsigned long t0 = millis();
        String resp = "";
        while (millis() - t0 < timeout) {
            while (_serial.available()) resp += (char)_serial.read();
            if (resp.indexOf("+HTTPACTION:") != -1) {
                delay(200);
                while (_serial.available()) resp += (char)_serial.read();
                break;
            }
            esp_task_wdt_reset();
        }
        return resp;
    }

public:
    GsmFirmwareSource(HardwareSerial& serial) : _serial(serial) {}

    bool begin(const String& url) override {
        Serial.println(">>> GSM OTA: init HTTP...");
        sendATSimple("AT+HTTPINIT", 5000);
        sendATSimple("AT+CSSLCFG=\"sslversion\",0,4");
        sendATSimple("AT+CSSLCFG=\"authmode\",0,1");
        sendATSimple("AT+CSSLCFG=\"cacert\",0,\"isrgrootx1.pem\"");
        sendATSimple("AT+CSSLCFG=\"enableSNI\",0,1");
        sendATSimple("AT+HTTPPARA=\"URL\",\"" + url + "\"");
        sendATSimple("AT+HTTPPARA=\"SSLCFG\",0");

        while (_serial.available()) _serial.read();
        _serial.println("AT+HTTPACTION=0");
        delay(100);
        while (_serial.available()) _serial.read(); // jette le "OK" immédiat, sans intérêt

        Serial.println(">>> GSM OTA: attente du résultat HTTPACTION...");
        String resp = waitForHttpAction(20000);

        int idx = resp.indexOf("+HTTPACTION:");
        if (idx == -1) {
            Serial.println(">>> GSM OTA: pas de résultat HTTPACTION reçu.");
            return false;
        }

        int firstComma  = resp.indexOf(',', idx);
        int secondComma = resp.indexOf(',', firstComma + 1);
        int lineEnd     = resp.indexOf('\r', secondComma);
        if (lineEnd == -1) lineEnd = resp.indexOf('\n', secondComma);
        if (lineEnd == -1) lineEnd = resp.length();

        int statusCode = resp.substring(firstComma + 1, secondComma).toInt();
        int dataLen    = resp.substring(secondComma + 1, lineEnd).toInt();

        Serial.printf(">>> GSM OTA: statuscode=%d, taille=%d\n", statusCode, dataLen);

        if (statusCode != 200) {
            Serial.println(">>> GSM OTA: code HTTP inattendu.");
            return false;
        }

        _totalSize = dataLen;
        _offset = 0;
        return true;
    }

    int totalSize() override {
        return _totalSize;
    }

    int readChunk(uint8_t* buffer, size_t len) override {
        if (_totalSize > 0 && _offset >= _totalSize) return -1;

        size_t toRead = min(len, READ_CHUNK);
        if (_totalSize > 0) {
            toRead = min(toRead, (size_t)(_totalSize - _offset));
        }
        if (toRead == 0) return -1;

        while (_serial.available()) _serial.read();
        String cmd = "AT+HTTPREAD=" + String(_offset) + "," + String(toRead);
        _serial.println(cmd);

        // --- Attend l'en-tête "+HTTPREAD: <n>" avant les données brutes ---
        unsigned long t0 = millis();
        String header = "";
        int n = -1;

        while (millis() - t0 < 10000) {
            while (_serial.available()) {
                header += (char)_serial.read();
                int hdrIdx = header.indexOf("+HTTPREAD:");
                if (hdrIdx != -1) {
                    int nlIdx = header.indexOf('\n', hdrIdx);
                    if (nlIdx != -1) {
                        String numStr = header.substring(hdrIdx + 10, nlIdx);
                        numStr.trim();
                        n = numStr.toInt();
                        t0 = 0; // sort de la boucle externe
                        break;
                    }
                }
            }
            if (t0 == 0) break;
            esp_task_wdt_reset();
        }

        if (n < 0) {
            Serial.println("[GSM OTA] Timeout en-tête HTTPREAD");
            return 0; // OTAUpdater retentera au tour suivant
        }
        if (n == 0) {
            _totalSize = _offset; // confirme la fin réelle du transfert
            return -1;
        }

        // --- Lit exactement n octets bruts qui suivent l'en-tête ---
        size_t received = 0;
        unsigned long t1 = millis();
        while (received < (size_t)n && millis() - t1 < 10000) {
            while (_serial.available() && received < (size_t)n) {
                buffer[received++] = (uint8_t)_serial.read();
            }
            esp_task_wdt_reset();
        }

        // --- Purge ce qu'il reste (marqueur de fin, OK...) avant le prochain appel ---
        delay(50);
        unsigned long purgeStart = millis();
        while (millis() - purgeStart < 300) {
            while (_serial.available()) _serial.read();
        }

        _offset += received;
        return received;
    }

    void end() override {
        sendATSimple("AT+HTTPTERM", 5000);
    }
};

#endif