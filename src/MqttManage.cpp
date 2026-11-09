#include "MqttManage.h"
#include "DataBase.h"
#include "esp_task_wdt.h"

MqttManager::MqttManager(TinyGsm& modem, const char* server, const char* user, const char* pass)
    : _modem(modem), _server(server), _user(user), _pass(pass) {
}

void MqttManager::setup(MqttCallback callback) {
    // On ne passe plus par _modem.mqtt_set_callback() : la réception ne fonctionne
    // donc plus pour l'instant (voir note à la fin de la réponse). Le pointeur
    // est conservé pour ne rien casser côté main.cpp.
    _callback = callback;
}

// --- Reprise exacte du helper "sendAT" du firmware de test ---
String MqttManager::sendAT(const String& cmd, uint32_t timeout) {
    while (Serial1.available()) Serial1.read();
    Serial.print(">> "); Serial.println(cmd);
    Serial1.println(cmd);

    unsigned long t0 = millis();
    String resp = "";
    while (millis() - t0 < timeout) {
        while (Serial1.available()) resp += (char)Serial1.read();
        if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) {
            delay(100);
            while (Serial1.available()) resp += (char)Serial1.read();
            break;
        }
        esp_task_wdt_reset();
    }
    Serial.print("<< "); Serial.println(resp);
    Serial.println("---");
    return resp;
}

// --- Reprise exacte du "publishAT" du firmware de test (3 étapes) ---
bool MqttManager::publishAT(const String& topic, const String& payload, uint32_t timeout) {
    while (Serial1.available()) _rxBuffer += (char)Serial1.read();
    String topicCmd = "AT+CMQTTTOPIC=0," + String(topic.length());
    Serial1.println(topicCmd);

    unsigned long t0 = millis();
    String resp = "";
    bool gotPrompt = false;
    while (millis() - t0 < timeout) {
        while (Serial1.available()) {
            char c = (char)Serial1.read();
            resp += c;
            if (c == '>') { gotPrompt = true; break; }
        }
        if (gotPrompt || resp.indexOf("ERROR") != -1) break;
    }
    if (!gotPrompt) {
        Serial.println("!! MQTT: pas de prompt pour TOPIC");
        Serial.print("!! Contenu brut recu: [");
        Serial.print(resp);
        Serial.println("]");

        _consecutiveFailures++;
        checkFailureThreshold();
        return false;
    }
    Serial1.print(topic);
    delay(200);
    while (Serial1.available()) _rxBuffer += (char)Serial1.read();

    String payloadCmd = "AT+CMQTTPAYLOAD=0," + String(payload.length());
    Serial1.println(payloadCmd);

    t0 = millis();
    resp = "";
    gotPrompt = false;
    while (millis() - t0 < timeout) {
        while (Serial1.available()) {
            char c = (char)Serial1.read();
            resp += c;
            if (c == '>') { gotPrompt = true; break; }
        }
        if (gotPrompt || resp.indexOf("ERROR") != -1) break;
    }
    if (!gotPrompt) {
        Serial.println("!! MQTT: pas de prompt pour PAYLOAD");
        Serial.print("!! Contenu brut recu: [");
        Serial.print(resp);
        Serial.println("]");

        _consecutiveFailures++;
        checkFailureThreshold();
        return false;
    }

    Serial1.print(payload);
    delay(200);
    while (Serial1.available()) Serial1.read();

    Serial1.println("AT+CMQTTPUB=0,0,60,0,0");
    t0 = millis();
    resp = "";
    while (millis() - t0 < timeout) {
        while (Serial1.available()) resp += (char)Serial1.read();
        if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) {
            delay(100);
            while (Serial1.available()) resp += (char)Serial1.read();
            break;
        }
    }

    bool ok = resp.indexOf("OK") != -1;
    if (ok) {
        _consecutiveFailures = 0; // vrai succès, on remet le compteur à zéro
    } else {
        _consecutiveFailures++;
        checkFailureThreshold();
    }
    return ok;
}

bool MqttManager::connect() {
    if (_connected) return true;

    static unsigned long lastAttempt = 0;
    const unsigned long retryInterval = 5000;
    if (millis() - lastAttempt < retryInterval) return false;
    lastAttempt = millis();

    if (_clientId.length() == 0) {
        _clientId = "ESP32_Jolan_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    }

    // --- Nettoyage systématique avant de retenter ---
    // (idempotent : si rien n'était démarré, ces commandes échouent silencieusement, pas grave)
    Serial.println(">>> MQTT: nettoyage état précédent...");
    sendAT("AT+CMQTTDISC=0,60", 3000);
    sendAT("AT+CMQTTREL=0", 3000);
    sendAT("AT+CMQTTSTOP", 3000);
    delay(300);


    Serial.println(">>> MQTT: sequence AT (identique au firmware de test)...");
    sendAT("AT+CSSLCFG=\"sslversion\",0,4");
    sendAT("AT+CSSLCFG=\"authmode\",0,1");
    sendAT("AT+CSSLCFG=\"cacert\",0,\"isrgrootx1.pem\"");
    sendAT("AT+CSSLCFG=\"enableSNI\",0,1");
    sendAT("AT+CMQTTSTART", 5000);
    sendAT("AT+CMQTTACCQ=0,\"" + _clientId + "\",1");
    sendAT("AT+CMQTTSSLCFG=0,0");

    String connectCmd = "AT+CMQTTCONNECT=0,\"tcp://" + String(_server) + ":8883\",60,1,\"" +
                         String(_user) + "\",\"" + String(_pass) + "\"";
    String resp = sendAT(connectCmd, 15000);

    _connected = (resp.indexOf("+CMQTTCONNECT: 0,0") != -1);
    Serial.println(_connected ? ">>> MQTT connecté !" : ">>> MQTT: échec de connexion.");
        if (_connected) {
        subscribeAT(_subTopic);
        subscribeAT(_otaTopic);
    }
    return _connected;
}




void MqttManager::loop() {
    // Rien à faire : le modem A7670 gère les pings lui-même (inchangé).
}

bool MqttManager::connected() {
    return _connected;
}

bool MqttManager::publishData(float lat, float lng, const IMUData& imu, bool hasFix) {
    if (!_connected) return false;

    char msg[350];
    snprintf(msg, sizeof(msg),
        "{\"lat\":%.6f,\"lng\":%.6f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"fix\":%s}",
        lat, lng, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, hasFix ? "true" : "false");

    return publishAT("esp32/gps/data", String(msg));
}

void MqttManager::publishParameter(const char* key, DataBase& db) {
    if (!_connected) return;

    String val = db.getValueAsString(key);
    if (val == "NOT_FOUND") {
        Serial.printf("MQTT Error: Key '%s' not found in DB\n", key);
        return;
    }

    String topic = "esp32/gps/status/" + String(key);
    bool ok = publishAT(topic, val);
    if (!ok) Serial.printf("[MQTT] Echec publish %s\n", key);
}

bool MqttManager::subscribeAT(const String& topic, uint8_t qos) {
    while (Serial1.available()) _rxBuffer += (char)Serial1.read();
    String cmd = "AT+CMQTTSUB=0," + String(topic.length()) + "," + String(qos);
    Serial.print(">> "); Serial.println(cmd);
    Serial1.println(cmd);

    unsigned long t0 = millis();
    String resp = "";
    bool gotPrompt = false;
    while (millis() - t0 < 5000) {
        while (Serial1.available()) {
            char c = (char)Serial1.read();
            resp += c;
            if (c == '>') { gotPrompt = true; break; }
        }
        if (gotPrompt || resp.indexOf("ERROR") != -1) break;
    }
    if (!gotPrompt) {
        Serial.println("!! MQTT: pas de prompt pour SUB");
        Serial.println(resp);
        return false;
    }
    Serial1.print(topic);

    t0 = millis();
    resp = "";
    while (millis() - t0 < 8000) {
        while (Serial1.available()) resp += (char)Serial1.read();
        if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) {
            delay(150);
            while (Serial1.available()) resp += (char)Serial1.read();
            break;
        }
    }
    Serial.print("<< (sub) "); Serial.println(resp);
    Serial.println("---");
    return resp.indexOf("OK") != -1;
}

void MqttManager::poll() {
    while (Serial1.available()) _rxBuffer += (char)Serial1.read();

    int startIdx = _rxBuffer.indexOf("+CMQTTRXSTART");
    int endIdx   = _rxBuffer.indexOf("+CMQTTRXEND");

    if (startIdx != -1 && endIdx != -1 && endIdx > startIdx) {
        String block = _rxBuffer.substring(startIdx);

        int topicHdr   = block.indexOf("+CMQTTRXTOPIC:");
        int payloadHdr = block.indexOf("+CMQTTRXPAYLOAD:");

        if (topicHdr != -1 && payloadHdr != -1) {
            // --- Topic ---
            int topicLineEnd = block.indexOf('\n', topicHdr);
            String topicHeaderLine = block.substring(topicHdr, topicLineEnd);
            int topicLen = topicHeaderLine.substring(topicHeaderLine.lastIndexOf(',') + 1).toInt();

            int topicStart = topicLineEnd + 1;
            while (topicStart < (int)block.length() &&
                   (block.charAt(topicStart) == '\r' || block.charAt(topicStart) == '\n')) topicStart++;
            String topic = block.substring(topicStart, topicStart + topicLen);

            // --- Payload ---
            int payloadLineEnd = block.indexOf('\n', payloadHdr);
            String payloadHeaderLine = block.substring(payloadHdr, payloadLineEnd);
            int payloadLen = payloadHeaderLine.substring(payloadHeaderLine.lastIndexOf(',') + 1).toInt();

            int payloadStart = payloadLineEnd + 1;
            while (payloadStart < (int)block.length() &&
                   (block.charAt(payloadStart) == '\r' || block.charAt(payloadStart) == '\n')) payloadStart++;
            String payload = block.substring(payloadStart, payloadStart + payloadLen);

            Serial.printf("[MQTT RX] topic=%s payload=%s\n", topic.c_str(), payload.c_str());

            if (_callback) {
                _callback(topic.c_str(), (const byte*)payload.c_str(), payload.length());
            }
        }

        _rxBuffer = "";
    }

    if (_rxBuffer.length() > 1024) _rxBuffer = "";
}

bool MqttManager::disconnect() {
    if (!_connected) {
        Serial.println(">>> MQTT: déjà déconnecté, rien à faire.");
        return true;
    }

    Serial.println(">>> MQTT: déconnexion propre...");
    sendAT("AT+CMQTTDISC=0,60", 10000);
    sendAT("AT+CMQTTREL=0", 5000);
    sendAT("AT+CMQTTSTOP", 5000);

    _connected = false;
    Serial.println(">>> MQTT: déconnecté.");
    return true;
}

bool MqttManager::publishRouteInfo(const String& json) {
    if (!_connected) return false;
    return publishAT("esp32/gps/route_info", json);
}