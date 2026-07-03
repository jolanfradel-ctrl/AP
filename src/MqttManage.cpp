#include "MqttManage.h"
#include "DataBase.h" // INDISPENSABLE ICI

MqttManager::MqttManager(Client& netClient, const char* server, const char* user, const char* pass) 
    : _client(netClient), _server(server), _user(user), _pass(pass) {
}

void MqttManager::setup(MQTT_CALLBACK_SIGNATURE) {
    _client.setServer(_server, 8883);
    _client.setCallback(callback);
}

bool MqttManager::connect() {
    if (!_client.connected()) {
        Serial.print("Tentative de connexion MQTT...");
        String clientId = "ESP32_Jolan_" + String(uint32_t(ESP.getEfuseMac()), HEX);
        
        if (_client.connect(clientId.c_str(), _user, _pass)) {
            Serial.println(" Connecté !");
            _client.subscribe("esp32/gps/command");
            return true;
        } else {
            Serial.print(" Échec, code=");
            Serial.println(_client.state());
            return false;
        }
    }
    return true;
}

void MqttManager::loop() {
    _client.loop();
}

bool MqttManager::connected() {
    return _client.connected();
}

bool MqttManager::publishData(float lat, float lng, const IMUData& imu, bool hasFix) {
    if (!_client.connected()) return false;

    char msg[350];
    snprintf(msg, sizeof(msg), 
        "{\"lat\":%.6f,\"lng\":%.6f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"fix\":%s}", 
        lat, lng, 
        imu.ax, imu.ay, imu.az, 
        imu.gx, imu.gy, imu.gz,
        hasFix ? "true" : "false"
    );

    return _client.publish("esp32/gps/data", msg);
}

void MqttManager::publishParameter(const char* key, DataBase& db) {
    if (!_client.connected()) return;
    
    // On appelle la méthode de la classe DataBase
    String val = db.getValueAsString(key);
    
    if (val != "NOT_FOUND") {
        String topic = "esp32/gps/status/" + String(key);
        _client.publish(topic.c_str(), val.c_str());
    } else {
        Serial.printf("MQTT Error: Key '%s' not found in DB\n", key);
    }
}