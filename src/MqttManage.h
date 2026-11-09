#ifndef MQTT_MANAGE_H
#define MQTT_MANAGE_H

#include <Arduino.h>
#include <TinyGsmClient.h>
#include "Bmi088V1.h"

class DataBase;

class MqttManager {
private:
    TinyGsm& _modem;
    const char* _user;
    const char* _pass;
    const char* _server;

    bool _connected = false;
    String _clientId;

    typedef void (*MqttCallback)(const char* topic, const byte* payload, uint32_t len);
    MqttCallback _callback = nullptr; // stocké mais pas encore utilisé (voir note en bas)

    // --- Bas niveau AT, repris tel quel du firmware de test qui fonctionne ---
    String sendAT(const String& cmd, uint32_t timeout = 8000);
    bool publishAT(const String& topic, const String& payload, uint32_t timeout = 5000);
    bool subscribeAT(const String& topic, uint8_t qos = 0);
    String _rxBuffer;
    const char* _subTopic = "esp32/gps/command"; // doit matcher topicCmd du site
    const char* _otaTopic = "esp32/gps/ota";
    uint8_t _consecutiveFailures = 0;

    void checkFailureThreshold() {
    if (_consecutiveFailures >= 3) {
        Serial.println(">>> MQTT: trop d'echecs consecutifs, on force une reconnexion.");
        _connected = false;
        _consecutiveFailures = 0;
    }
}


public:
    MqttManager(TinyGsm& modem, const char* server, const char* user, const char* pass);

    void setup(MqttCallback callback);

    bool connect();
    bool disconnect();
    void loop();
    bool connected();

    bool publishData(float lat, float lng, const IMUData& imu, bool hasFix);
    void publishParameter(const char* key, DataBase& db);
    void poll();

    bool publishRouteInfo(const String& json);
};

#endif