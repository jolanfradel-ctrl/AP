#ifndef MQTT_MANAGE_H
#define MQTT_MANAGE_H

#include <Arduino.h>
#include <PubSubClient.h>
#include "Bmi088V1.h"

// On dit au compilateur que la classe DataBase existe sans l'inclure totalement ici
// Cela évite les erreurs de "dépendances circulaires"
class DataBase; 

class MqttManager {
private:
    PubSubClient _client;
    const char* _user;
    const char* _pass;
    const char* _server;

public:
    MqttManager(Client& netClient, const char* server, const char* user, const char* pass);

    void setup(MQTT_CALLBACK_SIGNATURE);
    bool connect();
    void loop();
    bool connected();
    
    // Envoi du pack complet
    bool publishData(float lat, float lng, const IMUData& imu, bool hasFix);
    
    // Envoi d'une seule ligne (récupérée dans la DB)
    void publishParameter(const char* key, DataBase& db);
};

#endif