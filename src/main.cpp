#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "Bmi088V1.h"
#include "MqttManage.h"
#include "DataBase.h"
#include "LightKalman.h"
#include "EepromManager.h"
#include "Navigator.h"


// --- CONFIGURATION ---
const char* ssid = "Galaxy A20e2E93";
const char* password = "mihj6071";
const char* mqtt_server = "21f2976a53f440f5b67745ca8e8a14b9.s1.eu.hivemq.cloud";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "Esp32_Gps";

#define ACC_ADDR 0x19
#define GYR_ADDR 0x69



// --- VARIABLES DE CONTRÔLE & FILTRE ---
float gain_P = 1.0;
float gain_Q = 0.001; // Kalman Q
float gain_R = 0.01;  // Kalman R
float lock_sensitivity = 0.25;
uint8_t system_mode = 0;

// --- TELEMETRIE & GPS ---
float cur_ax = 0.0, cur_ay = 0.0, cur_az = 0.0; 
double lat = 0.0, lng = 0.0; 
float speed_ms = 0.0;

// --- GESTION DES WAYPOINTS (MAX 20) ---
#define MAX_WAYPOINTS 20
double wpoints_lat[MAX_WAYPOINTS];
double wpoints_lng[MAX_WAYPOINTS];
float  wpoints_spd[MAX_WAYPOINTS];
uint8_t wpoints_count = 0; 

// --- INSTANCES ---
Bmi088V1 imu(ACC_ADDR, GYR_ADDR);
WiFiClientSecure espClient;
MqttManager mqtt(espClient, mqtt_server, mqtt_user, mqtt_pass);
DataBase db;
EepromManager eeprom;
Navigator nav_system;


volatile IMUData globalIMU; 
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TaskHandle_t ComTaskHandle;

LightKalman kalman;             
double filtered_lat = 0.0;      
double filtered_lng = 0.0;

// --- CALLBACK MQTT (COEUR 0) ---
void monMqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    float temp_P, temp_Q, temp_R;
    for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
    db.updateFromPayload(message);
    if( temp_Q != gain_Q || temp_R != gain_R) {
        eeprom.save(gain_P, gain_Q, gain_R, lock_sensitivity);
    }

}

// --- TÂCHE COMMUNICATION (COEUR 0) ---
void CommunicationTask(void * pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) vTaskDelay(500 / portTICK_PERIOD_MS);
    espClient.setInsecure();
    mqtt.setup(monMqttCallback);

    for(;;) {
        mqtt.connect();
        mqtt.loop();

        unsigned long now = millis();

        // --- ENVOI LENT (Toutes les 1000ms) ---
        static unsigned long lastSlowMsg = 0;
        
        if (now - lastSlowMsg > 100) {
            lastSlowMsg = now;

            // GPS brut → LAT / LNG
            mqtt.publishParameter("LAT", db);   // lat = valeur brute GPS
            mqtt.publishParameter("LNG", db);   // lng = valeur brute GPS

            // Kalman filtré → FLAT / FLNG
            mqtt.publishParameter("FLAT", db);  // filtered_lat
            mqtt.publishParameter("FLNG", db);  // filtered_lng

                // IMU → AX / AY / AZ
            mqtt.publishParameter("AX", db);    // cur_ax       
            mqtt.publishParameter("AY", db);    // cur_ay
            mqtt.publishParameter("AZ", db);    // cur_az
            mqtt.publishParameter("SPD", db);   // speed_ms
        
            mqtt.publishParameter("Pitch", db); // imu._pry.pitch
            mqtt.publishParameter("Roll", db);  // imu._pry.roll  
            mqtt.publishParameter("Yaw", db);   // imu._pry.yaw
            
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// --- SETUP (COEUR 1) ---
void setup() {
    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, 14, 13);
    Wire.begin(21, 22);
   
    Wire.setClock(400000);
    if (!eeprom.load(gain_P, gain_Q, gain_R, lock_sensitivity)) {
    Serial.println("[EEPROM] Utilisation des valeurs par défaut.");
    }

    imu.begin();
    // 1. Liaison des paramètres simples
    db.link("KP", &gain_P);
    db.link("KQ", &gain_Q);
    db.link("KR", &gain_R);
    db.link("MODE", &system_mode);
    db.link("LAT", &lat);
    db.link("LNG", &lng);
    db.link("AX", &cur_ax);
    db.link("AY", &cur_ay);
    db.link("AZ", &cur_az);
    db.link("SPD", &speed_ms);
    db.link("FLAT", &filtered_lat); 
    db.link("FLNG", &filtered_lng);
    db.link("LOCK_THRES", &lock_sensitivity);
    db.link("Pitch", &imu._pry.pitch);
    db.link("Roll", &imu._pry.roll);
    db.link("Yaw", &imu._pry.yaw);
    // 2. BOUCLE DYNAMIQUE : Liaison des 20 Waypoints
    // On crée les clés WPLAT0, WPLNG0... jusqu'à 19
   
    static char keyLatBuf[MAX_WAYPOINTS][10];
    static char keyLngBuf[MAX_WAYPOINTS][10];
    static char keySpdBuf[MAX_WAYPOINTS][10];
    for (int i = 0; i < MAX_WAYPOINTS; i++) {
        sprintf(keyLatBuf[i], "WPLAT%d", i);
        sprintf(keyLngBuf[i], "WPLNG%d", i);
        sprintf(keySpdBuf[i], "WPSPD%d", i);
        db.link(keyLatBuf[i], &wpoints_lat[i]);
        db.link(keyLngBuf[i], &wpoints_lng[i]);
        db.link(keySpdBuf[i], &wpoints_spd[i]);
    }
    db.link("WPCNT", &wpoints_count);
    xTaskCreatePinnedToCore(CommunicationTask, "ComTask", 10000, NULL, 1, &ComTaskHandle, 0);
    
    Serial.println(">>> Système prêt. En attente de Waypoints...");
}

// --- BOUCLE PRINCIPALE (COEUR 1) ---
void loop() {
    kalman.setMotionThreshold(lock_sensitivity);
    IMUData freshData = imu.readAll();

    // 2. MISE À JOUR DES VARIABLES GLOBALES
    // Comme db.link("AX", &cur_ax) a été fait, mettre à jour cur_ax met à jour la DB
    cur_ax = freshData.ax;
    cur_ay = freshData.ay;
    cur_az = freshData.az;

    kalman.predict(cur_ax, cur_ay, gain_Q);
    filtered_lat = kalman.getLat();
    filtered_lng = kalman.getLng();
   
    static unsigned long lastGpsFixTime = 0;

    // --- MISE À JOUR GPS (Correction de la boucle infinie) ---
    
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read(); // On lit l'octet pour vider le tampon !
        if (gps.encode(c)) {       // On l'envoie à TinyGPS++ pour décodage            
            if (gps.location.isUpdated()) {
                lat = gps.location.lat();
                lng = gps.location.lng();
                speed_ms    = gps.speed.mps();     // m/s
                lastGpsFixTime = millis();
                kalman.update(lat, lng, gain_R);
                
                // Mise à jour post-correction
                filtered_lat = kalman.getLat();
                filtered_lng = kalman.getLng();

                bool wp_atteint = nav_system.updateNavigation(filtered_lat, filtered_lng, 3.0f);
    
                if (wp_atteint) {
                    Serial.printf("[NAV] Waypoint %d atteint !\n", nav_system.getCurrentWaypointIndex() - 1);
                }
               // Serial.printf("[GPS] Fix reçu : LAT=%.8f | LNG=%.8f\n", filtered_lat, filtered_lng);
            }
        }
    }

    imu._pry = imu.calculatePRY();
    // --- DETECTION DE CHANGEMENT DE PARCOURS ---
    static uint8_t old_count = 0;
    static double old_first_lat = 0.0;

    if (wpoints_count != old_count ||  (wpoints_count > 0 && wpoints_lat[0] != old_first_lat)) { 
        Serial.printf("\n[COEUR 1] NOUVEAU PARCOURS REÇU ! (%d points)\n", wpoints_count);
        for(int i = 0; i < wpoints_count; i++) {
            if(i < MAX_WAYPOINTS) {
                Serial.printf("  -> WP %d : LAT=%.8f | LNG=%.8f | SPD=%.2f\n", i, wpoints_lat[i], wpoints_lng[i], wpoints_spd[i]);

            }
        }
        float dist= nav_system.distanceTo(wpoints_lat[0], wpoints_lng[0], wpoints_lat[1], wpoints_lng[1]);
        Serial.printf("  -> Distance au premier waypoint : %.2f m\n", dist);
        float bearing= nav_system.bearingTo(wpoints_lat[0], wpoints_lng[0], wpoints_lat[1], wpoints_lng[1]);
        Serial.printf("  -> Cap vers le premier waypoint : %.2f °\n", bearing);

        old_count = wpoints_count;
        old_first_lat = wpoints_lat[0];
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); 
}