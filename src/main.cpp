#define TINY_GSM_MODEM_A7670
#define TINY_GSM_DEBUG Serial

#include <TinyGsmClient.h>
//#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "Bmi088V1.h"
#include "MqttManage.h"
#include "DataBase.h"
#include "LightKalmanAdvance.h"
#include "EepromManager.h"
#include "Navigator.h"
#include <esp_system.h>
#include "esp_task_wdt.h"

#include "IFirmwareSource.h"
#include "GsmFirmwareSource.h"
#include "OTAUpdater.h"
#include "GY273.h"


// --- CONFIGURATION ---
// --- Pins modem (garde exactement les mêmes valeurs que ton sketch ATdebug qui a marché en V1.0) ---
#define MODEM_TX_PIN       26   
#define MODEM_RX_PIN       27
#define MODEM_PWRKEY_PIN   4
#define MODEM_RESET_PIN    5
#define BOARD_POWERON_PIN  12








#define ACC_ADDR 0x19
#define GYR_ADDR 0x69



// --- VARIABLES DE CONTRÔLE & FILTRE ---
float gain_P = 1.0;
float gain_Q = 0.001; // Kalman Q
float gain_R = 0.01;  // Kalman R
float gain_Qbias = 0.00001f;
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

float  headingComp =0.0;

float Version =1.3;

// --- INSTANCES ---
Bmi088V1 imu(ACC_ADDR, GYR_ADDR);

GY273 compass(0x1E);


#define SerialAT Serial1
TinyGsm           modem(SerialAT);

MqttManager mqtt(modem, mqtt_server, mqtt_user, mqtt_pass);
DataBase db;
EepromManager eeprom;
Navigator nav_system;


volatile IMUData globalIMU; 
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
TaskHandle_t ComTaskHandle;

LightKalmanAdvance kalman;             
double filtered_lat = 0.0;      
double filtered_lng = 0.0;

GsmFirmwareSource gsmSource(SerialAT);
OTAUpdater updater(gsmSource);

volatile bool otaRequested = false;
String otaUrl = "";

// --- ROUTE INFO (distances + bearings entre waypoints) ---
#define ROUTE_INFO_BUF_SIZE 1024
char routeInfoJson[ROUTE_INFO_BUF_SIZE];
volatile bool routeInfoReady = false;
SemaphoreHandle_t routeInfoMutex;

// --- À REMPLACER DANS TON MAIN.CPP ---



void modemFullPowerCycle() {
    Serial.println(">>> Power cycle complet du modem (OFF -> ON)...");

    // 1) Coupe le rail pour repartir d'un état garanti propre
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, LOW);
    delay(1500);

    // 2) Rallume le rail
    digitalWrite(BOARD_POWERON_PIN, HIGH);
    delay(200);

    // 3) Pulse PWRKEY
    pinMode(MODEM_PWRKEY_PIN, OUTPUT);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(MODEM_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);
}

void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch (reason) {
    case ESP_RST_WDT:        Serial.println("Interrupt WDT (générique)"); break;
    case ESP_RST_TASK_WDT:   Serial.println("TASK WATCHDOG -> confirmé !"); break;
    case ESP_RST_INT_WDT:    Serial.println("Interrupt WDT (ISR trop longue)"); break;
    case ESP_RST_PANIC:      Serial.println("Panic (crash logiciel, pas watchdog)"); break;
    case ESP_RST_BROWNOUT:   Serial.println("Brownout -> problème d'alimentation !"); break;
    case ESP_RST_POWERON:    Serial.println("Power-on normal"); break;
    case ESP_RST_SW:         Serial.println("Reset logiciel (ESP.restart())"); break;
    default:                 Serial.printf("Autre: %d\n", reason); break;
  }
}

// --- CALLBACK MQTT (COEUR 0) ---
void monMqttCallback(const char* topic, const byte* payload, uint32_t length) {
    String message = "";
    for (uint32_t i = 0; i < length; i++) message += (char)payload[i];

    // --- Détection trigger OTA ---
    if (String(topic) == "esp32/gps/ota") {
        Serial.printf("[OTA] Trigger reçu, url=%s\n", message.c_str());
        otaUrl = message;
        otaRequested = true;
        return; // ne pas traiter comme un paramètre normal
    }

    float temp_Q = gain_Q;
    float temp_R = gain_R;

    Serial.printf("[MQTT] Message recu : %s\n", message.c_str());
    db.updateFromPayload(message);

    if (temp_Q != gain_Q || temp_R != gain_R) {
        eeprom.save(gain_P, gain_Q, gain_Qbias, gain_R, lock_sensitivity);
    }
}

bool waitForNetworkWithWDT(TinyGsm &modem, uint32_t timeoutMs = 60000) {
  uint32_t start = millis();
  const uint32_t checkInterval = 1000; // vérifie toutes les 1s

  while (millis() - start < timeoutMs) {
    esp_task_wdt_reset();          // nourrit le watchdog à chaque itération

    if (modem.isNetworkConnected()) {
      Serial.println("Réseau connecté !");
      return true;
    }

    vTaskDelay(pdMS_TO_TICKS(checkInterval)); // cède le CPU, laisse IDLE0 tourner
  }

  Serial.println("Timeout réseau");
  return false;
}

// --- TÂCHE COMMUNICATION (COEUR 0) ---
void CommunicationTask(void * pvParameters) {
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    // 1. Allumage du modem
    modemFullPowerCycle();  // à ajouter, cf. discussion précédente
    esp_task_wdt_reset();

    

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    delay(3000);
    esp_task_wdt_reset();

    Serial.println(">>> Initialisation du modem...");
    modem.init(); 
    esp_task_wdt_reset(); 

    updater.attachModem(modem);
    updater.attachSerial(SerialAT);


    if (strlen(simPIN) && modem.getSimStatus() != 3) {
        modem.simUnlock(simPIN);
        esp_task_wdt_reset();
    }

    if (!modem.isNetworkConnected()) {
        waitForNetworkWithWDT(modem, 60000);
    }
    esp_task_wdt_reset();

    bool gprsOk = modem.gprsConnect(apn, gprsUser, gprsPass);
    Serial.printf(">>> GPRS: %s | IP: %s\n", gprsOk ? "OK" : "ECHEC", modem.getLocalIP().c_str());

    esp_task_wdt_reset();
    Serial.println(">>> Stabilisation du contexte PDP...");
    delay(5000);
    esp_task_wdt_reset();
   
    
    // On lie notre fonction de réception
    mqtt.setup(monMqttCallback);

    esp_task_wdt_reset();

    // 3. Boucle infinie
    for(;;) {
        esp_task_wdt_reset(); 
        if (otaRequested) {
            otaRequested = false;
            Serial.println(">>> OTA demandé, déconnexion MQTT...");
            mqtt.disconnect();

            bool ok = updater.run(otaUrl);
            if (!ok) {
                Serial.println(">>> OTA: échec, reprise du fonctionnement normal.");
                // mqtt.connect() sera retenté normalement au prochain tour
            }
            // si succès, updater.run() a déjà fait ESP.restart() en interne
            continue;
        }
        // mqtt.connect() va vérifier si on est connecté.
        // Si non, il fera le modem.mqtt_connect(...) et le subscribe(...) !
        mqtt.connect(); 
        mqtt.poll();
        
        // (On a supprimé mqtt.loop() car le modem A7670 gère les pings lui-même)
        
        unsigned long now = millis();
        static unsigned long lastSlowMsg = 0;
        const unsigned long pubInterval = 300; // 11 params x 80ms délai ≈ 900ms, on prend une marge

        static const char* paramKeys[] = {
            "LAT", "LNG", "FLAT", "FLNG",
            "AX", "AY", "AZ", "SPD",
            "Pitch", "Roll", "Yaw", "Version","Cap"
        };
        const int paramCount = sizeof(paramKeys) / sizeof(paramKeys[0]);

        if (now - lastSlowMsg > pubInterval) {
            lastSlowMsg = now;
            for (int i = 0; i < paramCount; i++) {
                mqtt.publishParameter(paramKeys[i], db);
                esp_task_wdt_reset();
                delay(80); // laisse le temps au modem de traiter l'AT+CMQTTPUB précédent
                mqtt.poll();

            }
        }

        if (routeInfoReady) {
    char localCopy[ROUTE_INFO_BUF_SIZE];
    bool hasData = false;

    if (xSemaphoreTake(routeInfoMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        strncpy(localCopy, routeInfoJson, ROUTE_INFO_BUF_SIZE - 1);
        localCopy[ROUTE_INFO_BUF_SIZE - 1] = '\0';
        routeInfoReady = false;
        hasData = true;
        xSemaphoreGive(routeInfoMutex);  // <-- libéré tout de suite, avant le publish
    }

    if (hasData) {
        Serial.printf("[MQTT] Publication route info : %s\n", localCopy);
        bool ok = mqtt.publishRouteInfo(String(localCopy));
        if (!ok) Serial.println("[MQTT] Echec publish route_info");
        mqtt.poll();
    }
}

        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

// --- SETUP (COEUR 1) ---
void setup() {
    Serial.begin(115200);
    delay(500);
    printResetReason();

    xTaskCreatePinnedToCore(CommunicationTask, "ComTask", 10000, NULL, 1, &ComTaskHandle, 0);


    esp_task_wdt_deinit();          // force la désinit du WDT existant
    esp_err_t err = esp_task_wdt_init(60, true);


    SerialGPS.begin(9600, SERIAL_8N1, 14, 13);
    Wire.begin(21, 22);
   
    Wire.setClock(400000);
    if (!eeprom.load(gain_P, gain_Q, gain_Qbias, gain_R, lock_sensitivity)) {
    Serial.println("[EEPROM] Utilisation des valeurs par défaut.");
    }
    if (!compass.begin()) {
        Serial.println("GY-273 non détecté !");
    }
    compass.setDeclination(1.5);
    
    routeInfoMutex = xSemaphoreCreateMutex();

    imu.begin();
    // 1. Liaison des paramètres simples
    db.link("KP", &gain_P);
    db.link("KQ", &gain_Q);
    db.link("KQBIAS", &gain_Qbias);
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
    db.link("Version", &Version);
    db.link("Cap",&headingComp);
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
    
    Serial.println(">>> Système prêt. En attente de Waypoints...");
}

// --- BOUCLE PRINCIPALE (COEUR 1) ---
void loop() {

    kalman.setMotionThreshold(lock_sensitivity);
    IMUData freshData = imu.readAll();
    MagData mag = compass.readAll();
    // 2. MISE À JOUR DES VARIABLES GLOBALES
    // Comme db.link("AX", &cur_ax) a été fait, mettre à jour cur_ax met à jour la DB
    cur_ax = freshData.ax;
    cur_ay = freshData.ay;
    cur_az = freshData.az;


    kalman.predict(cur_ax, cur_ay, gain_Q, gain_Qbias, gain_R);
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
    headingComp = compass.getTiltCompensatedHeading(mag, imu._pry.roll, imu._pry.pitch);
    //Serial.printf("Capcompensé=%.1f°\n",  headingComp);    // --- DETECTION DE CHANGEMENT DE PARCOURS ---
    static uint8_t old_count = 0;
    static double old_first_lat = 0.0;

    if (wpoints_count != old_count ||  (wpoints_count > 0 && wpoints_lat[0] != old_first_lat)) { 
        Serial.printf("\n[COEUR 1] NOUVEAU PARCOURS REÇU ! (%d points)\n", wpoints_count);
        for(int i = 0; i < wpoints_count; i++) {
            if(i < MAX_WAYPOINTS) {
                Serial.printf("  -> WP %d : LAT=%.8f | LNG=%.8f | SPD=%.2f\n", i, wpoints_lat[i], wpoints_lng[i], wpoints_spd[i]);

            }
        }
        // --- Calcul distance + bearing pour CHAQUE segment consécutif ---
        char tmpBuf[ROUTE_INFO_BUF_SIZE];
        int offset = 0;
        offset += snprintf(tmpBuf + offset, ROUTE_INFO_BUF_SIZE - offset, "[");

        for (int i = 0; i < wpoints_count - 1 && i < MAX_WAYPOINTS - 1; i++) {
            float dist = nav_system.distanceTo(wpoints_lat[i], wpoints_lng[i], wpoints_lat[i+1], wpoints_lng[i+1]);
            float bearing = nav_system.bearingTo(wpoints_lat[i], wpoints_lng[i], wpoints_lat[i+1], wpoints_lng[i+1]);

            Serial.printf("  -> Segment %d->%d : dist=%.2f m | cap=%.2f°\n", i, i+1, dist, bearing);

            offset += snprintf(tmpBuf + offset, ROUTE_INFO_BUF_SIZE - offset,
                                "%s{\"seg\":%d,\"d\":%.2f,\"b\":%.2f}",
                                (i == 0) ? "" : ",", i, dist, bearing);

            if (offset >= ROUTE_INFO_BUF_SIZE - 50) {
                Serial.println("[ROUTE] Buffer JSON presque plein, troncature !");
                break;
            }
        }

        snprintf(tmpBuf + offset, ROUTE_INFO_BUF_SIZE - offset, "]");

        if (xSemaphoreTake(routeInfoMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            strncpy(routeInfoJson, tmpBuf, ROUTE_INFO_BUF_SIZE - 1);
            routeInfoJson[ROUTE_INFO_BUF_SIZE - 1] = '\0';
            routeInfoReady = true;
            xSemaphoreGive(routeInfoMutex);

            old_count = wpoints_count;
            old_first_lat = wpoints_lat[0];
        } else {
            Serial.println("[ROUTE] Echec verrouillage buffer, nouvelle tentative au prochain tour");
            // old_count/old_first_lat PAS mis à jour -> le changement sera redétecté au prochain passage
        }
        

        old_count = wpoints_count;
        old_first_lat = wpoints_lat[0];
        
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); 
}
