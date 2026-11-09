#ifndef LIGHT_KALMAN_H
#define LIGHT_KALMAN_H

#include <Arduino.h>

class LightKalman {
private:
    double x_lat[2]; 
    double x_lng[2];

    float P_lat[2][2];
    float P_lng[2][2];

    unsigned long lastMicros;
    bool isInitialized;
    
    // Seuil en m/s² en dessous duquel on considère qu'on est immobile (Lock)
    float motionThreshold; 

    void predictAxis(double &x_pos, double &x_vel, float P[2][2], float acc, float dt, float Q);
    void updateAxis(double &x_pos, double &x_vel, float P[2][2], double gps_val, float R);
    void filterGps();
    
public:
    LightKalman();

    void init(double initial_lat, double initial_lng);

    // Étape de Prédiction modifiée pour intégrer le Lock automatique
    void predict(float acc_x, float acc_y, float gain_Q);

    void update(double gps_lat, double gps_lng, float gain_R);

    // Permet de modifier le seuil de détection de mouvement (Ex: 0.25 m/s² par défaut)
    void setMotionThreshold(float threshold) { motionThreshold = threshold; }

    double getLat() const { return x_lat[0]; }
    double getLng() const { return x_lng[0]; }
};

#endif