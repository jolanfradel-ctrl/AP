#include "LightKalman.h"

LightKalman::LightKalman() {
    isInitialized = false;
    lastMicros = micros();
    motionThreshold = 0.25f; // Valeur par défaut en m/s² (sensibilité du lock)

    x_lat[0] = 0.0; x_lat[1] = 0.0;
    x_lng[0] = 0.0; x_lng[1] = 0.0;

    P_lat[0][0] = 1.0f; P_lat[0][1] = 0.0f; P_lat[1][0] = 0.0f; P_lat[1][1] = 1.0f;
    P_lng[0][0] = 1.0f; P_lng[0][1] = 0.0f; P_lng[1][0] = 0.0f; P_lng[1][1] = 1.0f;
}

void LightKalman::init(double initial_lat, double initial_lng) {
    x_lat[0] = initial_lat; x_lat[1] = 0.0;
    x_lng[0] = initial_lng; x_lng[1] = 0.0;
    
    P_lat[0][0] = 1.0f; P_lat[0][1] = 0.0f; P_lat[1][0] = 0.0f; P_lat[1][1] = 1.0f;
    P_lng[0][0] = 1.0f; P_lng[0][1] = 0.0f; P_lng[1][0] = 0.0f; P_lng[1][1] = 1.0f;
    
    lastMicros = micros();
    isInitialized = true;
}

void LightKalman::predict(float acc_x, float acc_y, float gain_Q) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f; 
    lastMicros = now;

    if (!isInitialized || dt <= 0.0f || dt > 0.5f) return; 

    // --- LE LOCK ICI ---
    // Calcul de la norme de l'accélération sur le plan horizontal
    float acc_magnitude = sqrt(acc_x * acc_x + acc_y * acc_y);
    
    float final_acc_x = acc_x;
    float final_acc_y = acc_y;
    float final_Q = gain_Q;

    //Si le mouvement est trop faible (bruit/repos), on force l'accélération à zéro
    // et on réduit drastiquement Q pour bloquer la dérive de la covariance
         if (acc_magnitude < motionThreshold) {
             final_acc_x = 0.0f;
             final_acc_y = 0.0f;
             final_Q = 0.0f; 
             // On force aussi la vitesse estimée précédente à zéro pour dissiper l'énergie accumulée
             x_lat[1] = 0.0;
             x_lng[1] = 0.0;
         }

    // Conversion m/s² en degrés GPS
    const float M_TO_DEG = 0.000009f;
    float acc_lat_deg = final_acc_y * M_TO_DEG; 
    float acc_lng_deg = final_acc_x * M_TO_DEG; 

    predictAxis(x_lat[0], x_lat[1], P_lat, acc_lat_deg, dt, final_Q);
    predictAxis(x_lng[0], x_lng[1], P_lng, acc_lng_deg, dt, final_Q);
}

void LightKalman::update(double gps_lat, double gps_lng, float gain_R) {
    if (!isInitialized) {
        init(gps_lat, gps_lng);
        return;
    }

    updateAxis(x_lat[0], x_lat[1], P_lat, gps_lat, gain_R);
    updateAxis(x_lng[0], x_lng[1], P_lng, gps_lng, gain_R);
}

void LightKalman::predictAxis(double &x_pos, double &x_vel, float P[2][2], float acc, float dt, float Q) {
    x_pos = x_pos + x_vel * dt + 0.5f * acc * dt * dt;
    x_vel = x_vel + acc * dt;

    float P00_n = P[0][0] + dt * (P[1][0] + P[0][1] + dt * P[1][1]) + 0.25f * Q * dt * dt * dt * dt;
    float P01_n = P[0][1] + dt * P[1][1] + 0.5f * Q * dt * dt * dt;
    float P10_n = P[1][0] + dt * P[1][1] + 0.5f * Q * dt * dt * dt;
    float P11_n = P[1][1] + Q * dt * dt;

    P[0][0] = P00_n; P[0][1] = P01_n; P[1][0] = P10_n; P[1][1] = P11_n;
}

void LightKalman::updateAxis(double &x_pos, double &x_vel, float P[2][2], double gps_val, float R) {
    float S = P[0][0] + R;
    if (S == 0.0f) return;

    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double y = gps_val - x_pos;
    x_pos += K[0] * y;
    x_vel += K[1] * y;

    float P00_n = P[0][0] - K[0] * P[0][0];
    float P01_n = P[0][1] - K[0] * P[0][1];
    float P10_n = P[1][0] - K[1] * P[0][0];
    float P11_n = P[1][1] - K[1] * P[0][1];

    P[0][0] = P00_n; P[0][1] = P01_n; P[1][0] = P10_n; P[1][1] = P11_n;
}