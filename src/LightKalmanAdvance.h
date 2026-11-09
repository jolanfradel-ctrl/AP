#ifndef LIGHT_KALMAN_ADVANCE_H
#define LIGHT_KALMAN_ADVANCE_H
 
#include <Arduino.h>
 
// Filtre de Kalman GPS/ACC avec vecteur d'état à 3 composantes par axe :
//   x[0] = position   (degres)
//   x[1] = vitesse    (degres/s, unite "equivalente" via M_TO_DEG)
//   x[2] = biais acc  (meme unite que x[1], cf. M_TO_DEG dans predict())
//
// Le biais represente la derive lente de l'accelerometre (offset non nul,
// sensible a la temperature). Il n'est jamais mesure directement : il est
// uniquement corrige via les correlations de covariance P lors des
// recalages GPS (cf. update()).
class LightKalmanAdvance {
private:
    double x_lat[3];
    double x_lng[3];
 
    float P_lat[3][3];
    float P_lng[3][3];
 
    unsigned long lastMicros;
    bool isInitialized;
 
    // Seuil en m/s² en dessous duquel on considere qu'on est immobile (Lock)
    float motionThreshold;
 
    // locked = true => on ignore le biais pour ne pas reintroduire de derive
    // sur un axe qu'on vient de forcer a zero (cf. .cpp pour le detail)
    void predictAxis(double &x_pos, double &x_vel, double &x_bias,
                      float P[3][3], float acc, float dt,
                      float Q, float Qbias, bool locked);
 
    void updateAxis(double &x_pos, double &x_vel, double &x_bias,
                     float P[3][3], double gps_val, float R);
 
public:
    LightKalmanAdvance();
 
    void init(double initial_lat, double initial_lng);
 
    // acc_x, acc_y, acc_z : acceleration brute IMU en m/s² (repere capteur)
    // gain_Q      : bruit de process sur position/vitesse (comme avant)
    // gain_Q_bias : bruit de process sur la derive du biais (nouveau, doit
    //               etre nettement plus petit que gain_Q)
    void predict(float acc_x, float acc_y, float acc_z, float gain_Q, float gain_Q_bias);
 
    void update(double gps_lat, double gps_lng, float gain_R);
 
    void setMotionThreshold(float threshold) { motionThreshold = threshold; }
 
    double getLat() const { return x_lat[0]; }
    double getLng() const { return x_lng[0]; }
 
    // Debug / telemetrie : biais estime courant (meme unite que la vitesse
    // interne, pas directement en m/s² -- utile surtout pour verifier la
    // convergence, ex: tracer sa valeur dans le temps)
    double getBiasLat() const { return x_lat[2]; }
    double getBiasLng() const { return x_lng[2]; }
};
 
#endif