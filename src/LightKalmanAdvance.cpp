#include "LightKalmanAdvance.h"

// Valeur de reference de la gravite pour le detecteur de mouvement SVM
static const float GRAVITY = 9.81f;

LightKalmanAdvance::LightKalmanAdvance() {
    isInitialized = false;
    lastMicros = micros();
    motionThreshold = 0.25f;

    x_lat[0] = 0.0; x_lat[1] = 0.0; x_lat[2] = 0.0;
    x_lng[0] = 0.0; x_lng[1] = 0.0; x_lng[2] = 0.0;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_lat[i][j] = (i == j) ? 1.0f : 0.0f;
            P_lng[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void LightKalmanAdvance::init(double initial_lat, double initial_lng) {
    x_lat[0] = initial_lat; x_lat[1] = 0.0; x_lat[2] = 0.0;
    x_lng[0] = initial_lng; x_lng[1] = 0.0; x_lng[2] = 0.0;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_lat[i][j] = (i == j) ? 1.0f : 0.0f;
            P_lng[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    lastMicros = micros();
    isInitialized = true;
}

void LightKalmanAdvance::predict(float acc_x, float acc_y, float acc_z, float gain_Q, float gain_Q_bias) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;

    if (!isInitialized || dt <= 0.0f || dt > 0.5f) return;

    // --- LOCK via SVM (Signal Vector Magnitude) sur les 3 axes ---
    // Contrairement a sqrt(ax^2+ay^2) seul, cette methode reste valable
    // meme si le device est incline (la norme d'un vecteur ne change pas
    // par rotation). On soustrait g car au repos l'IMU mesure la reaction
    // a la gravite, pas 0.
    float acc_norm = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    float acc_magnitude = fabs(acc_norm - GRAVITY);

    bool locked = (acc_magnitude < motionThreshold);

    float acc_lat_deg = 0.0f;
    float acc_lng_deg = 0.0f;
    float final_Q = gain_Q;

    if (locked) {
        final_Q = 0.0f;
        // On dissipe l'energie accumulee sur la vitesse (comme avant).
        // ATTENTION : le biais n'est PAS remis a zero ici, et predictAxis
        // ignorera la correction de biais dans ce cas (cf. plus bas) pour
        // ne pas reintroduire de derive sur une vitesse qu'on vient de figer.
        x_lat[1] = 0.0;
        x_lng[1] = 0.0;
    } else {
        // Conversion m/s² -> degres GPS (unite interne du filtre)
        const float M_TO_DEG = 0.000009f;
        acc_lat_deg = acc_y * M_TO_DEG;
        acc_lng_deg = acc_x * M_TO_DEG;
    }

    predictAxis(x_lat[0], x_lat[1], x_lat[2], P_lat, acc_lat_deg, dt, final_Q, gain_Q_bias, locked);
    predictAxis(x_lng[0], x_lng[1], x_lng[2], P_lng, acc_lng_deg, dt, final_Q, gain_Q_bias, locked);
}

void LightKalmanAdvance::update(double gps_lat, double gps_lng, float gain_R) {
    if (!isInitialized) {
        init(gps_lat, gps_lng);
        return;
    }

    updateAxis(x_lat[0], x_lat[1], x_lat[2], P_lat, gps_lat, gain_R);
    updateAxis(x_lng[0], x_lng[1], x_lng[2], P_lng, gps_lng, gain_R);
}

// x' = F.x + bruit, avec F = [1  dt  -0.5dt²]
//                             [0  1   -dt   ]
//                             [0  0   1     ]
void LightKalmanAdvance::predictAxis(double &x_pos, double &x_vel, double &x_bias,
                                       float P[3][3], float acc, float dt,
                                       float Q, float Qbias, bool locked) {
    // En mode locked, acc vaut deja 0 et on ne veut PAS soustraire le biais
    // (sinon vel repartirait a -bias*dt au lieu de rester a 0).
    float acc_effective = locked ? acc : (acc - (float)x_bias);

    x_pos = x_pos + x_vel * dt + 0.5 * acc_effective * dt * dt;
    x_vel = x_vel + acc_effective * dt;
    // x_bias inchange ici (marche aleatoire geree via le bruit Qbias sur P)

    // --- Propagation de la covariance : P' = F.P.F^T + Q ---
    float a = dt;
    float b = 0.5f * dt * dt;

    // Etape 1 : FP = F * P
    float FP00 = P[0][0] + a * P[1][0] - b * P[2][0];
    float FP01 = P[0][1] + a * P[1][1] - b * P[2][1];
    float FP02 = P[0][2] + a * P[1][2] - b * P[2][2];

    float FP10 = P[1][0] - a * P[2][0];
    float FP11 = P[1][1] - a * P[2][1];
    float FP12 = P[1][2] - a * P[2][2];

    float FP20 = P[2][0];
    float FP21 = P[2][1];
    float FP22 = P[2][2];

    // Etape 2 : P' = FP * F^T
    float P00_n = FP00 + a * FP01 - b * FP02;
    float P01_n = FP01 - a * FP02;
    float P02_n = FP02;

    float P10_n = FP10 + a * FP11 - b * FP12;
    float P11_n = FP11 - a * FP12;
    float P12_n = FP12;

    float P20_n = FP20 + a * FP21 - b * FP22;
    float P21_n = FP21 - a * FP22;
    float P22_n = FP22;

    // Etape 3 : ajout du bruit de process
    // Bloc position/vitesse : bruit sur l'acceleration (comme la version 2 etats)
    P00_n += 0.25f * Q * dt * dt * dt * dt;
    P01_n += 0.5f * Q * dt * dt * dt;
    P10_n += 0.5f * Q * dt * dt * dt;
    P11_n += Q * dt * dt;
    // Biais : bruit independant, derive lente
    P22_n += Qbias * dt;

    P[0][0] = P00_n; P[0][1] = P01_n; P[0][2] = P02_n;
    P[1][0] = P10_n; P[1][1] = P11_n; P[1][2] = P12_n;
    P[2][0] = P20_n; P[2][1] = P21_n; P[2][2] = P22_n;
}

// H = [1 0 0] : le GPS n'observe que la position
void LightKalmanAdvance::updateAxis(double &x_pos, double &x_vel, double &x_bias,
                                      float P[3][3], double gps_val, float R) {
    float S = P[0][0] + R;
    if (S == 0.0f) return;

    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float K2 = P[2][0] / S;

    double y = gps_val - x_pos;
    x_pos  += K0 * y;
    x_vel  += K1 * y;
    x_bias += K2 * y;

    // P = (I - K.H) . P
    float P00_n = P[0][0] - K0 * P[0][0];
    float P01_n = P[0][1] - K0 * P[0][1];
    float P02_n = P[0][2] - K0 * P[0][2];

    float P10_n = P[1][0] - K1 * P[0][0];
    float P11_n = P[1][1] - K1 * P[0][1];
    float P12_n = P[1][2] - K1 * P[0][2];

    float P20_n = P[2][0] - K2 * P[0][0];
    float P21_n = P[2][1] - K2 * P[0][1];
    float P22_n = P[2][2] - K2 * P[0][2];

    P[0][0] = P00_n; P[0][1] = P01_n; P[0][2] = P02_n;
    P[1][0] = P10_n; P[1][1] = P11_n; P[1][2] = P12_n;
    P[2][0] = P20_n; P[2][1] = P21_n; P[2][2] = P22_n;
}