#ifndef SPLINE_PATH_H
#define SPLINE_PATH_H
#include <Arduino.h>


class SplinePath {

// Structure d'un segment de la courbe


private:
    struct SplineSegment {
        float t_start;
        float t_end;
        double a, b, c, d; // Coefficients du polynôme
    };
    SplineSegment* _latSegments;
    SplineSegment* _lngSegments;
    uint8_t _segmentCount;

    // Fonction d'évaluation interne d'un polynôme
    double evaluatePolynomial(SplineSegment* segments, float t);

public:
    // Constructeur : on alloue la mémoire dynamiquement selon le nombre de segments
    SplinePath(uint8_t maxSegments);
    ~SplinePath();

    // Remplissage des données (à appeler lors de la réception MQTT)
    void setLatSegment(uint8_t index, float t_s, float t_e, double a, double b, double c, double d);
    void setLngSegment(uint8_t index, float t_s, float t_e, double a, double b, double c, double d);

    // Récupère la position cible exacte à l'instant T
    bool getTargetPosition(float currentTime, double &outLat, double &outLng);
    
    // Vérifie si la mission (la courbe) est terminée
    bool isComplete(float currentTime);
};

#endif