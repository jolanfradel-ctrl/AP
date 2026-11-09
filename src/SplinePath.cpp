#include "SplinePath.h"

SplinePath::SplinePath(uint8_t maxSegments) {
    _segmentCount = maxSegments;
    _latSegments = new SplineSegment[_segmentCount];
    _lngSegments = new SplineSegment[_segmentCount];
}

SplinePath::~SplinePath() {
    delete[] _latSegments;
    delete[] _lngSegments;
}

void SplinePath::setLatSegment(uint8_t index, float t_s, float t_e, double a, double b, double c, double d) {
    if (index >= _segmentCount) return;
    _latSegments[index] = {t_s, t_e, a, b, c, d};
}

void SplinePath::setLngSegment(uint8_t index, float t_s, float t_e, double a, double b, double c, double d) {
    if (index >= _segmentCount) return;
    _lngSegments[index] = {t_s, t_e, a, b, c, d};
}

double SplinePath::evaluatePolynomial(SplineSegment* segments, float t) {
    SplineSegment* current = &segments[_segmentCount - 1]; // Dernier segment par défaut
    
    // Trouver le bon segment pour l'instant 't'
    for (uint8_t i = 0; i < _segmentCount; i++) {
        if (t >= segments[i].t_start && t <= segments[i].t_end) {
            current = &segments[i];
            break;
        }
    }

    // Calcul du temps relatif au segment
    float dt = t - current->t_start;

    // Évaluation du polynôme (a*dt^3 + b*dt^2 + c*dt + d)
    return ((current->a * dt + current->b) * dt + current->c) * dt + current->d;
}

bool SplinePath::getTargetPosition(float currentTime, double &outLat, double &outLng) {
    if (_segmentCount == 0) return false;
    
    // Verrouiller le temps au maximum si on dépasse la fin de la courbe
    if (currentTime > _latSegments[_segmentCount - 1].t_end) {
        currentTime = _latSegments[_segmentCount - 1].t_end;
    }

    outLat = evaluatePolynomial(_latSegments, currentTime);
    outLng = evaluatePolynomial(_lngSegments, currentTime);

    return true;
}

bool SplinePath::isComplete(float currentTime) {
    if (_segmentCount == 0) return true;
    return currentTime >= _latSegments[_segmentCount - 1].t_end;
}