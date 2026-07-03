#include "Navigator.h"
#include <math.h>

Navigator::Navigator() {
    _count = 0;
    _currentIndex = 0;
}


float Navigator::toRad(double deg) {
   
    return deg * M_PI / 180.0;
}

float Navigator::distanceTo(double lat1, double lng1,
                             double lat2, double lng2) {
    // tout en double pour garder la précision GPS
    double lat1rd = toRad(lat1);
    double lng1rd = toRad(lng1);
    double lat2rd = toRad(lat2);
    double lng2rd = toRad(lng2);

    double dlat = lat2rd - lat1rd;
    double dlng = lng2rd - lng1rd;

    double a = sin(dlat / 2) * sin(dlat / 2)
             + cos(lat1rd) * cos(lat2rd) * sin(dlng / 2) * sin(dlng / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return (float)(EARTH_RADIUS * c);
}

float Navigator::bearingTo(double lat1, double lng1,
                            double lat2, double lng2) {
    // Étape 1 : convertir en radians (déjà dans ton squelette)
    float lat1rd = toRad(lat1);
    float lng1rd = toRad(lng1);
    float lat2rd = toRad(lat2);
    float lng2rd = toRad(lng2);

    // Étape 2 : Δlng
    double dlng = lng2rd - lng1rd;

    // Étape 3 : composantes x et y
    double x = cos(lat2rd) * sin(dlng);
    double y = cos(lat1rd) * sin(lat2rd)
             - sin(lat1rd) * cos(lat2rd) * cos(dlng);

    // Étape 4 : atan2 → degrés
    double bearing = atan2(x, y) * 180.0 / M_PI;

    // Étape 5 : normaliser dans [0, 360[
    return fmod((bearing + 360.0), 360.0);
}

void Navigator::loadWaypoints(double* lats, double* lngs, uint8_t count) {
    // Copier les waypoints dans _lats et _lngs
    // Ne pas dépasser MAX_WP
    // Mettre à jour _count
    // Appeler resetRoute()
}

float Navigator::distanceToNext(double current_lat, double current_lng) {
    // Si route complète, retourner 0
    // Sinon appeler distanceTo vers _lats[_currentIndex]
}


bool Navigator::isRouteComplete() {
    return _currentIndex >= _count;
}

void Navigator::resetRoute() {
    // Remettre _currentIndex à 0
}

bool Navigator::updateNavigation(double cur_lat, double cur_lng, float threshold) {
    if (isRouteComplete() || _count < 2) return false;

    // Vecteur du segment : WP[i] → WP[i+1]
    double seg_lat = _lats[_currentIndex] - _lats[_currentIndex - 1];  // si _currentIndex > 0
    double seg_lng = _lngs[_currentIndex] - _lngs[_currentIndex - 1];

    // Vecteur position actuelle : WP[i] → rover
    double pos_lat = cur_lat - _lats[_currentIndex - 1];
    double pos_lng = cur_lng - _lngs[_currentIndex - 1];

    // Produit scalaire normalisé → t = progression sur le segment [0.0 ... 1.0+]
    double seg_len_sq = seg_lat * seg_lat + seg_lng * seg_lng;
    double t = (seg_len_sq > 0)
               ? (pos_lat * seg_lat + pos_lng * seg_lng) / seg_len_sq
               : 0.0;

    // Rover a dépassé le WP → segment suivant
    if (t >= 1.0) {
        _currentIndex++;
        return true;
    }

    // Fallback : distance brute (si le rover est très loin de la trajectoire)
    if (distanceToNext(cur_lat, cur_lng) < threshold) {
        _currentIndex++;
        return true;
    }

    return false;
}

uint8_t Navigator::getCurrentWaypointIndex() {
    return _currentIndex;
}