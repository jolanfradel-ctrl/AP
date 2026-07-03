#pragma once
#include <stdint.h>

class Navigator {
public:
    // Constructeur
    Navigator();

    // --- CALCULS GEOMETRIQUES ---

    // Retourne la distance en mètres entre deux points GPS
    float distanceTo(double lat1, double lng1,
                     double lat2, double lng2);

    // Retourne le cap en degrés (0° = Nord, 90° = Est)
    // vers le point 2 depuis le point 1
    float bearingTo(double lat1, double lng1,
                    double lat2, double lng2);

    // --- GESTION DU PARCOURS ---

    // Charge les waypoints reçus depuis la DB
    void loadWaypoints(double* lats, double* lngs, uint8_t count);

    // Retourne l'index du waypoint actuel
    uint8_t getCurrentWaypointIndex();

    // Retourne la distance au prochain waypoint (mètres)
    float distanceToNext(double current_lat, double current_lng);

 

    // Retourne true si tous les waypoints ont été atteints
    bool isRouteComplete();

    // Remet la navigation au début
    void resetRoute();
    bool updateNavigation(double cur_lat, double cur_lng, float threshold);

private:
    // Conversion degrés → radians
    float toRad(double deg);

    // Données internes
    static const uint8_t MAX_WP = 20;
    double _lats[MAX_WP];
    double _lngs[MAX_WP];
    uint8_t _count;
    uint8_t _currentIndex;

    // Rayon de la Terre en mètres
    static constexpr float EARTH_RADIUS = 6371000.0f;
};