#ifndef __ASTRO_CALC_H__
#define __ASTRO_CALC_H__

#include <stdbool.h>

/*
 * Horizontal position of a celestial body.
 *
 * Coordinate convention:
 *   Latitude  : north positive, south negative
 *   Longitude : east positive, west negative
 *   Timezone  : UTC offset, e.g. China/Singapore = +8.0
 *
 * Azimuth convention:
 *   0 deg   = North
 *   90 deg  = East
 *   180 deg = South
 *   270 deg = West
 */
typedef struct {
    double altitude_deg;    // -90 ~ +90, above horizon is positive
    double azimuth_deg;     // 0 ~ 360, 0=N, 90=E, 180=S, 270=W
    double hour_angle_deg;  // -180 ~ +180, negative=east side, positive=west side
} AstroHorizontal;

/* Sun / Moon horizontal position (altitude, azimuth, hour angle) */
AstroHorizontal calculate_sun_horizontal(int year, int month, int day,
                                         int hour, int minute, int second,
                                         double latitude_deg,
                                         double longitude_deg,
                                         double timezone);

AstroHorizontal calculate_moon_horizontal(int year, int month, int day,
                                          int hour, int minute, int second,
                                          double latitude_deg,
                                          double longitude_deg,
                                          double timezone);

/* Compatibility wrappers: altitude only */
double calculate_sun_altitude(int year, int month, int day,
                              int hour, int minute, int second,
                              double latitude_deg,
                              double longitude_deg,
                              double timezone);

double calculate_moon_altitude(int year, int month, int day,
                               int hour, int minute, int second,
                               double latitude_deg,
                               double longitude_deg,
                               double timezone);

/*
 * Map Sun position to ideal semicircle angle:
 *   0 deg   = sunrise horizon
 *   90 deg  = solar noon
 *   180 deg = sunset horizon
 *
 * Returns false for polar day/night (arc_angle_deg set to 0).
 */
bool calculate_sun_arc_angle(int year, int month, int day,
                             int hour, int minute, int second,
                             double latitude_deg,
                             double longitude_deg,
                             double timezone,
                             double *arc_angle_deg);

/* Color temperature and brightness derived from Sun altitude */
double sun_cct_from_altitude(double altitude_deg);
double sun_cct_from_altitude_piecewise(double altitude_deg);
double sun_cct_with_morning_evening(double altitude_deg, double hour_angle_deg);
double sun_brightness_from_altitude(double altitude_deg);

const char *east_west_side(double hour_angle_deg);
const char *visible_state(double altitude_deg);
double clamp_double(double x, double min_v, double max_v);
float clampf(float x, float min_v, float max_v);
double smoothstep(double edge0, double edge1, double x);
float smoothstepf(float edge0, float edge1, float x);


#endif
