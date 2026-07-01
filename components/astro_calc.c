/*
 * Sun and Moon horizontal position calculator
 *
 * Calculate:
 *   - altitude angle
 *   - azimuth angle
 *   - hour angle
 *
 * C99
 *
 * Compile:
 *   gcc astro_horizontal.c -lm -o astro_horizontal
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

 #include <stdio.h>
 #include <math.h>
 #include <stdbool.h>

 #include "astro_calc.h"
 
 #ifndef M_PI
 #define M_PI 3.14159265358979323846
 #endif
 
 #define DEG2RAD (M_PI / 180.0)
 #define RAD2DEG (180.0 / M_PI)
 

 static double fix_angle(double a)
 {
     a = fmod(a, 360.0);
     if (a < 0.0) a += 360.0;
     return a;
 }
 
 static double fix_hour_angle(double a)
 {
     a = fmod(a + 180.0, 360.0);
     if (a < 0.0) a += 360.0;
     return a - 180.0;
 }
 
 /*
  * Julian Day from calendar date and UTC hour.
  *
  * hour_utc can be outside 0~24.
  * For example:
  *   local 01:00, timezone +8 => UTC hour = -7
  */
 static double julian_day(int year, int month, int day, double hour_utc)
 {
     if (month <= 2) {
         year -= 1;
         month += 12;
     }
 
     int A = year / 100;
     int B = 2 - A + A / 4;
 
     double JD = floor(365.25 * (year + 4716))
               + floor(30.6001 * (month + 1))
               + day + B - 1524.5
               + hour_utc / 24.0;
 
     return JD;
 }
 
 /*
  * Greenwich Mean Sidereal Time in degrees.
  */
 static double gmst_deg(double jd)
 {
     double T = (jd - 2451545.0) / 36525.0;
 
     double gmst = 280.46061837
                 + 360.98564736629 * (jd - 2451545.0)
                 + 0.000387933 * T * T
                 - T * T * T / 38710000.0;
 
     return fix_angle(gmst);
 }
 
 /*
  * Low precision solar right ascension and declination.
  *
  * Output:
  *   ra_deg  : right ascension, degrees, 0~360
  *   dec_deg : declination, degrees, -90~+90
  */
 static void sun_ra_dec(double jd, double *ra_deg, double *dec_deg)
 {
     double n = jd - 2451545.0;
 
     double L = fix_angle(280.460 + 0.9856474 * n);
     double g = fix_angle(357.528 + 0.9856003 * n);
 
     double lambda = L
                   + 1.915 * sin(g * DEG2RAD)
                   + 0.020 * sin(2.0 * g * DEG2RAD);
 
     lambda = fix_angle(lambda);
 
     double epsilon = 23.439 - 0.0000004 * n;
 
     double lambda_rad = lambda * DEG2RAD;
     double eps_rad = epsilon * DEG2RAD;
 
     double x = cos(lambda_rad);
     double y = cos(eps_rad) * sin(lambda_rad);
     double z = sin(eps_rad) * sin(lambda_rad);
 
     double ra = atan2(y, x) * RAD2DEG;
     double dec = asin(z) * RAD2DEG;
 
     *ra_deg = fix_angle(ra);
     *dec_deg = dec;
 }
 
 /*
  * Approximate lunar right ascension and declination.
  *
  * Output:
  *   ra_deg     : right ascension, degrees, 0~360
  *   dec_deg    : declination, degrees, -90~+90
  *   dist_earth : distance in Earth radii
  */
 static void moon_ra_dec(double jd,
                         double *ra_deg,
                         double *dec_deg,
                         double *dist_earth)
 {
     double d = jd - 2451543.5;
 
     /*
      * Simplified lunar orbital elements.
      */
     double N = fix_angle(125.1228 - 0.0529538083 * d);   // longitude of ascending node
     double i = 5.1454;                                   // inclination
     double w = fix_angle(318.0634 + 0.1643573223 * d);   // argument of perigee
     double a = 60.2666;                                  // mean distance, Earth radii
     double e = 0.054900;                                 // eccentricity
     double M = fix_angle(115.3654 + 13.0649929509 * d);  // mean anomaly
 
     double Mrad = M * DEG2RAD;
 
     /*
      * Approximate solution of Kepler's equation.
      */
     double E = M + RAD2DEG * e * sin(Mrad) * (1.0 + e * cos(Mrad));
     double Erad = E * DEG2RAD;
 
     double xv = a * (cos(Erad) - e);
     double yv = a * (sqrt(1.0 - e * e) * sin(Erad));
 
     double v = atan2(yv, xv) * RAD2DEG;
     double r = sqrt(xv * xv + yv * yv);
 
     double lon = fix_angle(v + w);
 
     double Nr = N * DEG2RAD;
     double ir = i * DEG2RAD;
     double lonr = lon * DEG2RAD;
 
     /*
      * Lunar ecliptic rectangular coordinates.
      */
     double xh = r * (cos(Nr) * cos(lonr)
               - sin(Nr) * sin(lonr) * cos(ir));
 
     double yh = r * (sin(Nr) * cos(lonr)
               + cos(Nr) * sin(lonr) * cos(ir));
 
     double zh = r * (sin(lonr) * sin(ir));
 
     /*
      * Convert ecliptic coordinates to equatorial coordinates.
      */
     double eps = (23.4393 - 3.563e-7 * d) * DEG2RAD;
 
     double xe = xh;
     double ye = yh * cos(eps) - zh * sin(eps);
     double ze = yh * sin(eps) + zh * cos(eps);
 
     double ra = atan2(ye, xe) * RAD2DEG;
     double dec = atan2(ze, sqrt(xe * xe + ye * ye)) * RAD2DEG;
 
     *ra_deg = fix_angle(ra);
     *dec_deg = dec;
     *dist_earth = r;
 }
 
 /*
  * Convert equatorial coordinates to horizontal coordinates.
  *
  * Input:
  *   jd            : Julian Day
  *   latitude_deg  : observer latitude
  *   longitude_deg : observer longitude
  *   ra_deg        : right ascension
  *   dec_deg       : declination
  *
  * Output:
  *   AstroHorizontal:
  *     altitude_deg
  *     azimuth_deg
  *     hour_angle_deg
  */
 static AstroHorizontal horizontal_position_deg(double jd,
                                                double latitude_deg,
                                                double longitude_deg,
                                                double ra_deg,
                                                double dec_deg)
 {
     AstroHorizontal result;
 
     /*
      * Local Sidereal Time.
      */
     double lst = fix_angle(gmst_deg(jd) + longitude_deg);
 
     /*
      * Hour angle.
      *
      * H < 0 : object is east of local meridian
      * H > 0 : object is west of local meridian
      */
     double H = fix_hour_angle(lst - ra_deg);
 
     double lat = latitude_deg * DEG2RAD;
     double dec = dec_deg * DEG2RAD;
     double Hrad = H * DEG2RAD;
 
     /*
      * Altitude.
      */
     double sin_alt = sin(lat) * sin(dec)
                    + cos(lat) * cos(dec) * cos(Hrad);
 
     if (sin_alt > 1.0) sin_alt = 1.0;
     if (sin_alt < -1.0) sin_alt = -1.0;
 
     double alt = asin(sin_alt);
 
     /*
      * Azimuth.
      *
      * This formula returns:
      *   0 deg   = North
      *   90 deg  = East
      *   180 deg = South
      *   270 deg = West
      */
     double az = atan2(
         -sin(Hrad),
         tan(dec) * cos(lat) - sin(lat) * cos(Hrad)
     );
 
     result.altitude_deg = alt * RAD2DEG;
     result.azimuth_deg = fix_angle(az * RAD2DEG);
     result.hour_angle_deg = H;
 
     return result;
 }
 
 /*
  * Calculate Sun horizontal position by local time.
  */
 AstroHorizontal calculate_sun_horizontal(int year, int month, int day,
                                          int hour, int minute, int second,
                                          double latitude_deg,
                                          double longitude_deg,
                                          double timezone)
 {
     double local_hour = hour + minute / 60.0 + second / 3600.0;
     double utc_hour = local_hour - timezone;
 
     double jd = julian_day(year, month, day, utc_hour);
 
     double ra, dec;
     sun_ra_dec(jd, &ra, &dec);
 
     return horizontal_position_deg(
         jd,
         latitude_deg,
         longitude_deg,
         ra,
         dec
     );
 }
 
 /*
  * Calculate Moon horizontal position by local time.
  */
 AstroHorizontal calculate_moon_horizontal(int year, int month, int day,
                                           int hour, int minute, int second,
                                           double latitude_deg,
                                           double longitude_deg,
                                           double timezone)
 {
     double local_hour = hour + minute / 60.0 + second / 3600.0;
     double utc_hour = local_hour - timezone;
 
     double jd = julian_day(year, month, day, utc_hour);
 
     double ra, dec, dist;
     moon_ra_dec(jd, &ra, &dec, &dist);
 
     return horizontal_position_deg(
         jd,
         latitude_deg,
         longitude_deg,
         ra,
         dec
     );
 }
 
 /*
  * Compatibility wrapper:
  * only return Sun altitude.
  */
 double calculate_sun_altitude(int year, int month, int day,
                               int hour, int minute, int second,
                               double latitude_deg,
                               double longitude_deg,
                               double timezone)
 {
     AstroHorizontal sun = calculate_sun_horizontal(
         year, month, day,
         hour, minute, second,
         latitude_deg,
         longitude_deg,
         timezone
     );
 
     return sun.altitude_deg;
 }
 
 /*
  * Compatibility wrapper:
  * only return Moon altitude.
  */
 double calculate_moon_altitude(int year, int month, int day,
                                int hour, int minute, int second,
                                double latitude_deg,
                                double longitude_deg,
                                double timezone)
 {
     AstroHorizontal moon = calculate_moon_horizontal(
         year, month, day,
         hour, minute, second,
         latitude_deg,
         longitude_deg,
         timezone
     );
 
     return moon.altitude_deg;
 }
 
 const char *east_west_side(double hour_angle_deg)
 {
     if (hour_angle_deg < -0.001) {
         return "East side / rising side";
     } else if (hour_angle_deg > 0.001) {
         return "West side / setting side";
     } else {
         return "Meridian / transit";
     }
 }
 
 const char *visible_state(double altitude_deg)
 {
     if (altitude_deg > 0.0) {
         return "Above horizon";
     } else {
         return "Below horizon";
     }
 }


 double clamp_double(double x, double min_v, double max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

/*
 * Calculate the absolute hour angle of sunrise/sunset.
 *
 * return:
 *   H0 in degrees
 *
 * special:
 *   return -1.0 if sun never rises
 *   return -2.0 if sun never sets
 */
double sun_rise_set_hour_angle_deg(double latitude_deg, double sun_dec_deg)
{
    double lat = latitude_deg * DEG2RAD;
    double dec = sun_dec_deg * DEG2RAD;

    /*
     * Standard sunrise/sunset apparent altitude.
     */
    double h0 = -0.833 * DEG2RAD;

    double cosH0 = (sin(h0) - sin(lat) * sin(dec)) /
                   (cos(lat) * cos(dec));

    if (cosH0 > 1.0) {
        return -1.0;  // polar night / sun never rises
    }

    if (cosH0 < -1.0) {
        return -2.0;  // polar day / sun never sets
    }

    return acos(cosH0) * RAD2DEG;
}

/*
 * Convert Sun horizontal position to ideal semicircle angle.
 *
 * output:
 *   0 deg   = sunrise horizon side
 *   90 deg  = solar noon / highest point
 *   180 deg = sunset horizon side
 *
 * return:
 *   true  : valid
 *   false : invalid, for example polar day/night
 */
bool calculate_sun_arc_angle(int year, int month, int day,
                             int hour, int minute, int second,
                             double latitude_deg,
                             double longitude_deg,
                             double timezone,
                             double *arc_angle_deg)
{
    double local_hour = hour + minute / 60.0 + second / 3600.0;
    double utc_hour = local_hour - timezone;

    double jd = julian_day(year, month, day, utc_hour);

    double ra, dec;
    sun_ra_dec(jd, &ra, &dec);

    AstroHorizontal sun = horizontal_position_deg(
        jd,
        latitude_deg,
        longitude_deg,
        ra,
        dec
    );

    double H0 = sun_rise_set_hour_angle_deg(latitude_deg, dec);

    if (H0 < 0.0) {
        /*
         * -1.0: sun never rises
         * -2.0: sun never sets
         */
        *arc_angle_deg = 0.0;
        return false;
    }

    /*
     * Map:
     *   H = -H0 -> 0 deg
     *   H = 0   -> 90 deg
     *   H = +H0 -> 180 deg
     */
    double arc = (sun.hour_angle_deg + H0) / (2.0 * H0) * 180.0;

    /*
     * Clamp to visible semicircle.
     * When sun is before sunrise, arc < 0.
     * When sun is after sunset, arc > 180.
     */
    arc = clamp_double(arc, 0.0, 180.0);

    *arc_angle_deg = arc;
    return true;
}

double smoothstep(double edge0, double edge1, double x)
{
    double t = (x - edge0) / (edge1 - edge0);
    t = clamp_double(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

float smoothstepf(float edge0, float edge1, float x)
{
    float t = (x - edge0) / (edge1 - edge0);
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/*
 * 根据太阳高度角粗略计算色温。
 *
 * input:
 *   altitude_deg: 太阳高度角，范围约 -90 ~ +90
 *
 * output:
 *   color temperature in Kelvin
 */
double sun_cct_from_altitude(double altitude_deg)
{
    /*
     * -6° 认为是民用曙暮光边界
     * 45° 以上认为接近白天高色温
     */
    double t = smoothstep(-6.0, 45.0, altitude_deg);

    double cct_min = 1800.0;
    double cct_max = 6500.0;

    return cct_min + t * (cct_max - cct_min);
}

double sun_cct_from_altitude_piecewise(double altitude_deg)
{
    if (altitude_deg <= -6.0) {
        return 1800.0;
    }

    if (altitude_deg < 0.0) {
        /*
         * 黎明 / 黄昏：1800K ~ 2200K
         */
        double t = smoothstep(-6.0, 0.0, altitude_deg);
        return 1800.0 + t * (2200.0 - 1800.0);
    }

    if (altitude_deg < 5.0) {
        /*
         * 刚日出 / 快日落：2200K ~ 2800K
         */
        double t = smoothstep(0.0, 5.0, altitude_deg);
        return 2200.0 + t * (2800.0 - 2200.0);
    }

    if (altitude_deg < 15.0) {
        /*
         * 低角度阳光：2800K ~ 4000K
         */
        double t = smoothstep(5.0, 15.0, altitude_deg);
        return 2800.0 + t * (4000.0 - 2800.0);
    }

    if (altitude_deg < 45.0) {
        /*
         * 上午 / 下午：4000K ~ 5500K
         */
        double t = smoothstep(15.0, 45.0, altitude_deg);
        return 4000.0 + t * (5500.0 - 4000.0);
    }

    /*
     * 正午附近
     */
    double t = smoothstep(45.0, 75.0, altitude_deg);
    return 5500.0 + t * (6500.0 - 5500.0);
}

double sun_cct_with_morning_evening(double altitude_deg, double hour_angle_deg)
{
    double cct = sun_cct_from_altitude_piecewise(altitude_deg);

    /*
     * 日落侧额外降低色温，让黄昏更暖。
     */
    if (hour_angle_deg > 0.0 && altitude_deg < 10.0) {
        double t = smoothstep(10.0, 0.0, altitude_deg);
        cct -= 300.0 * t;
    }

    if (cct < 1500.0) cct = 1500.0;
    if (cct > 6500.0) cct = 6500.0;

    return cct;
}

double sun_brightness_from_altitude(double altitude_deg)
{
    /*
     * -6° 到 45° 映射到 0~1
     */
    double b = smoothstep(-6.0, 45.0, altitude_deg);
    return b;
}
