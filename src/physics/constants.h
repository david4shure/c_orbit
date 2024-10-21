#pragma once

#ifndef PHYSICAL_CONSTANTS
#define PHYSICAL_CONSTANTS

// Universal gravitational constant
static const float G = 6.67430e-20; // km3kg−1s−2

// Earth mass kg
static const double EARTH_MASS_KG = 5.972e+24;

// Moon mass kg
static const double MOON_MASS_KG = 7.348e+22;

// Sun mass kg
static const double SUN_MASS_KG = 1.989e+30;

// 20 km to 1 render unit
static const float RENDER_UNITS_TO_KM = 1.0;

// 0.05 KM render units to a KM
static const float KM_TO_RENDER_UNITS = 1/RENDER_UNITS_TO_KM;

// Earth radius km
static const float EARTH_RADIUS_KM = 6378.137;
static const float MOON_RADIUS_KM = 1737.4;
static const float EARTH_SEMIMAJOR_AXIS_KM = 149597870.7;

#endif // PHYSICAL_CONSTANTS
