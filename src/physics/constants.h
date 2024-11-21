#pragma once

#ifndef PHYSICAL_CONSTANTS
#define PHYSICAL_CONSTANTS

static const double D_PI = 3.1415926535897932384626433832795028841917f;

static const double D_RAD2DEG = 180.0f/D_PI;

static const double D_DEG2RAD = D_PI/180.0f;

// Universal gravitational constant
static const double  G = 6.67430e-20; // km3kg−1s−2

// Earth mass kg
static const double EARTH_MASS_KG = 5.972161874653522e24;

// Moon mass kg
static const double MOON_MASS_KG = 7.348e+22;

// Sun mass kg
static const double SUN_MASS_KG = 1.989e+30;

// 20 km to 1 render unit
static const double RENDER_UNITS_TO_KM = 200.0;

// 0.05 KM render units to a KM
static const double KM_TO_RENDER_UNITS = 1/RENDER_UNITS_TO_KM;

// 20 km to 1 render unit
static const double RENDER_UNITS_TO_KM_2D = 1.0;

// 0.05 KM render units to a KM
static const double KM_TO_RENDER_UNITS_2D = 1/RENDER_UNITS_TO_KM_2D;

// Earth radius km
static const double EARTH_RADIUS_KM = 6378.137;
static const double MOON_RADIUS_KM = 1737.4;
static const double EARTH_SEMIMAJOR_AXIS_KM = 149597870.7;

static const int SECONDS_IN_MINUTE = 60;
static const int MINUTES_IN_HOUR = 60;
static const int HOURS_IN_DAY = 24;
static const int DAYS_IN_WEEK = 7;
static const int DAYS_IN_YEAR = 365;

// Calculate seconds in each time frame
static const int SECONDS_IN_HOUR = SECONDS_IN_MINUTE * MINUTES_IN_HOUR;
static const int SECONDS_IN_DAY = SECONDS_IN_HOUR * HOURS_IN_DAY;
static const int SECONDS_IN_WEEK = SECONDS_IN_DAY * DAYS_IN_WEEK;
static const int SECONDS_IN_YEAR = SECONDS_IN_DAY * DAYS_IN_YEAR;

#endif // PHYSICAL_CONSTANTS
