#pragma once

#ifndef PHYSICAL_CONSTANTS
#define PHYSICAL_CONSTANTS

// 20 km to 1 render unit
static const float RENDER_UNITS_TO_KM = 2.0;

// 0.05 KM render units to a KM
static const float KM_TO_RENDER_UNITS = 1/RENDER_UNITS_TO_KM;

// Earth radius km
static const float EARTH_RADIUS_KM = 6378.137;
static const float MOON_RADIUS_KM = 1737.4;

#endif // PHYSICAL_CONSTANTS
