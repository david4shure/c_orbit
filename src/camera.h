#pragma once

#ifndef RAYLIB_H
#include "raylib.h"
#endif

#ifndef CAMERAH
#define CAMERAH

#include "tree.h"

void spherical_camera_system(DVector3 offset, float* r, float* theta, float* phi, Camera3D* camera, bool is_locked);

#endif
