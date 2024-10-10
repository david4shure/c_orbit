#include "camera.h"
#include "raylib.h"
#include "rlutil.h"
#include "math.h"

#ifdef __APPLE__
#define SCROLL_FACTOR 0.01
#else
#define SCROLL_FACTOR 0.1
#endif

#define CAMERA_ETA 0.000001

void SphericalCameraSystem(float *r, float *theta, float *phi, Camera3D *camera) {
    Vector2 mousePositionDelta = GetMouseDelta();
    float mouseWheelMove = GetMouseWheelMove();

    *r -= mouseWheelMove * 0.001;

    float r_delta = mouseWheelMove * SCROLL_FACTOR;
    
    *r -= r_delta * *r;
    *theta -= mousePositionDelta.x*0.005f;
    *phi += mousePositionDelta.y*0.005f;

    *phi = clampf(*phi,-PI/2+CAMERA_ETA,PI/2.0-CAMERA_ETA);

    camera->position.x = *r*sin(*theta)*cos(*phi);
    camera->position.y = *r*sin(*phi);
    camera->position.z = *r*cos(*theta)*cos(*phi);
}
