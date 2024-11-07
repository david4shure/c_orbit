#include "camera.h"
#include "raylib.h"
#include "utils/rlutil.h"
#include "math.h"
#include "utils/logger.h"

#ifdef __APPLE__
#define SCROLL_FACTOR 0.01
#else
#define SCROLL_FACTOR 0.1
#endif

#define CAMERA_ETA 0.000001

void SphericalCameraSystem(float *r, float *theta, float *phi, Camera3D *camera) {
    Vector2 mousePositionDelta = GetMouseDelta();
    float mouseWheelMove = GetMouseWheelMove();

    float scale = 2.0;
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        scale = 0.1;
    }

    Debug("mousePositionDelta=(%.2f,%.2f), mouseWheelMove=%.2f\n",mousePositionDelta.x,mousePositionDelta.y,mouseWheelMove);
    Debug("Before r=%.2f, theta = %.2f, phi = %.2f\n",*r,*theta,*phi);

    *r -= mouseWheelMove * 0.001;

    float r_delta = mouseWheelMove * SCROLL_FACTOR;
    
    *r -= scale * r_delta * *r;
    *theta -= scale * mousePositionDelta.x*0.005;
    *phi -= scale * mousePositionDelta.y*0.005;

    *phi = clampf(*phi,PI/2 + CAMERA_ETA,3 * PI/2-CAMERA_ETA);

    Debug("After r=%.2f, theta = %.2f, phi = %.2f\n",*r,*theta,*phi);


    camera->position.x = *r*sin(*theta)*cos(*phi);
    camera->position.y = *r*sin(*phi);
    camera->position.z = *r*cos(*theta)*cos(*phi);
}
