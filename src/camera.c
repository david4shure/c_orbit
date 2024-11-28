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

void spherical_camera_system_locked(DVector3 offset, float *r, float *theta, float *phi, Camera3D *camera) {
    float scale = 2.0;
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        scale = 0.1;
    }

    Vector2 mousePositionDelta = GetMouseDelta();
    float mouseWheelMove = GetMouseWheelMove();

    Debug("mousePositionDelta=(%.2f,%.2f), mouseWheelMove=%.2f\n",mousePositionDelta.x,mousePositionDelta.y,mouseWheelMove);
    Debug("Before r=%.2f, theta = %.2f, phi = %.2f\n",*r,*theta,*phi);

    *r -= mouseWheelMove * 0.001;

    float r_delta = mouseWheelMove * SCROLL_FACTOR;

    *r -= scale * r_delta * *r;

    *theta -= scale * mousePositionDelta.x*0.005;
    *phi -= scale * mousePositionDelta.y*0.005;

    *phi = clampf(*phi,PI/2 + CAMERA_ETA,3 * PI/2-CAMERA_ETA);

    Debug("After r=%.2f, theta = %.2f, phi = %.2f\n",*r,*theta,*phi);

    Debug("offset = (%.2f,%.2f,%.2f)\n",offset.x,offset.y,offset.z);

    camera->position.x = offset.x + *r*sin(*theta)*cos(*phi);
    camera->position.y = offset.y + *r*sin(*phi);
    camera->position.z = offset.z + *r*cos(*theta)*cos(*phi);
}
void spherical_camera_system_click_to_drag(DVector3 offset, float *r, float *theta, float *phi, Camera3D *camera) {
    float scale = 2.0;
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        scale = 0.1;
    }

    Vector2 mousePositionDelta = GetMouseDelta();
    float mouseWheelMove = GetMouseWheelMove();

    Debug("mousePositionDelta=(%.2f,%.2f), mouseWheelMove=%.2f\n",mousePositionDelta.x,mousePositionDelta.y,mouseWheelMove);
    Debug("Before r=%.2f, theta = %.2f, phi = %.2f\n",*r,*theta,*phi);

    *r -= mouseWheelMove * 0.001;

    float r_delta = mouseWheelMove * SCROLL_FACTOR;
    
    *r -= scale * r_delta * *r;
    
    Debug("offset = (%.2f,%.2f,%.2f)\n",offset.x,offset.y,offset.z);

    camera->position.x = offset.x + *r*sin(*theta)*cos(*phi);
    camera->position.y = offset.y + *r*sin(*phi);
    camera->position.z = offset.z + *r*cos(*theta)*cos(*phi);

    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        return;
    }

    *theta -= scale * mousePositionDelta.x*0.005;
    *phi -= scale * mousePositionDelta.y*0.005;

    *phi = clampf(*phi,PI/2 + CAMERA_ETA,3 * PI/2-CAMERA_ETA);

    Debug("After r=%.2f, theta = %.2f, phi = %.2f\n",*r,*theta,*phi);

    Debug("offset = (%.2f,%.2f,%.2f)\n",offset.x,offset.y,offset.z);
    camera->position.x = offset.x + *r*sin(*theta)*cos(*phi);
    camera->position.y = offset.y + *r*sin(*phi);
    camera->position.z = offset.z + *r*cos(*theta)*cos(*phi);

    camera->target = (Vector3){offset.x,offset.y,offset.z};
    camera->up = (Vector3){0.0,1.0,0.0};
}

void spherical_camera_system(DVector3 offset, float *r, float *theta, float *phi, Camera3D *camera, bool is_locked) {
    Debug("R = %.3f\n",*r);
    // If the camera is locked process any inputs regardless
    if (is_locked) {
        spherical_camera_system_locked(offset, r,theta,phi,camera);
    } else {
        spherical_camera_system_click_to_drag(offset, r,theta,phi,camera);
    }
}

