#include "camera.h"
#include "raylib.h"
#include "utils/rlutil.h"
#include "math.h"
#include "utils/logger.h"

// Enhanced camera sensitivity and trackpad support
#define CAMERA_ETA 0.01
#define MOUSE_SENSITIVITY 0.003f
#define TRACKPAD_SENSITIVITY 0.001f
#define ZOOM_SENSITIVITY 0.3f
#define TRACKPAD_ZOOM_SENSITIVITY 0.1f

// Trackpad gesture detection
static float last_zoom_time = 0.0f;
static float zoom_rate_limit = 0.02f; // Minimum time between zoom events - faster response

void spherical_camera_system_locked(DVector3 offset, float *r, float *theta, float *phi, Camera3D *camera) {
    float scale = 1.0f;
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        scale = 0.3f;
    }

    Vector2 mousePositionDelta = GetMouseDelta();
    Vector2 mouseWheelMoveV = GetMouseWheelMoveV();
    float mouseWheelMove = GetMouseWheelMove();

    // Enhanced zoom handling with trackpad support
    float current_time = GetTime();
    if (current_time - last_zoom_time > zoom_rate_limit) {
        float zoom_delta = 0.0f;
        
        // Handle trackpad gestures (vertical scroll)
        if (mouseWheelMoveV.y != 0) {
            zoom_delta = mouseWheelMoveV.y * TRACKPAD_ZOOM_SENSITIVITY;
            last_zoom_time = current_time;
        }
        // Handle mouse wheel
        else if (mouseWheelMove != 0) {
            zoom_delta = mouseWheelMove * ZOOM_SENSITIVITY;
            last_zoom_time = current_time;
        }
        
        if (zoom_delta != 0.0f) {
            *r -= scale * zoom_delta * *r;
            *r = fmaxf(*r, 100.0f); // Much closer zoom - 10x closer
            *r = fminf(*r, 10000000.0f); // Allow much further zoom out
        }
    }

    // Enhanced mouse movement with sensitivity adjustment
    float sensitivity = MOUSE_SENSITIVITY;
    if (mouseWheelMoveV.y != 0 || mouseWheelMoveV.x != 0) {
        // Likely trackpad input, use different sensitivity
        sensitivity = TRACKPAD_SENSITIVITY;
    }

    *theta -= scale * mousePositionDelta.x * sensitivity;
    *phi -= scale * mousePositionDelta.y * sensitivity;

    *phi = clampf(*phi, PI/2 + CAMERA_ETA, 3 * PI/2 - CAMERA_ETA);

    camera->position.x = offset.x + *r * sin(*theta) * cos(*phi);
    camera->position.y = offset.y + *r * sin(*phi);
    camera->position.z = offset.z + *r * cos(*theta) * cos(*phi);
}

void spherical_camera_system_click_to_drag(DVector3 offset, float *r, float *theta, float *phi, Camera3D *camera) {
    float scale = 1.0f;
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        scale = 0.3f;
    }

    Vector2 mousePositionDelta = GetMouseDelta();
    Vector2 mouseWheelMoveV = GetMouseWheelMoveV();
    float mouseWheelMove = GetMouseWheelMove();

    // Enhanced zoom handling with trackpad support
    float current_time = GetTime();
    if (current_time - last_zoom_time > zoom_rate_limit) {
        float zoom_delta = 0.0f;
        
        // Handle trackpad gestures (vertical scroll)
        if (mouseWheelMoveV.y != 0) {
            zoom_delta = mouseWheelMoveV.y * TRACKPAD_ZOOM_SENSITIVITY;
            last_zoom_time = current_time;
        }
        // Handle mouse wheel
        else if (mouseWheelMove != 0) {
            zoom_delta = mouseWheelMove * ZOOM_SENSITIVITY;
            last_zoom_time = current_time;
        }
        
        if (zoom_delta != 0.0f) {
            *r -= scale * zoom_delta * *r;
            *r = fmaxf(*r, 100.0f); // Much closer zoom - 10x closer
            *r = fminf(*r, 10000000.0f); // Allow much further zoom out
        }
    }
    
    camera->position.x = offset.x + *r * sin(*theta) * cos(*phi);
    camera->position.y = offset.y + *r * sin(*phi);
    camera->position.z = offset.z + *r * cos(*theta) * cos(*phi);

    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        return;
    }

    // Enhanced mouse movement with sensitivity adjustment
    float sensitivity = MOUSE_SENSITIVITY;
    if (mouseWheelMoveV.y != 0 || mouseWheelMoveV.x != 0) {
        // Likely trackpad input, use different sensitivity
        sensitivity = TRACKPAD_SENSITIVITY;
    }

    *theta -= scale * mousePositionDelta.x * sensitivity;
    *phi -= scale * mousePositionDelta.y * sensitivity;

    *phi = clampf(*phi, PI/2 + CAMERA_ETA, 3 * PI/2 - CAMERA_ETA);

    camera->position.x = offset.x + *r * sin(*theta) * cos(*phi);
    camera->position.y = offset.y + *r * sin(*phi);
    camera->position.z = offset.z + *r * cos(*theta) * cos(*phi);

    camera->target = (Vector3){offset.x, offset.y, offset.z};
    camera->up = (Vector3){0.0, 1.0, 0.0};
}

void spherical_camera_system(DVector3 offset, float *r, float *theta, float *phi, Camera3D *camera, bool is_locked) {
    // If the camera is locked process any inputs regardless
    if (is_locked) {
        spherical_camera_system_locked(offset, r, theta, phi, camera);
    } else {
        spherical_camera_system_click_to_drag(offset, r, theta, phi, camera);
    }
}

