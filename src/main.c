#include "camera.h"
#include "physics/propagation.h"
#include "time.h"
#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"
#include "rlgl.h"
#include <stdio.h>
#include "raygui.h"

#include "physics/constants.h"
#include "physics/time.h"
#include "physics/kepler.h"
#include "utils/rlutil.h"
#include "utils/darray.h"
#include "utils/logger.h"

#define LOG_LEVEL WARN

const double screenWidth = 1500;
const double screenHeight = 1000;

Vector3 ToRay(DVector3 vec) {
    return (Vector3){.x = (float)vec.x, .y = (float)vec.y, .z = (float)vec.z};
}

DVector2 perifocal_to_screenspace(DVector2 pq, double screen_width, double screen_height, double scale_factor) {
    // Convert from kilometers to render units
    double scaled_x = pq.x / RENDER_UNITS_TO_KM;
    double scaled_y = pq.y / RENDER_UNITS_TO_KM;

    // Apply the scale factor and center the coordinates on the screen
    double screen_x = (scaled_x * scale_factor) + (screen_width / 2);
    double screen_y = (scaled_y * scale_factor) + (screen_height / 2);

    return (DVector2){.x = screen_x, .y = screen_y};
}

void convert_and_debug(DVector2 pq, double screen_width, double screen_height) {
    // Use dynamic scaling factor based on a heuristic value
    double scale_factor = 0.001;  // Adjust based on orbit size

    DVector2 screen_point = perifocal_to_screenspace(pq, screen_width, screen_height, scale_factor);

    Debug("Screen point = (%f, %f)\n", screen_point.x, screen_point.y);
}

// Function to check if an object is behind the camera
bool is_object_behind_camera(DVector3 cam_pos, DVector3 cam_target, DVector3 obj_pos) {
    DVector3 camera_to_target = DVector3Normalize(DVector3Subtract(cam_pos,cam_target));
    DVector3 camera_to_object = DVector3Normalize(DVector3Subtract(cam_pos,obj_pos));

    float dot = DVector3DotProduct(camera_to_target, camera_to_object);

    return dot < 0;
}

// Bounding box to dynamically calculate scaling factor
DVector2 get_bounding_box(void* orbital_positions, OrbitalElements oe) {
    DVector2 min = {1e9, 1e9};
    DVector2 max = {-1e9, -1e9};
    
    for (int i = 0; i < darray_length(orbital_positions); i++) {
        DVector3* pos = darray_get(orbital_positions, i);
        DVector2 pq_coords = eci_coords_to_perifocal_coords(*pos, oe.long_of_asc_node, oe.arg_of_periapsis, oe.inclination);

        if (pq_coords.x < min.x) min.x = pq_coords.x;
        if (pq_coords.y < min.y) min.y = pq_coords.y;
        if (pq_coords.x > max.x) max.x = pq_coords.x;
        if (pq_coords.y > max.y) max.y = pq_coords.y;
    }
    
    return (DVector2){max.x - min.x, max.y - min.y};
}

double calculate_scale_factor(DVector2 bounding_box, double screen_width, double screen_height) {
    double scale_x = screen_width / bounding_box.x;
    double scale_y = screen_height / bounding_box.y;
    return fmin(scale_x, scale_y);
}

void draw_orbital_lines_2d(void* orbital_positions, OrbitalElements oe) {
    printf("Length of positions = %d\n", darray_length(orbital_positions));

    DVector3* prev_pos = NULL;
    DVector3* current_pos = NULL;

    DVector2 bounding_box = get_bounding_box(orbital_positions, oe);
    double scale_factor = calculate_scale_factor(bounding_box, screenWidth, screenHeight);

    for (int i = 0; i < darray_length(orbital_positions); i++) {
        if (i == 0) {
            prev_pos = darray_get(orbital_positions, i);
            continue;
        }

        current_pos = darray_get(orbital_positions, i);

        DVector2 pq_coords_curr = eci_coords_to_perifocal_coords(*current_pos, oe.long_of_asc_node, oe.arg_of_periapsis, oe.inclination);
        DVector2 pq_coords_prev = eci_coords_to_perifocal_coords(*prev_pos, oe.long_of_asc_node, oe.arg_of_periapsis, oe.inclination);

        DVector2 screen_point1 = perifocal_to_screenspace(pq_coords_curr, screenWidth, screenHeight, scale_factor);
        DVector2 screen_point2 = perifocal_to_screenspace(pq_coords_prev, screenWidth, screenHeight, scale_factor);

        float x1 = screen_point1.x, y1 = screen_point1.y;
        float x2 = screen_point2.x, y2 = screen_point2.y;

        DrawPixel(x1,y1,WHITE);
        /* DrawLine(x1, y1, x2, y2, BLUE); */

        prev_pos = current_pos;
    }
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    InitializeLogger(LOG_LEVEL);

    DVector3 moon_position = {86874.49762938, 316748.93677354, 172163.38271959};
    DVector3 moon_velocity = {-1.01153543, 0.0, 0.0};

    PhysicalState RV = {
        .r = moon_position,
        .v = moon_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG,
    };

    float M_naught = 2.35585;
    float t_naught = 0.0;
    OrbitalElements eles = orb_elems_from_rv(RV, 0.0, 0.0);

    PhysicsTimeClock clock = {.tick_interval_seconds = 86400, .mode = Elapsing, .scale = 100.0, .delta_seconds = 0.0, .clock_seconds = 0.0};

    InitWindow(screenWidth, screenHeight, "c_orbit");

    Camera camera = {0};
    camera.position = (Vector3){0.0f, 2.0f, 4.0f};
    camera.target = (Vector3){0.0f, 2.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Camera2D camera_2d = {.offset = (Vector2){screenWidth / 2, screenHeight / 2}, .rotation = 0.0, .zoom = 0.4};

    void* orbital_positions;

    while (!WindowShouldClose()) {
        RV.v = DVector3Scale(RV.v,1.0001);
        float delta = GetFrameTime();
        UpdatePhysicsClock(&clock, delta);

        orbital_positions = propagate_orbit(RV, 0.0, M_naught, t_naught, 10000000.0);
        eles = orb_elems_from_rv(RV, 0.0, 0.0);

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode2D(camera_2d);
        draw_orbital_lines_2d(orbital_positions, eles);
        EndMode2D();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

