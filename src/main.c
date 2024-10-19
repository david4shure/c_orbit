/*******************************************************************************************
*
*   raylib [core] example - 3d camera first person
*
*   Example originally created with raylib 1.3, last time updated with raylib 1.3
*
*   Example licensed under an unmodified zlib/libpng license, which is an OSI-certified,
*   BSD-like license that allows static linking with closed source software
*
*   Copyright (c) 2015-2024 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

#include "camera.h"
#include "physics/propagation.h"
#include "time.h"
#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"
#include "rlgl.h"
#include <stdio.h>
#include "raygui.h"
#include "math.h"

#include "physics/constants.h"
#include "physics/time.h"
#include "physics/kepler.h"
#include "utils/rlutil.h"
#include "utils/darray.h"
#include "utils/logger.h"

#define LOG_LEVEL DEBUG

// Function to check if an object is behind the camera
bool is_object_behind_camera(Vector3 cam_pos, Vector3 cam_target, Vector3 obj_pos) {
    Vector3 camera_to_target = Vector3Normalize(Vector3Subtract(cam_pos,cam_target));
    Vector3 camera_to_object = Vector3Normalize(Vector3Subtract(cam_pos,obj_pos));

    float dot = Vector3DotProduct(camera_to_target, camera_to_object);

    return dot < 0;
}

void draw_orbital_lines(void* orbital_positions) {
    printf("Length of positions = %d\n",darray_length(orbital_positions));

    Vector3* prev_pos = NULL;
    Vector3* current_pos = NULL;

    // Now iterate over our darray and draw lines
    for (int i = 0; i < darray_length(orbital_positions); i++) {
        if (i==0) {
            prev_pos = darray_get(orbital_positions, i);
            continue;
        }

        current_pos = darray_get(orbital_positions, i);
        
        // Draw the dang line
        DrawLine3D(vector_from_physical_to_world(*current_pos), vector_from_physical_to_world(*prev_pos), BLUE);

        prev_pos = current_pos;
    }

    // Free our array
    //darray_free(orbital_positions);
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Logger
    InitializeLogger(LOG_LEVEL);

    // Approximate ECI position of the Moon (in km)
    Vector3 moon_position = { 200000.0f, -5000.0f, -100.0f };

    // Approximate ECI velocity of the Moon (in km/s)
    Vector3 moon_velocity = { 0.0, 0.0, 2.0};

    Log("Earth mass kg = %.2f\n",EARTH_MASS_KG);
    PhysicalState RV = {
        .r = moon_position,
        .v = moon_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG,
    };

    float M_naught = 2.35585;
    float t_naught = 0.0;
    OrbitalElements eles = orb_elems_from_rv(RV,M_naught,t_naught);

    print_orbital_elements(eles);

    print_physical_state(RV);

    Debug("Eccentricity = %.2f\n",eles.eccentricity);

    PhysicsTimeClock clock = { .tick_interval_seconds = 86400, .mode = Elapsing, .scale = 10000.0, .delta_seconds = 0.0, .clock_seconds = 0.0};

    //--------------------------------------------------------------------------------------
    const int screenWidth = 1500;
    const int screenHeight = 1000;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera first person");

    // Define the camera to look into our 3d world (position, target, up vector)
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 2.0f, 4.0f };    // Camera position
    camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 60.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    int cameraMode = CAMERA_FREE;
    float time = 0.0;

    // Spherical Camera coordinates
    float r = 100000.0;
    float theta = 0.0;
    float phi = 0.0;

    Matrix matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,10000000.0);

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float delta;
    print_physical_state(RV);
    float r_at_sphere_of_influence = calculate_sphere_of_influence_r(149597870.7, eles.mass_of_parent, eles.mass_of_grandparent);
    float r_at_soi_world_coords = r_at_sphere_of_influence * KM_TO_RENDER_UNITS;

    void* orbital_lines;

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        time = GetTime();
        delta = GetFrameTime();

        UpdatePhysicsClock(&clock, delta);

        orbital_lines = propagate_orbit(RV, 0.0, M_naught, t_naught);

        Debug("len(orbital_lines)=%d\n",darray_length(orbital_lines));

        RV = rv_from_r0v0(RV,clock.clock_seconds); 

        eles = orb_elems_from_rv(RV, M_naught, t_naught);
        print_orbital_elements(eles);
        moon_position = RV.r;
        moon_velocity = RV.v;

        Vector3 moon_pos_world = vector_from_physical_to_world(moon_position);
        // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
        float dist = Vector3Distance(moon_pos_world, camera.position);

        Vector3 t0_line = vector_from_physical_to_world(solve_kepler_ellipse_inertial(eles, 0.0, 0.0, 0.0));
        Vector3 t0_norm = Vector3Normalize(t0_line);
        Vector3 t0_farther_out = Vector3Add(t0_line, Vector3Scale(t0_norm,200));

        Vector2 t0_farther_out_world = GetWorldToScreen(t0_farther_out, camera);
        Vector2 moon = GetWorldToScreen(moon_pos_world, camera);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();


            ClearBackground(BLACK);

            SphericalCameraSystem(&r, &theta, &phi, &camera);

            BeginMode3D(camera);
                rlSetMatrixProjection(matProj);

                draw_orbital_lines(orbital_lines);

                darray_free(orbital_lines);
                Vector3 sphere_pos = {0.0,0.0,0.0};

                DrawSphereWires(sphere_pos,EARTH_RADIUS_KM * KM_TO_RENDER_UNITS,10,10,SKYBLUE);
                Color grid_color = { .r = 0, .g = 240, .b = 0, .a = 150};
                DrawGridOfColor(250,50000,grid_color); // Draw ground

                DrawSphereWires(moon_pos_world,(MOON_RADIUS_KM * KM_TO_RENDER_UNITS),10,10,GRAY);
                DrawSphereWires(sphere_pos,(r_at_soi_world_coords),10,10,PINK);
            EndMode3D();

            float max_distance = 5000.0;
            float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-moon_pos_world.x,camera.position.y-moon_pos_world.y,camera.position.z-moon_pos_world.z});
            float distance_scale_factor = max_distance/camera_to_moon_distance;
            float time_scale_factor = sin(time*1.5)/4.0 + 1;
            float scale_factor = distance_scale_factor * time_scale_factor;

            if (!is_object_behind_camera(camera.position, camera.target, moon_pos_world)) {
                DrawRing(moon, 14*(scale_factor), 15*(scale_factor), 0.0, 360.0,20, GRAY);
                DrawText("MOON",(int)moon.x - MeasureText("MOON",10)/2,(int)moon.y - MeasureText("MOON", 10),10,GREEN);
            }

            if (!is_object_behind_camera(camera.position, camera.target, t0_farther_out)) {
                DrawText("T=0",(int)t0_farther_out_world.x - MeasureText("T=0",10)/2,(int)t0_farther_out_world.y - MeasureText("T=0", 10),10,GREEN);
            }

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
