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

#include "time.h"
#include "camera.h"
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

#define LOG_LEVEL DEBUG

// Function to check if an object is behind the camera
bool is_object_behind_camera(Vector3 cam_pos, Vector3 cam_target, Vector3 obj_pos) {
    Vector3 camera_to_target = Vector3Normalize(Vector3Subtract(cam_pos,cam_target));
    Vector3 camera_to_object = Vector3Normalize(Vector3Subtract(cam_pos,obj_pos));

    float dot = Vector3DotProduct(camera_to_target, camera_to_object);

    return dot < 0;
}

void draw_orbital_lines(OrbitalElements orbit, Camera* camera) {
    int num_lines = 300;

    void* orbital_positions = darray_init(num_lines, sizeof(Vector3));

    Debug("Malloc ptr = %p\n",orbital_positions);

    // Draw num_lines lines (+1)
    float increment = orbit.period / num_lines;
    int i = 0;

    // Iterate through entire orbit
    for (float t = 0.0; t < orbit.period; t += increment) {
        i += 1;

        Vector3 moon_pos_physical = solve_kepler_ellipse_inertial(orbit, 0.0, 0.0, t);

        // Push computed vector
        orbital_positions = darray_push(orbital_positions,(void*)&moon_pos_physical);
    }

    Vector3* prev_pos = NULL;
    Vector3* current_pos = NULL;

    // Now iterate over our darray and draw lines
    for (int i = 0; i < darray_length(orbital_positions); i++) {
        if (i == 0) {
            prev_pos = (Vector3*)darray_get(orbital_positions, 0);
            continue;
        }

        // Wrap around and connect last point to first to complete the ellipse
        if (i >= darray_length(orbital_positions)-1) {
            current_pos = (Vector3*)darray_get(orbital_positions, 0);
        } else {
            current_pos = (Vector3*)darray_get(orbital_positions, i);
        }

        // Convert vector from physical dimensions to render dimensions
        Vector3 world_curr = vector_from_physical_to_world(*prev_pos);
        Vector3 world_prev = vector_from_physical_to_world(*current_pos);

        // Actually draw the line
        DrawLine3D(world_curr, world_prev, RED);

        // Wrap
        prev_pos = current_pos;
    }

    Vector3* curr_pos = NULL;
    for (int j = 0; j < darray_length(orbital_positions); j++) {
        if (j == 0) {
            prev_pos = (Vector3*)darray_get(orbital_positions, 0);
            continue;
        }

        curr_pos = (Vector3*)darray_get(orbital_positions, j);

        // Wrap around and connect last point to first to complete the ellipse
        if (j >= darray_length(orbital_positions)-1) {
            curr_pos = (Vector3*)darray_get(orbital_positions, 0);
        } else {
            curr_pos = (Vector3*)darray_get(orbital_positions, j);
        }

        // Convert vector from physical dimensions to render dimensions
        Vector3 world_j = vector_from_physical_to_world(*prev_pos);
        Vector3 world_k = vector_from_physical_to_world(*curr_pos);

        Color backside  = (Color){.r=100.0,.g=0,.b=120,.a=50};
        Color frontside = (Color){.r=100.0,.g=100,.b=120,.a=50};

        // Draw the symmetry line
        DrawTriangle3D((Vector3){.x=0.0,.y=0.0,.z=0.0}, world_j, world_k, frontside);
        DrawTriangle3D((Vector3){.x=0.0,.y=0.0,.z=0.0}, world_k, world_j, backside);

        prev_pos = curr_pos;
    }

    // Draw t=0 line
    darray_free(orbital_positions);
}


//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Logger
    InitializeLogger(LOG_LEVEL);

    Fatal("Fatal = %i.\n",10);
    Error("Error.\n");
    Warn("Warn = %i.\n",100);
    Log("Log.\n");
    Info("Info.\n");
    Debug("Debug.\n");

    // Approximate ECI position of the Moon (in km)
    Vector3 moon_position = { -6045.0, -3490.0, 2500.0 };

    // Approximate ECI velocity of the Moon (in km/s)
    Vector3 moon_velocity = { -3.457, 6.618, 2.533 };

    PhysicalState state = {moon_position,moon_velocity};
    OrbitalElements eles = orb_elems_from_rv(state, 398600.4418);
    print_orbital_elements(eles);
    PhysicalState RV = rv_from_orb_elems(eles);

    Log("given R = (%.2f,%.2f,%.2f), V = (%.2f, %.2f, %.2f)\n",moon_position.x,moon_position.y,moon_position.z,moon_velocity.x,moon_velocity.y,moon_velocity.z);
    Log("computed R = (%.2f,%.2f,%.2f), V = (%.2f, %.2f, %.2f)\n",RV.r.x,RV.r.y,RV.r.z,RV.v.x,RV.v.y,RV.v.z);


    OrbitalElements eles_to_rv = (OrbitalElements){
        .ang_momentum = 80000.0,
        .grav_param = 398600.4418,
        .eccentricity = 1.4,
        .long_of_asc_node = 40.0 * DEG2RAD,
        .arg_of_periapsis = 60.0 * DEG2RAD,
        .true_anomaly = 30.0 * DEG2RAD,
        .inclination = 30.0 * DEG2RAD,
    };

    PhysicalState RV2 = rv_from_orb_elems(eles_to_rv);
    Log("final computed R = (%.2f,%.2f,%.2f), V = (%.2f, %.2f, %.2f)\n",RV2.r.x,RV2.r.y,RV2.r.z,RV2.v.x,RV2.v.y,RV2.v.z);

    Log("Computing universal anomaly now...\n");
    float r0 = 10000.0;
    float v0 = 3.0752;
    float dt = 3600;
    float a = -19655.0;

    float x = solve_universal_anomaly(dt, r0, v0, 1/a, 398600.4418);
    Log("Universal anomaly x = %.4f\n",x);

    Debug("computing r0v0\n");
    Vector3 R0 = (Vector3){7000.0, -12124.0, 0.0};
    Vector3 V0 = (Vector3){2.6679, 4.621, 0.0};

    float t = 3600.0;

    PhysicalState RV_FINAL = rv_from_r0v0(R0, V0, t, 398600.0);

    Log("RV_FINAL PREDICTED R = (%.3f,%.3f,%.3f), V = (%.3f, %.3f, %.3f)\n",RV_FINAL.r.x,RV_FINAL.r.y,RV_FINAL.r.z,RV_FINAL.v.x,RV_FINAL.v.y,RV_FINAL.v.z);

    PhysicsTimeClock clock = { .tick_interval_seconds = 86400, .mode = Elapsing, .scale = 50.0, .delta_seconds = 0.0 };

    float M_naught = 2.35585;
    float t_naught = 0.0;

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
    float r = 1000.0;
    float theta = 0.0;
    float phi = 0.0;


    Matrix matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,10000000.0);

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float delta;

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        time = GetTime();
        delta = GetFrameTime();

        UpdatePhysicsClock(&clock, delta*1000);

        Vector3 moon_pos_physical = solve_kepler_ellipse_inertial(eles, M_naught, t_naught, clock.clock_seconds);
        Vector3 moon_pos_world = vector_from_physical_to_world(moon_pos_physical);
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

                draw_orbital_lines(eles,&camera);

                Vector3 sphere_pos = {0.0,0.0,0.0};

                DrawSphereWires(sphere_pos,EARTH_RADIUS_KM * KM_TO_RENDER_UNITS,10,10,SKYBLUE);
                Color grid_color = { .r = 0, .g = 240, .b = 0, .a = 150};
                DrawGridOfColor(250,50000,grid_color); // Draw ground

                DrawSphereWires(moon_pos_world,(MOON_RADIUS_KM * KM_TO_RENDER_UNITS),10,10,GRAY);
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
