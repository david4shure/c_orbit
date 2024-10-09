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
#include "physics.h"
#include "rlutil.h"
#include "raygui.h"
#include "darray.h"

#define MAX_COLUMNS 20

void draw_orbital_lines(OrbitalElements orbit) {
    void* orbital_positions = darray_init(100, sizeof(Vector3));

    // Draw 100 lines (+1)
    float increment = orbit.period / 10000.0;

    // Iterate through entire orbit
    for (float t = 0.0; t < orbit.period; t += increment) {
        float M = mean_anom(0.0,t,0, orbit.period);

        if (M > 2 * PI) {
            M = M - 2 * PI;
        }

        // Solve keplers eq MA -> E -> TA -> DIST -> (x,y,z)
        float E = solve_kepler_eq_ellipse(orbit.eccentricity, M, 50);
        float TA = ecc_anom_to_true_anom(orbit.eccentricity, E);
        float dist = distance_sphere_coords(orbit.eccentricity,orbit.semimajor_axis,E);

        // TODO: Actually remove orbital lines
        float x = dist * cos(TA);
        float y = dist * sin(TA);

        Vector3 vec = (Vector3){.x=x,.y=0.0,.z=y};

        // Push computed vector
        orbital_positions = darray_push(orbital_positions,(void*)&vec);
    }

    Vector3* prev_pos = NULL;
    Vector3* current_pos = NULL;

    // Now iterate over our darray and draw lines
    for (int i = 0; i < darray_length(orbital_positions) + 1; i++) {
        if (i == 0) {
            prev_pos = (Vector3*)darray_get(orbital_positions, 0);
            continue;
        }

        // Wrap around and connect last point to first to complete the ellipse
        if (i >= darray_length(orbital_positions)) {
            current_pos = (Vector3*)darray_get(orbital_positions, 0);
        } else {
            current_pos = (Vector3*)darray_get(orbital_positions, i);
        }


        // Convert vector from physical dimensions to render dimensions
        Vector3 world_curr = vector_from_physical_to_world(*prev_pos);
        Vector3 world_prev = vector_from_physical_to_world(*current_pos);

        DrawLine3D(world_curr, world_prev, RED);
        prev_pos = current_pos;
    }

}


//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {


    // Approximate ECI position of the Moon (in km)
    Vector3 moon_position = { 318000.0f, 215000.0f, 5000.0f };

    // Approximate ECI velocity of the Moon (in km/s)
    Vector3 moon_velocity = { -0.6f, 0.5f, 0.1f };

    PhysicalState state = {moon_position,moon_velocity};
    OrbitalElements eles = orb_elems_from_rv(state, 398600.4418);
    
    print_orbital_elements(eles);
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
                                                        //

    int cameraMode = CAMERA_FREE;
    float time = 0.0;

    // Spherical Camera coordinates
    float r = 1000.0;
    float theta = 0.0;
    float phi = 0.0;


    Matrix matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,1000000.0);

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

        float M = mean_anom(eles.mean_anomaly,clock.clock_seconds, 0, eles.period);

        if (M > 2 * PI) {
            M = M - 2 * PI;
        }

        float E = solve_kepler_eq_ellipse(eles.eccentricity, M, 50);
        float TA = ecc_anom_to_true_anom(eles.eccentricity, E);
        float dist = distance_sphere_coords(eles.eccentricity,eles.semimajor_axis,E);

        printf("M = %.2f\n",M);
        printf("E = %.2f\n",E);
        printf("TA = %.2f\n",TA);

        printf("a = %.2f\n",eles.semimajor_axis);

        printf("dist = %2.f\n",dist);

        float x = dist * cos(TA);
        float y = dist * sin(TA);


        printf("M = %.2f, E = %.2f, TA = %2f\n",M,E,TA);
        // Update camera computes movement internally depending on the camera mode
        // Some default standard keyboard/mouse inputs are hardcoded to simplify use
        // For advance camera controls, it's reecommended to compute camera movement manually
        //UpdateCamera(&camera, CAMERA_ORBITAL);

        // theta = clampf(theta,0+eta, 2*PI-eta);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            SphericalCameraSystem(&r, &theta, &phi, &camera);

            BeginMode3D(camera);
                rlSetMatrixProjection(matProj);

                draw_orbital_lines(eles);

                Vector3 sphere_pos = {0.0,0.0,0.0};

                // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
                dist = Vector3Distance(sphere_pos, camera.position);

                printf("x = %.2f, y = %.2f\n",x,y);
                Vector3 moon_pos = vector_from_physical_to_world((Vector3){.x = x,.y = 0,.z = y});
                printf("Moon pos = (%.2f,%.2f,%.2f)\n",moon_pos.x,moon_pos.y,moon_pos.z);
                Vector2 screenspace_pos = GetWorldToScreen(moon_pos, camera);
                printf("Screenspace pos (%d,%d)\n",(int)screenspace_pos.x,(int)screenspace_pos.y);

                DrawSphereWires(sphere_pos,100.0,10,10,SKYBLUE);
                Color grid_color = { .r = 0, .g = 240, .b = 0, .a = 50};
                DrawGridOfColor(250,500.0,grid_color); // Draw ground

                DrawSphereWires(moon_pos,15,10,10,LIGHTGRAY);
            EndMode3D();

            float max_distance = 5000.0;
            float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-moon_pos.x,camera.position.y-moon_pos.y,camera.position.z-moon_pos.z});
            float distance_scale_factor = max_distance/camera_to_moon_distance;
            float time_scale_factor = sin(time*1.5)/4.0 + 1;
            float scale_factor = distance_scale_factor * time_scale_factor;

            DrawRing(screenspace_pos, 14*(scale_factor), 15*(scale_factor), 0.0, 360.0,20, GRAY);
            DrawText("MOON",(int)screenspace_pos.x - MeasureText("MOON",10)/2,(int)screenspace_pos.y - MeasureText("MOON", 10),10,GREEN);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
