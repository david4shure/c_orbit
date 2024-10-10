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
    int num_lines = 300;

    void* orbital_positions = darray_init(num_lines, sizeof(Vector3));

    printf("Malloc ptr = %p\n",orbital_positions);

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

        // Actually draw the line
        DrawLine3D(world_curr, world_prev, RED);

        // Wrap
        prev_pos = current_pos;
    }

    for (int j = num_lines/2; j < num_lines; j++) {
        if (j%3 != 0) {
            continue;
        }

        if (j == num_lines / 2) {
            continue;
        }

        // Grab symmetrical index on other side
        int k = (num_lines/2) - (j-num_lines/2);

        printf("j=%d,k=%d\n",j,k);

        Vector3* j_pos = (Vector3*)darray_get(orbital_positions, j);
        Vector3* k_pos = (Vector3*)darray_get(orbital_positions, k);

        // Convert vector from physical dimensions to render dimensions
        Vector3 world_j = vector_from_physical_to_world(*j_pos);
        Vector3 world_k = vector_from_physical_to_world(*k_pos);

        // Draw the symmetry line
        DrawLine3D(world_j, world_k, DARKGRAY);
    }

    printf("Freeing orbital positions at adr = %p, i = %d\n",orbital_positions,i);
    darray_free(orbital_positions);
}


//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {

    const Vector3 moon_real_position = {-6045, -3490, 2500};
    const Vector3 moon_real_velocity = {-3.457, 6.618, 2.533};

    // Approximate ECI position of the Moon (in km)
    Vector3 moon_position = { 318000.0f, 215000.0f, 5000.0f };

    // Approximate ECI velocity of the Moon (in km/s)
    Vector3 moon_velocity = { -0.6f, 0.8f, 0.1f };

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

        Vector3 moon_pos_physical = solve_kepler_ellipse_inertial(eles, M_naught, t_naught, clock.clock_seconds);
        Vector3 moon_pos_world = vector_from_physical_to_world(moon_pos_physical);
        Vector2 screenspace_pos = GetWorldToScreen(moon_pos_world, camera);
        // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
        float dist = Vector3Distance(moon_pos_world, camera.position);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            SphericalCameraSystem(&r, &theta, &phi, &camera);

            BeginMode3D(camera);
                rlSetMatrixProjection(matProj);

                draw_orbital_lines(eles);

                Vector3 sphere_pos = {0.0,0.0,0.0};

                DrawSphereWires(sphere_pos,100.0,10,10,SKYBLUE);
                Color grid_color = { .r = 0, .g = 240, .b = 0, .a = 50};
                DrawGridOfColor(250,5000,grid_color); // Draw ground

                DrawSphereWires(moon_pos_world,15,10,10,LIGHTGRAY);
            EndMode3D();

            float max_distance = 5000.0;
            float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-moon_pos_world.x,camera.position.y-moon_pos_world.y,camera.position.z-moon_pos_world.z});
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
