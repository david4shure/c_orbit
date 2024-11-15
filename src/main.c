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
#include "physics/corbit_math.h"
#include "physics/orbital_lines.h"
#include "time.h"
#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"
#include "rlgl.h"
#include <stdint.h>
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

const int screenWidth = 1500;
const int screenHeight = 1000;

Vector3 TF(DVector3 vec) {
    return (Vector3){.x = (float)vec.x, .y = (float)vec.y, .z= (float)vec.z};
}

DVector3 TD(Vector3 vec) {
    return (DVector3){.x = (double)vec.x, .y = (double)vec.y, .z= (double)vec.z};
}

// Function to check if an object is behind the camera
bool is_object_behind_camera(Vector3 cam_pos, Vector3 cam_target, Vector3 obj_pos) {
    Vector3 camera_to_target = Vector3Normalize(Vector3Subtract(cam_pos,cam_target));
    Vector3 camera_to_object = Vector3Normalize(Vector3Subtract(cam_pos,obj_pos));

    float dot = Vector3DotProduct(camera_to_target, camera_to_object);

    return dot < 0;
}


Color interpolate_color(Color color1, Color color2, float t) {
    // Clamp t between 0.0 and 1.0
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    Color result;
    result.r = (uint8_t)(color1.r + (color2.r - color1.r) * t);
    result.g = (uint8_t)(color1.g + (color2.g - color1.g) * t);
    result.b = (uint8_t)(color1.b + (color2.b - color1.b) * t);
    result.a = 255; // Alpha is fixed at maximum

    return result;
}


void draw_orbital_lines(void* orbital_positions) {
    PointBundle* prev_pos = NULL;
    PointBundle* current_pos = NULL;

    // Now iterate over our darray and draw lines
    for (int i = 0; i < darray_length(orbital_positions); i++) {
        if (i==0) {
            prev_pos = darray_get(orbital_positions, i);
            continue;
        }

        current_pos = darray_get(orbital_positions, i);

        // Draw the dang line
        /* DrawPoint3D(TF(vector_from_physical_to_world(*current_pos)),WHITE); */

        Color col = BLUE;

        DrawLine3D(TF(vector_from_physical_to_world(current_pos->point)), TF(vector_from_physical_to_world(prev_pos->point)), col);

        prev_pos = current_pos;
    }
}

void draw_element(char* format_text, double value, int x, int y, Color color) {
    char buffer[50];  // Buffer for the formatted string
            // Format the float as a string
    snprintf(buffer, sizeof(buffer), format_text, value);

    DrawText(buffer,x,y,2,color);
}

void draw_orbital_parameters(OrbitalElements oe, int num_lines) {
    int left_padding = 15;
    int padding_between_rows = 13;
    Color text_color = RED;

    Debug("LEN LINES %d\n",num_lines);

    draw_element("e = %.2f", oe.eccentricity, left_padding, padding_between_rows, text_color);
    draw_element("a = %.2f", oe.semimajor_axis, left_padding, padding_between_rows * 2, text_color);
    draw_element("true anomaly = %.2f°", oe.true_anomaly * RAD2DEG, left_padding, padding_between_rows * 3, text_color);
    draw_element("argument of periapsis = %.2f°", oe.arg_of_periapsis * RAD2DEG, left_padding, padding_between_rows * 4, text_color);
    draw_element("inclination = %.2f°", oe.inclination * RAD2DEG, left_padding, padding_between_rows * 5, text_color);
    draw_element("longitude of the ascending node = %.2f°", oe.long_of_asc_node * RAD2DEG, left_padding, padding_between_rows * 6, text_color);
    draw_element("number of lines = %.0f", (double)num_lines, screenWidth-140, padding_between_rows, GREEN);
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Loggeruuu
    InitializeLogger(LOG_LEVEL);

    // DEBUG : r={2646.36390,0.00000,172135.51695},v={2.15104,0.00000,-0.04530}
    // r={535344.68601,0.00000,-253679.25784},v={0.61903,0.00000,-0.98094}
    DVector3 moon_position = {384400.0,0.0,0.0};
    DVector3 moon_velocity = {0.0,1.022,0.0};
    /* DVector3 moon_position = {535344.68601,0.00000,-253679.25784}; */
    /* DVector3 moon_velocity = {0.61903,0.00000,-0.98094}; */

    // X =-1.434469380595836E+05 Y =-3.679580723218923E+05 Z =-3.718096192515444E+04
    // VX= 5.910407088277736E-01 VY=-4.828271439706466E-01 VZ= 3.566713870051011E-02

    /* DVector3 moon_position = {-1.434469380595836E+05,-3.679580723218923E+05,-3.718096192515444E+04}; */
    /* DVector3 moon_velocity = { 5.910407088277736E-01,-4.828271439706466E-01, 3.566713870051011E-02}; */

    PhysicalState RV = {
        .r = moon_position,
        .v = moon_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG,
    };

    float M_naught = 2.35585;
    float t_naught = 0.0;
    OrbitalElements eles = orb_elems_from_rv(RV,0.0,0.0);

    PhysicsTimeClock clock = { .tick_interval_seconds = 86400, .mode = Elapsing, .scale = 10000.0, .delta_seconds = 0.0, .clock_seconds = 0.0};

    //--------------------------------------------------------------------------------------

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
    float phi = PI;

    Matrix matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,10000000.0);

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float delta;
    print_physical_state(RV);
    float r_at_sphere_of_influence = calculate_sphere_of_influence_r(EARTH_SEMIMAJOR_AXIS_KM, eles.mass_of_parent, eles.mass_of_grandparent);
    float r_at_soi_world_coords = r_at_sphere_of_influence * KM_TO_RENDER_UNITS;

    darray orbital_lines;

    // Initial slider value and integer input value
    float sliderValue = 50.0f;
    int intValue = 10;


    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        time = GetTime();
        delta = GetFrameTime();

        UpdatePhysicsClock(&clock, delta);

        orbital_lines = compute_orbital_lines(RV, clock.clock_seconds, 0.0, 0.0,r_at_sphere_of_influence*3);
        int len_lines = darray_length(orbital_lines);

        RV = rv_from_r0v0(RV,clock.delta_seconds); 
        Debug("r={%.5f,%.5f,%.5f},v={%.5f,%.5f,%.5f}\n",RV.r.x,RV.r.y,RV.r.z,RV.v.x,RV.v.y,RV.v.z);

        eles = orb_elems_from_rv(RV, M_naught, t_naught);
        print_orbital_elements(eles);
        moon_position = RV.r;
        moon_velocity = RV.v;

        if(IsKeyDown(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
            RV.v = DVector3Scale(RV.v,1.001);
        }

        if(IsKeyDown(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
            RV.v = DVector3Scale(RV.v,0.999);
        }

        if(IsKeyDown(KEY_D) || IsKeyPressed(KEY_D)) {
            clock.scale *= 2;
        }

        if(IsKeyDown(KEY_A) || IsKeyPressed(KEY_A)) {
            clock.scale /= 2;
        }

        /* moon_velocity = Vector3Scale(RV.v,0.9); */

        DVector3 moon_pos_world = vector_from_physical_to_world(moon_position);
        // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
        float dist = DVector3Distance(moon_pos_world, TD(camera.position));

        DVector3 t0_line = vector_from_physical_to_world(solve_kepler_ellipse_inertial(eles, 0.0, 0.0, 0.0));
        DVector3 t0_norm = DVector3Normalize(t0_line);
        DVector3 t0_farther_out = DVector3Add(t0_line, DVector3Scale(t0_norm,200));

        Vector2 t0_farther_out_world = GetWorldToScreen(TF(t0_farther_out), camera);
        Vector2 moon = GetWorldToScreen(TF(moon_pos_world), camera);

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
                DrawGridOfColor(250,500000,grid_color); // Draw ground

                DrawSphereWires(TF(moon_pos_world),(MOON_RADIUS_KM * KM_TO_RENDER_UNITS),10,10,GRAY);
                DrawSphereWires(sphere_pos,(r_at_soi_world_coords),10,10,(Color){.r=255, .b=182, .g=193,.a=50});
            EndMode3D();

            draw_orbital_parameters(eles,len_lines);

            float max_distance = 5000.0;
            float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-moon_pos_world.x,camera.position.y-moon_pos_world.y,camera.position.z-moon_pos_world.z});
            float distance_scale_factor = max_distance/camera_to_moon_distance;
            float time_scale_factor = sin(time*1.5)/4.0 + 1;

            float scale_factor = distance_scale_factor * time_scale_factor;
            double velocity = DVector3Length(RV.v);
            char velocity_str[20];  // Buffer for the formatted string
            // Format the float as a string
            snprintf(velocity_str, sizeof(velocity_str), "V=%.2f KM/s", velocity);


            if (!is_object_behind_camera(camera.position, camera.target, TF(moon_pos_world))) {
                DrawRing(moon, 14*(scale_factor), 15*(scale_factor), 0.0, 360.0,20, GRAY);
                DrawText("MOON",(int)moon.x - MeasureText("MOON",10)/2,(int)moon.y - MeasureText("MOON", 10),10,GREEN);
                DrawText(velocity_str,(int)moon.x - MeasureText(velocity_str,10)/2,(int)moon.y - MeasureText(velocity_str,10),10,GREEN);
            }

            if (!is_object_behind_camera(camera.position, camera.target, TF(t0_farther_out))) {
                DrawText("Pa",(int)t0_farther_out_world.x - MeasureText("T=0",10)/2,(int)t0_farther_out_world.y - MeasureText("T=0", 10),10,GREEN);
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
