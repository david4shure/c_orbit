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
#include <objc/objc.h>
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
#include "physics/propagation.h"

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


void draw_orbital_lines(darray orbital_lines, ClassicalOrbitalElements oe, PhysicalState RV, Nodes n, PhysicsTimeClock clock, Camera3D camera, float time) {
    PointBundle* prev_pos = NULL;
    PointBundle* current_pos = NULL;
    
    float r_at_sphere_of_influence = calculate_sphere_of_influence_r(RV.mass_of_parent, oe.mass_of_parent, oe.mass_of_grandparent);
    float r_at_soi_world_coords = r_at_sphere_of_influence * KM_TO_RENDER_UNITS;

    // Now iterate over our darray and draw lines
    for (int i = 0; i < darray_length(orbital_lines); i++) {
        if (i==0) {
            prev_pos = darray_get(orbital_lines, i);
            continue;
        }

        current_pos = darray_get(orbital_lines, i);

        if (current_pos == NULL) {
            break;
        }

        DrawLine3D(TF(vector_from_physical_to_world(current_pos->point)), TF(vector_from_physical_to_world(prev_pos->point)), BLUE);

        prev_pos = current_pos;
    }
    
    Color myyellow = (Color){ 253, 249, 0, 100};

    if (oe.eccentricity < 1.0) {
        DrawLine3D(TF(vector_from_physical_to_world(n.asc)), TF(vector_from_physical_to_world(n.desc)), myyellow);
    }
}


void draw_body_label(ClassicalOrbitalElements oe, PhysicalState RV, Nodes n, PhysicsTimeClock clock, Camera3D camera, float time) {
    float max_distance = 5000.0;
    float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-RV.r.x,camera.position.y-RV.r.y,camera.position.z-RV.r.z});
    float distance_scale_factor = max_distance/camera_to_moon_distance;
    float time_scale_factor = sin(time*1.5)/4.0 + 1;

    float scale_factor = distance_scale_factor * time_scale_factor;
    double velocity = DVector3Length(RV.v);
    char velocity_str[20];  // Buffer for the formatted string
    // Format the float as a string
    snprintf(velocity_str, sizeof(velocity_str), "V=%.2f KM/s", velocity);

    DVector3 moon_pos_world = vector_from_physical_to_world(RV.r);
    // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    float dist = DVector3Distance(moon_pos_world, TD(camera.position));

    DVector3 periapsis_point = vector_from_physical_to_world(solve_kepler_ellipse_inertial(oe, 0.0, 0.0, 0.0));
    DVector3 apoapsis_point = vector_from_physical_to_world(solve_kepler_ellipse_inertial(oe, 0.0, 0.0, oe.period/2.0));
    DVector3 asc_node_point = vector_from_physical_to_world(n.asc);
    DVector3 desc_node_point = vector_from_physical_to_world(n.desc);

    Vector2 periapsis_point_camera = GetWorldToScreen(TF(periapsis_point), camera);
    Vector2 apoapsis_point_camera = GetWorldToScreen(TF(apoapsis_point), camera);
    Vector2 asc_point_camera = GetWorldToScreen(TF(asc_node_point), camera);
    Vector2 desc_point_camera = GetWorldToScreen(TF(desc_node_point), camera);

    Vector2 moon = GetWorldToScreen(TF(moon_pos_world), camera);

    if (!is_object_behind_camera(camera.position, camera.target, TF(moon_pos_world))) {
        DrawText("MOON",(int)moon.x - MeasureText("MOON",10)/2,(int)moon.y - MeasureText("MOON", 10),10,GREEN);
        DrawText(velocity_str,(int)moon.x - MeasureText(velocity_str,10)/2,(int)moon.y - MeasureText(velocity_str,10),10,GREEN);
    }
}


void draw_orbital_features(ClassicalOrbitalElements oe, PhysicalState RV, Nodes n, PhysicsTimeClock clock, Camera3D camera, float time) {
    float max_distance = 5000.0;
    float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-RV.r.x,camera.position.y-RV.r.y,camera.position.z-RV.r.z});
    float distance_scale_factor = max_distance/camera_to_moon_distance;
    float time_scale_factor = sin(time*1.5)/4.0 + 1;

    float scale_factor = distance_scale_factor * time_scale_factor;
    double velocity = DVector3Length(RV.v);
    char velocity_str[20];  // Buffer for the formatted string
    // Format the float as a string
    snprintf(velocity_str, sizeof(velocity_str), "V=%.2f KM/s", velocity);

    DVector3 moon_pos_world = vector_from_physical_to_world(RV.r);
    // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    float dist = DVector3Distance(moon_pos_world, TD(camera.position));

    DVector3 periapsis_point = vector_from_physical_to_world(solve_kepler_ellipse_inertial(oe, 0.0, 0.0, 0.0));
    DVector3 apoapsis_point = vector_from_physical_to_world(solve_kepler_ellipse_inertial(oe, 0.0, 0.0, oe.period/2.0));
    DVector3 asc_node_point = vector_from_physical_to_world(n.asc);
    DVector3 desc_node_point = vector_from_physical_to_world(n.desc);

    Vector2 periapsis_point_camera = GetWorldToScreen(TF(periapsis_point), camera);
    Vector2 apoapsis_point_camera = GetWorldToScreen(TF(apoapsis_point), camera);
    Vector2 asc_point_camera = GetWorldToScreen(TF(asc_node_point), camera);
    Vector2 desc_point_camera = GetWorldToScreen(TF(desc_node_point), camera);

    Vector2 moon = GetWorldToScreen(TF(moon_pos_world), camera);


    if (!is_object_behind_camera(camera.position, camera.target, TF(moon_pos_world))) {
        DrawRing(moon, 14*(scale_factor), 15*(scale_factor), 0.0, 360.0,20, GRAY);
        DrawText("MOON",(int)moon.x - MeasureText("MOON",10)/2,(int)moon.y - MeasureText("MOON", 10),10,GREEN);
        DrawText(velocity_str,(int)moon.x - MeasureText(velocity_str,10)/2,(int)moon.y - MeasureText(velocity_str,10),10,GREEN);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(periapsis_point))) {
        DrawText("Pa",(int)periapsis_point_camera.x - MeasureText("Pa",10)/2,(int)periapsis_point_camera.y - MeasureText("Pa", 10),10,SKYBLUE);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(apoapsis_point))) {
        DrawText("Apo",(int)apoapsis_point_camera.x - MeasureText("Apo",10)/2,(int)apoapsis_point_camera.y - MeasureText("Apo", 10),10,DARKBLUE);
    }

    if (oe.eccentricity >= 1.0) {
        return;
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(asc_node_point))) {
        DrawText("Asc",(int)asc_point_camera.x - MeasureText("Asc",10)/2,(int)asc_point_camera.y,10,YELLOW);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(desc_node_point))) {
        DrawText("Desc",(int)desc_point_camera.x - MeasureText("Desc",10)/2,(int)desc_point_camera.y,10,ORANGE);
    }
}


void draw_element(char* format_text, double value, int x, int y, Color color) {
    char buffer[50];  // Buffer for the formatted string
            // Format the float as a string
    snprintf(buffer, sizeof(buffer), format_text, value);

    DrawText(buffer,x,y,2,color);
}

void draw_textual_orbital_elements(ClassicalOrbitalElements oe, int num_lines) {
    int left_padding = 15;
    int padding_between_rows = 13;
    Color text_color = RED;

    Debug("LEN LINES %d\n",num_lines);

    draw_element("e = %.3f", oe.eccentricity, left_padding, padding_between_rows, text_color);
    draw_element("a = %.3f", oe.semimajor_axis, left_padding, padding_between_rows * 2, text_color);
    draw_element("true anomaly = %.3f°", oe.true_anomaly * RAD2DEG, left_padding, padding_between_rows * 3, text_color);
    draw_element("argument of periapsis = %.3f°", oe.arg_of_periapsis * RAD2DEG, left_padding, padding_between_rows * 4, text_color);
    draw_element("inclination = %.3f°", oe.inclination * RAD2DEG, left_padding, padding_between_rows * 5, text_color);
    draw_element("longitude of the ascending node = %.3f°", oe.long_of_asc_node * RAD2DEG, left_padding, padding_between_rows * 6, text_color);
    draw_element("number of lines = %.0f", (double)num_lines, screenWidth-140, padding_between_rows, GREEN);
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Loggeruuu
    InitializeLogger(LOG_LEVEL,true);

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
        .mass = MOON_MASS_KG,
    };

    float M_naught = 2.35585;
    float t_naught = 0.0;
    ClassicalOrbitalElements eles = rv_to_classical_elements(RV);

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

    /* DisableCursor();                    // Limit cursor to relative movement inside the window */

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float delta;
    print_physical_state(RV);

    darray orbital_lines;

    // Initial slider value and integer input value
    float sliderValue = 50.0f;
    int intValue = 10;
    bool should_draw_orbital_features = true;

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        time = GetTime();
        delta = GetFrameTime();

        UpdatePhysicsClock(&clock, delta);

        RV = rv_from_r0v0(RV,clock.delta_seconds); 
        ClassicalOrbitalElements neweles = rv_to_classical_elements(RV);
        PhysicalState lilrv = classical_elements_to_rv(eles);
        Info("physical state r = (%.2f,%.2f,%.2f), v = (%.2f, %.2f, %.2f)\n",RV.r.x,RV.r.y,RV.r.z,RV.v.x,RV.v.y,RV.v.z);
        Info("Eles = \n");
        print_orbital_elements(neweles);
        Info("rv_from_orb_elems r = (%.2f,%.2f,%.2f), v = (%.2f, %.2f, %.2f)\n",lilrv.r.x,lilrv.r.y,lilrv.r.z,lilrv.v.x,lilrv.v.y,lilrv.v.z);

        Debug("r={%.5f,%.5f,%.5f},v={%.5f,%.5f,%.5f}\n",RV.r.x,RV.r.y,RV.r.z,RV.v.x,RV.v.y,RV.v.z);

        eles = rv_to_classical_elements(RV);
        print_orbital_elements(eles);
        moon_position = RV.r;
        moon_velocity = RV.v;

        if(IsKeyDown(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
            DVector3 forward = DVector3Scale(DVector3Normalize(RV.v),10000000000000000000000.0);

            Debug("Forward vector = (%.4f, %.4f, %.4f)\n",forward.x,forward.y,forward.z);

            Debug("Applying force\n");
            RV = apply_force_to(RV, forward, delta);
            Debug("Printing physical state.\n");

            print_physical_state(RV);
        }


        if(IsKeyDown(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
            DVector3 backwards = DVector3Scale(DVector3Normalize(RV.v),-10000000000000000000000.0);

            Debug("Forward vector = (%.4f, %.4f, %.4f)\n",backwards.x,backwards.y,backwards.z);

            Debug("Applying force\n");
            RV = apply_force_to(RV, backwards, delta);
            Debug("Printing physical state.\n");

            print_physical_state(RV);
        }

        if(IsKeyDown(KEY_UP) || IsKeyPressed(KEY_UP)) {
            DVector3 up = DVector3Scale(DVector3Normalize(DVector3CrossProduct(RV.r, RV.v)),10000000000000000000000.0);

            Debug("Up vector = (%.4f, %.4f, %.4f)\n",up.x,up.y,up.z);

            Debug("Applying force\n");
            RV = apply_force_to(RV, up, delta);
            Debug("Printing physical state.\n");

            print_physical_state(RV);
        }


        if(IsKeyDown(KEY_DOWN) || IsKeyPressed(KEY_DOWN)) {
            DVector3 down = DVector3Scale(DVector3Normalize(DVector3CrossProduct(RV.r, RV.v)),-10000000000000000000000.0);

            Debug("Down vector = (%.4f, %.4f, %.4f)\n",down.x,down.y,down.z);

            Debug("Applying force\n");
            RV = apply_force_to(RV, down, delta);
            Debug("Printing physical state.\n");

            print_physical_state(RV);
        }

        if(IsKeyDown(KEY_D) || IsKeyPressed(KEY_D)) {
            clock.scale *= 2;
        }

        if(IsKeyDown(KEY_A) || IsKeyPressed(KEY_A)) {
            clock.scale /= 2;
        }

        if(IsKeyPressed(KEY_V)) {
            should_draw_orbital_features = !should_draw_orbital_features;
        }

        eles = rv_to_classical_elements(RV);


        float r_at_sphere_of_influence = calculate_sphere_of_influence_r(EARTH_SEMIMAJOR_AXIS_KM, eles.mass_of_parent, eles.mass_of_grandparent);
        float r_at_soi_world_coords = r_at_sphere_of_influence * KM_TO_RENDER_UNITS;
        
        darray orbital_lines = compute_orbital_lines(RV, clock.clock_seconds, r_at_sphere_of_influence*3);
        int len_lines = darray_length(orbital_lines);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            SphericalCameraSystem(&r, &theta, &phi, &camera);

            Nodes n = compute_nodes(eles);

            draw_textual_orbital_elements(eles,darray_length(orbital_lines));
            draw_body_label(eles, RV, n, clock, camera, time);

            if (should_draw_orbital_features) {
                draw_orbital_features(eles, RV, n, clock, camera, time);
            }

            BeginMode3D(camera);
                rlSetMatrixProjection(matProj);

                if (should_draw_orbital_features) {
                    draw_orbital_lines(orbital_lines,eles,RV,n,clock,camera,clock.clock_seconds);
                    darray_free(orbital_lines);
                }

                Vector3 sphere_pos = {0.0,0.0,0.0};

                DrawSphereWires(sphere_pos,EARTH_RADIUS_KM * KM_TO_RENDER_UNITS,10,10,SKYBLUE);
                Color grid_color = { .r = 0, .g = 240, .b = 0, .a = 150};
                DrawSphereWires(TF(vector_from_physical_to_world(RV.r)),MOON_RADIUS_KM * KM_TO_RENDER_UNITS,10,10,LIGHTGRAY);
                DrawGridOfColor(250,500000,grid_color); // Draw equatorial plane 
                DrawSphereWires(sphere_pos,(r_at_soi_world_coords),10,10,(Color){.r=255, .b=182, .g=193,.a=50});

            EndMode3D();


        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
