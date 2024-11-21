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
#include "tree.h"
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

typedef struct TimeDisplayInfo{
    int years;
    int weeks;
    int days;
    int hours;
    int minutes;
    int seconds;
} TimeDisplayInfo;

TimeDisplayInfo get_time_info(double total_seconds) {
    int years = 0;
    int weeks = 0;
    int days = 0;
    int hours = 0;
    int minutes = 0;
    int seconds = 0;

    double burndown_seconds = total_seconds;

    if(fmod(burndown_seconds,SECONDS_IN_YEAR) >= 1) {
        years = burndown_seconds / SECONDS_IN_YEAR;
        burndown_seconds = fmod(burndown_seconds,SECONDS_IN_YEAR);
    }

    if (fmod(burndown_seconds,SECONDS_IN_WEEK) >= 1) {
        weeks = burndown_seconds / SECONDS_IN_WEEK;
        burndown_seconds = fmod(burndown_seconds,SECONDS_IN_WEEK);
    }

    if (fmod(burndown_seconds,SECONDS_IN_DAY) >= 1) {
        days = burndown_seconds / SECONDS_IN_DAY;
        burndown_seconds = fmod(burndown_seconds,SECONDS_IN_DAY);
    }

    if (fmod(burndown_seconds,SECONDS_IN_HOUR) >= 1) {
        hours = burndown_seconds / SECONDS_IN_HOUR;
        burndown_seconds = fmod(burndown_seconds,SECONDS_IN_HOUR);
    }

    if (fmod(burndown_seconds,SECONDS_IN_MINUTE) >= 1) {
        minutes = burndown_seconds / SECONDS_IN_MINUTE;
        burndown_seconds = fmod(burndown_seconds, SECONDS_IN_MINUTE);
    }

    seconds = burndown_seconds;

    return (TimeDisplayInfo){
        .years = years,
        .weeks = weeks,
        .days = days,
        .hours = hours,
        .minutes = minutes,
        .seconds = seconds,
    };
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


void draw_orbital_lines(darray orbital_lines, OrbitalTreeNode* node, PhysicsTimeClock clock, Camera3D camera) {
    if (orbital_lines == NULL) {
        return;
    }
    PointBundle* prev_pos = NULL;
    PointBundle* current_pos = NULL;
    
    float r_at_sphere_of_influence = node->parent->physical_params.sphere_of_influence;
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

        Color myred = (Color){ 230, 41, 55, 100};

        if (DVector3Length(current_pos->point) > r_at_sphere_of_influence) {
            DrawLine3D(TF(vector_from_physical_to_world(current_pos->point)), TF(vector_from_physical_to_world(prev_pos->point)), myred);
        } else {
            DrawLine3D(TF(vector_from_physical_to_world(current_pos->point)), TF(vector_from_physical_to_world(prev_pos->point)), BLUE);
        }

        prev_pos = current_pos;
    }
    
    Color myyellow = (Color){ 253, 249, 0, 100};

    if (node->orbital_elements.eccentricity < 1.0) {
        DrawLine3D(TF(vector_from_physical_to_world(node->asc_desc.asc)), TF(vector_from_physical_to_world(node->asc_desc.desc)), myyellow);
    }
}

void draw_orbital_features(OrbitalTreeNode* node, PhysicsTimeClock clock, Camera3D camera) {
    float max_distance = 5000.0;
    float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-node->physical_state.r.x,camera.position.y-node->physical_state.r.y,camera.position.z-node->physical_state.r.z});
    float distance_scale_factor = max_distance/camera_to_moon_distance;
    float time_scale_factor = sin(clock.clock_seconds*1.5)/4.0 + 1;

    float scale_factor = distance_scale_factor * time_scale_factor;
    double velocity = DVector3Length(node->physical_state.v);
    char velocity_str[20];  // Buffer for the formatted string
    // Format the float as a string
    snprintf(velocity_str, sizeof(velocity_str), "V=%.2f KM/s", velocity);

    DVector3 body_pos_world = vector_from_physical_to_world(node->physical_state.r);
    // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    float dist = DVector3Distance(body_pos_world, TD(camera.position));

    DVector3 periapsis_point = vector_from_physical_to_world(solve_kepler_ellipse_inertial(node->orbital_elements, 0.0, 0.0, 0.0));
    DVector3 apoapsis_point = vector_from_physical_to_world(solve_kepler_ellipse_inertial(node->orbital_elements, 0.0, 0.0, node->orbital_elements.period/2.0));
    DVector3 asc_node_point = vector_from_physical_to_world(node->asc_desc.asc);
    DVector3 desc_node_point = vector_from_physical_to_world(node->asc_desc.desc);
    DVector3 positive_x = vector_from_physical_to_world((DVector3){EARTH_RADIUS_KM*10,0,0});
    DVector3 positive_y = vector_from_physical_to_world((DVector3){0,EARTH_RADIUS_KM*10,0});
    DVector3 positive_z = vector_from_physical_to_world((DVector3){0,0,EARTH_RADIUS_KM*10});

    Vector2 periapsis_point_camera = GetWorldToScreen(TF(periapsis_point), camera);
    Vector2 apoapsis_point_camera = GetWorldToScreen(TF(apoapsis_point), camera);
    Vector2 asc_point_camera = GetWorldToScreen(TF(asc_node_point), camera);
    Vector2 desc_point_camera = GetWorldToScreen(TF(desc_node_point), camera);

    Vector2 body = GetWorldToScreen(TF(body_pos_world), camera);
    Vector2 positive_x_direction = GetWorldToScreen(TF(positive_x),camera);
    Vector2 positive_y_direction = GetWorldToScreen(TF(positive_y),camera);
    Vector2 positive_z_direction = GetWorldToScreen(TF(positive_z),camera);

    if (!is_object_behind_camera(camera.position, camera.target, TF(body_pos_world))) {
        DrawText(node->body_name,(int)body.x - MeasureText(node->body_name,10)/2,(int)body.y - MeasureText(node->body_name, 10),10,GREEN);
        DrawText(velocity_str,(int)body.x - MeasureText(velocity_str,10)/2,(int)body.y - MeasureText(velocity_str,10),10,GREEN);
    }
    if (!is_object_behind_camera(camera.position, camera.target, TF(periapsis_point))) {
        DrawText("Pa",(int)periapsis_point_camera.x - MeasureText("Pa",10)/2,(int)periapsis_point_camera.y - MeasureText("Pa", 10),10,SKYBLUE);
    }
    if (!is_object_behind_camera(camera.position, camera.target, TF(apoapsis_point))) {
        DrawText("Apo",(int)apoapsis_point_camera.x - MeasureText("Apo",10)/2,(int)apoapsis_point_camera.y - MeasureText("Apo", 10),10,DARKBLUE);
    }
    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_x))) {
        DrawText("+x",(int)positive_x_direction.x,(int)positive_x_direction.y,13,LIGHTGRAY);
    }
    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_y))) {
        DrawText("+y",(int)positive_y_direction.x,(int)positive_y_direction.y,13,LIGHTGRAY);
    }
    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_z))) {
        DrawText("+z",(int)positive_z_direction.x,(int)positive_z_direction.y,13,LIGHTGRAY);
    }

    if (node->orbital_elements.eccentricity >= 1.0) {
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

    TimeDisplayInfo time_info = get_time_info(oe.period);

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "orbital period = %d years %d weeks %d days %d hours %d minutes %d seconds", time_info.years, time_info.weeks, time_info.days, time_info.hours, time_info.minutes, time_info.seconds);

    DrawText(buffer,left_padding,padding_between_rows * 7,2,PURPLE);
}

void draw_clock_info(PhysicsTimeClock clock) {
    TimeDisplayInfo time_info = get_time_info(clock.clock_seconds);

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%d years %d weeks %d days %d hours %d minutes %d seconds", time_info.years, time_info.weeks, time_info.days, time_info.hours, time_info.minutes, time_info.seconds);

    DrawText(buffer,screenWidth/2-140,10,10,VIOLET);
}

void draw_body(OrbitalTreeNode* node, Camera3D camera) {
    DrawSphereWires(TF(vector_from_physical_to_world(node->physical_state.r)),node->physical_params.radius * KM_TO_RENDER_UNITS,10,10,SKYBLUE);
}

void draw_sphere_of_influence(OrbitalTreeNode* node, Camera3D camera) {
    Debug("drawing sphere of influence for %s\n",node->body_name);
    DrawSphereWires(TF(vector_from_physical_to_world(node->physical_state.r)),node->physical_params.sphere_of_influence * KM_TO_RENDER_UNITS,10,10,(Color){.r=255, .b=182, .g=193,.a=50});
}

// What are the steps?
// 1. Compute position of the body
//     a. Get initial position from either PhysicalState or ClassicalOrbitalElements to start.
//     b. Compute orbital elements and update them in the OrbitalTreeNode*
// 2. Compute asc and descending nodes
// 3. Compute orbital lines
// 3. Draw name of body (if not root)
// 4. Draw orbital ring
// 5. Draw orbital features
// 6. Draw sphere of influence
void draw_orbital_node(OrbitalTreeNode* node, PhysicsTimeClock clock, Camera3D camera3d, bool should_draw_orbital_features, Matrix proj) {
    bool is_root_node = node->parent == NULL;

    Info("Rendering %s\n",node->body_name);

    // Compute next position
    // TODO: Use true numerical integration here.

    if (!is_root_node) {
        // Drawing steps for non root nodes
        if (should_draw_orbital_features) {
            draw_orbital_features(node, clock, camera3d);
        }
    }
  
    BeginMode3D(camera3d);
    rlSetMatrixProjection(proj);
    if (should_draw_orbital_features) {
        draw_orbital_lines(node->orbital_lines, node, clock, camera3d);
    }
    draw_sphere_of_influence(node,camera3d);
    draw_body(node,camera3d);
    EndMode3D();

    // Iterate over children & recurse!
    for (int i = 0; i < darray_length(node->children); i++) {
        draw_orbital_node(darray_get(node->children,i), clock, camera3d, should_draw_orbital_features,proj);
    }
}

void update_orbital_node(OrbitalTreeNode* node, PhysicsTimeClock clock) {
    bool is_root_node = node->parent == NULL;

    Info("Updating %s\n",node->body_name);

    if (!is_root_node && node->use_state_vector) {
        node->orbital_elements = rv_to_classical_elements(node->physical_state,node->parent->physical_params.grav_param);
    } else if (!is_root_node) {
        node->physical_state = classical_elements_to_rv(node->orbital_elements,node->parent->physical_params.grav_param);
    }

    // Compute next position
    // TODO: Use true numerical integration here.

    if (!is_root_node) {
        node->physical_state = rv_from_r0v0(node->physical_state, node->parent->physical_params.grav_param, clock.delta_seconds);
        node->orbital_elements = rv_to_classical_elements(node->physical_state, node->parent->physical_params.grav_param);

        print_physical_state(node->physical_state);
        print_orbital_elements(node->body_name,node->orbital_elements);

        node->asc_desc = compute_nodes(node->orbital_elements);

        // Free previous orbital lines
        if(node->orbital_lines != NULL) {
            darray_set_length(node->orbital_lines, 0);
        }

        Debug("Computing orbital lines\n");
        node->orbital_lines = compute_orbital_lines(node->physical_state, node->parent->physical_params.grav_param, clock.clock_seconds, node->physical_params.sphere_of_influence*2);
        Debug("Done computing orbital lines\n");
    }

    // Iterate over children & recurse!
    for (int i = 0; i < darray_length(node->children) && node->children != NULL; i++) {
        OrbitalTreeNode* child = (OrbitalTreeNode*)darray_get(node->children,i);
        Info("Updating %s child of %s\n",child->body_name,node->body_name);
        update_orbital_node(child, clock);
    }

    Debug("Returning from %s\n",node->body_name);
}


//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Loggeruuu
    InitializeLogger(LOG_LEVEL,true);

    PhysicsTimeClock clock = {
        .tick_interval_seconds = 86400,
        .mode = Elapsing,
        .scale = 10000.0,
        .delta_seconds = 0.0,
        .clock_seconds = 0.0
    };

    //--------------------------------------------------------------------------------------
    InitWindow(screenWidth, screenHeight, "C Orbit");

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

    OrbitalTreeNode* root = load_earth_moon_system();

    // Indicates if the camera
    bool is_click_to_drag_on = true;

    Matrix matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,10000000.0);

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float delta;

    darray orbital_lines;

    bool should_draw_orbital_features = true;

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        time = GetTime();
        delta = GetFrameTime();

        if(IsKeyDown(KEY_D) || IsKeyPressed(KEY_D)) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                clock.scale *= 1.1;
            } else {
                clock.scale *= 2;
            }
        }

        if(IsKeyDown(KEY_A) || IsKeyPressed(KEY_A)) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                clock.scale /= 1.5;
            } else {
                clock.scale /= 2;
            }
        }

        UpdatePhysicsClock(&clock, delta);

        Info("Updating orbital node...\n");
        update_orbital_node(root, clock);


        if(IsKeyPressed(KEY_V)) {
            should_draw_orbital_features = !should_draw_orbital_features;
        }


        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            spherical_camera_system(&r, &theta, &phi, &camera,is_click_to_drag_on);

            draw_orbital_node(root,clock,camera,should_draw_orbital_features,matProj);
            BeginMode3D(camera);
                rlSetMatrixProjection(matProj);

                if (should_draw_orbital_features) {
                    DrawLine3D((Vector3){0,0,0},TF(vector_from_physical_to_world((DVector3){EARTH_RADIUS_KM*10,0,0})),RED);
                    DrawLine3D((Vector3){0,0,0},TF(vector_from_physical_to_world((DVector3){0,EARTH_RADIUS_KM*10,0})),GREEN);
                    DrawLine3D((Vector3){0,0,0},TF(vector_from_physical_to_world((DVector3){0,0,EARTH_RADIUS_KM*10})),BLUE);
                }

                Info("Drawing orbital node...\n");

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
