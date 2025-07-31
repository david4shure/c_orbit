/*******************************************************************************************
*
*   C Orbit
*
*   david4shure
*
*   MIT License, give me credit pls
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
#include <string.h>
#include "raygui.h"

#include "math.h"

#include "physics/constants.h"
#include "physics/time.h"
#include "physics/kepler.h"
#include "tree.h"
#include "utils/darray.h"
#include "utils/logger.h"
#include "physics/propagation.h"

#define LOG_LEVEL DEBUG

const int screenWidth = 1500;
const int screenHeight = 1000;

static Font customFont;

Vector3 TF(DVector3 vec) {
    return (Vector3){.x = (float)vec.x, .y = (float)vec.y, .z= (float)vec.z};
}

DVector3 TD(Vector3 vec) {
    return (DVector3){.x = (double)vec.x, .y = (double)vec.y, .z= (double)vec.z};
}

typedef struct ProgramState {
    OrbitalTreeNode* tree;
    PhysicsTimeClock* clock;
    OrbitalTreeNode* focused_node;
    int sibling_index;
} ProgramState;

typedef struct TimeDisplayInfo{
    int years;
    int weeks;
    int days;
    int hours;
    int minutes;
    int seconds;
} TimeDisplayInfo;

// ProgramState is malloc'ed
ProgramState* load_program_state() {
    ProgramState* program_state = malloc(sizeof(ProgramState));

    if (!program_state) {
        Error("Failed to load program state\n");
        return NULL;
    }

    // Set top level tree
    program_state->tree = load_earth_moon_system();

    // Default the program to the root of the tree
    program_state->focused_node = program_state->tree;

    PhysicsTimeClock* clock = malloc(sizeof(PhysicsTimeClock));
    clock->tick_interval_seconds = 86400;
    clock->mode = Elapsing;
    clock->scale = 10000.0;
    clock->delta_seconds = 0.0;
    clock->clock_seconds = 0.0;

    if (!clock) {
        Error("Failed to malloc physics time clock\n");
        return NULL;
    }

    program_state->clock = clock;
    program_state->sibling_index = 0;

    return program_state;
}

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


void draw_orbital_lines(darray orbital_lines, OrbitalTreeNode* node, Camera3D camera, DVector3 center) {
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
            DrawLine3D(TF(vector_from_physical_to_world(DVector3Add(current_pos->point,center))), TF(vector_from_physical_to_world(DVector3Add(prev_pos->point,center))), myred);
        } else {
            DrawLine3D(TF(vector_from_physical_to_world(DVector3Add(current_pos->point,center))), TF(vector_from_physical_to_world(DVector3Add(prev_pos->point,center))), node->line_color);
        }

        prev_pos = current_pos;
    }

    Color myyellow = (Color){ 253, 249, 0, 100};

    if (node->orbital_elements.eccentricity < 1.0) {
        DrawLine3D(TF(vector_from_physical_to_world(DVector3Add(node->asc_desc.asc,center))), TF(vector_from_physical_to_world(DVector3Add(node->asc_desc.desc,center))), myyellow);
    }
}

void draw_orbital_features(OrbitalTreeNode* node, PhysicsTimeClock* clock, Camera3D camera, DVector3 center) {
    float max_distance = 5000.0;
    float camera_to_moon_distance = Vector3Length((Vector3){camera.position.x-node->physical_state.r.x,camera.position.y-node->physical_state.r.y,camera.position.z-node->physical_state.r.z});
    float distance_scale_factor = max_distance/camera_to_moon_distance;
    float time_scale_factor = sin(clock->clock_seconds*1.5)/4.0 + 1;
    DVector3 body_pos_world = vector_from_physical_to_world(DVector3Add(node->physical_state.r,center));

    float scale_factor = distance_scale_factor * time_scale_factor;
    double velocity = DVector3Length(node->physical_state.v);
    char velocity_str[20];  // Buffer for the formatted string
    // Format the float as a string
    snprintf(velocity_str, sizeof(velocity_str), "V=%.2f KM/s", velocity);

    // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    float dist = DVector3Distance(body_pos_world, TD(camera.position));

    DVector3 periapsis_point = solve_kepler_ellipse_inertial(node->orbital_elements, 0.0, 0.0, 0.0);
    DVector3 apoapsis_point = solve_kepler_ellipse_inertial(node->orbital_elements, 0.0, 0.0, node->orbital_elements.period/2.0);
    DVector3 asc_node_point = node->asc_desc.asc;
    DVector3 desc_node_point = node->asc_desc.desc;
    DVector3 positive_x = (DVector3){EARTH_RADIUS_KM*10,0,0};
    DVector3 positive_y = (DVector3){0,EARTH_RADIUS_KM*10,0};
    DVector3 positive_z = (DVector3){0,0,EARTH_RADIUS_KM*10};

    // Account for center offset
    periapsis_point = DVector3Add(periapsis_point, center);
    apoapsis_point  = DVector3Add(apoapsis_point, center);
    asc_node_point  = DVector3Add(asc_node_point, center);
    desc_node_point = DVector3Add(desc_node_point, center);
    positive_x      = DVector3Add(positive_x, center);
    positive_y      = DVector3Add(positive_y, center);
    positive_z      = DVector3Add(positive_z, center);

    // Vector from physical to render coordinates
    periapsis_point = vector_from_physical_to_world(periapsis_point);
    apoapsis_point  = vector_from_physical_to_world(apoapsis_point);
    asc_node_point  = vector_from_physical_to_world(asc_node_point);
    desc_node_point = vector_from_physical_to_world(desc_node_point);
    positive_x      = vector_from_physical_to_world(positive_x);
    positive_y      = vector_from_physical_to_world(positive_y);
    positive_z      = vector_from_physical_to_world(positive_z);

    Vector2 periapsis_point_camera = GetWorldToScreen(TF(periapsis_point), camera);
    Vector2 apoapsis_point_camera = GetWorldToScreen(TF(apoapsis_point), camera);
    Vector2 asc_point_camera = GetWorldToScreen(TF(asc_node_point), camera);
    Vector2 desc_point_camera = GetWorldToScreen(TF(desc_node_point), camera);

    Vector2 body = GetWorldToScreen(TF(body_pos_world), camera);
    Vector2 positive_x_direction = GetWorldToScreen(TF(positive_x),camera);
    Vector2 positive_y_direction = GetWorldToScreen(TF(positive_y),camera);
    Vector2 positive_z_direction = GetWorldToScreen(TF(positive_z),camera);

    if (!is_object_behind_camera(camera.position, camera.target, TF(body_pos_world))) {
        // Measure text sizes
        Vector2 bodyLabelSize = MeasureTextEx(customFont, node->body_name, 12, 0);
        Vector2 velocityLabelSize = MeasureTextEx(customFont, velocity_str, 12, 0);

        // Layout spacing
        float labelSpacing = 4;       // spacing between labels
        float verticalOffset = 20;    // how high to lift the whole label group above the body

        // Compute label positions
        Vector2 bodyLabelPos = {
            body.x - bodyLabelSize.x / 2.0f,
            body.y - bodyLabelSize.y - verticalOffset
        };

        Vector2 velocityLabelPos = {
            body.x - velocityLabelSize.x / 2.0f,
            bodyLabelPos.y - velocityLabelSize.y - labelSpacing
        };

        // Draw text labels
        DrawTextEx(customFont, velocity_str, velocityLabelPos, 12, 0, GREEN);
        DrawTextEx(customFont, node->body_name, bodyLabelPos, 12, 0, GREEN);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(periapsis_point))) {
        DrawTextEx(customFont, "Pa",
            (Vector2){
                (int)periapsis_point_camera.x - MeasureTextEx(customFont, "Pa", 12, 0).x / 2.0f,
                (int)periapsis_point_camera.y - MeasureTextEx(customFont, "Pa", 12, 0).y
            }, 12, 0, SKYBLUE);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(apoapsis_point))) {
        DrawTextEx(customFont, "Apo",
            (Vector2){
                (int)apoapsis_point_camera.x - MeasureTextEx(customFont, "Apo", 12, 0).x / 2.0f,
                (int)apoapsis_point_camera.y - MeasureTextEx(customFont, "Apo", 12, 0).y
            }, 12, 0, DARKBLUE);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_x))) {
        DrawTextEx(customFont, "+x", (Vector2){(int)positive_x_direction.x, (int)positive_x_direction.y}, 12, 0, LIGHTGRAY);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_y))) {
        DrawTextEx(customFont, "+y", (Vector2){(int)positive_y_direction.x, (int)positive_y_direction.y}, 12, 0, LIGHTGRAY);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_z))) {
        DrawTextEx(customFont, "+z", (Vector2){(int)positive_z_direction.x, (int)positive_z_direction.y}, 12, 0, LIGHTGRAY);
    }

    if (node->orbital_elements.eccentricity >= 1.0) return;

    if (!is_object_behind_camera(camera.position, camera.target, TF(asc_node_point))) {
        DrawTextEx(customFont, "Asc",
            (Vector2){
                (int)asc_point_camera.x - MeasureTextEx(customFont, "Asc", 12, 0).x / 2.0f,
                (int)asc_point_camera.y
            }, 12, 0, YELLOW);
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(desc_node_point))) {
        DrawTextEx(customFont, "Desc",
            (Vector2){
                (int)desc_point_camera.x - MeasureTextEx(customFont, "Desc", 12, 0).x / 2.0f,
                (int)desc_point_camera.y
            }, 12, 0, ORANGE);
    }

}

void draw_element_str(char* format_text, char* value, int x, int y, Color color) {
    char buffer[50];  // Buffer for the formatted string
            // Format the float as a string
    snprintf(buffer, sizeof(buffer), format_text, value);

    Vector2 pos = {x,y};

    DrawTextEx(customFont,buffer,pos,15,0,color);
}

void draw_element(char* format_text, double value, int x, int y, Color color) {
    char buffer[50];  // Buffer for the formatted string
            // Format the float as a string
    snprintf(buffer, sizeof(buffer), format_text, value);

    Vector2 pos = {x,y};

    DrawTextEx(customFont,buffer,pos,15,0,color);
}

void draw_textual_orbital_elements(ClassicalOrbitalElements oe, int num_lines) {
    int left_padding = 15;
    int padding_between_rows = 25;
    Color text_color = RED;

    draw_element("e = %.3f", oe.eccentricity, left_padding, padding_between_rows, text_color);
    draw_element("a = %.3f", oe.semimajor_axis, left_padding, padding_between_rows * 2, text_color);
    draw_element("true anomaly = %.3f degrees", oe.true_anomaly * RAD2DEG, left_padding, padding_between_rows * 3, text_color);
    draw_element("argument of periapsis = %.3f degrees", oe.arg_of_periapsis * RAD2DEG, left_padding, padding_between_rows * 4, text_color);
    draw_element("inclination = %.3f degrees", oe.inclination * RAD2DEG, left_padding, padding_between_rows * 5, text_color);
    draw_element("longitude of the ascending node = %.3f degrees", oe.long_of_asc_node * RAD2DEG, left_padding, padding_between_rows * 6, text_color);
    draw_element("number of lines = %.0f", (double)num_lines, screenWidth-180, padding_between_rows, GREEN);

    TimeDisplayInfo time_info = get_time_info(oe.period);

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "orbital period = %d years %d weeks %d days %d hours %d minutes %d seconds", time_info.years, time_info.weeks, time_info.days, time_info.hours, time_info.minutes, time_info.seconds);
    Vector2 pos = {left_padding,padding_between_rows*7};

    DrawTextEx(customFont, buffer,pos,15,0,PURPLE);
}

void draw_orbital_hierarchy(OrbitalTreeNode* focus, OrbitalTreeNode* tree, int depth, int width) {
    if (tree == NULL) {
        return;
    }

    int vertical_padding = GetScreenHeight() - 100;
    int horizontal_padding = 30;

    int horizontal_indentation_amount = 10;
    int vertical_indentation_amount = 10;

    int vertical_offset = depth * vertical_indentation_amount + vertical_padding;
    int horizontal_offset = width * horizontal_indentation_amount + horizontal_padding;

    if (tree == focus) {
        Vector2 pos = {horizontal_offset-20,vertical_offset};
        DrawTextEx(customFont,"->  ",pos,15,0,BLUE);
        pos.x = horizontal_offset;
        pos.y = vertical_offset;
        DrawTextEx(customFont,tree->body_name,pos,15,0,BLUE);
    } else {
        Vector2 pos = {horizontal_offset,vertical_offset};
        DrawTextEx(customFont,tree->body_name,pos,15,0,LIME);
    }

    for (int j = 0; tree->children != NULL && j < darray_length(tree->children); j++) {
        depth++;
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(tree->children,j);

        draw_orbital_hierarchy(focus,*child, depth+1, width+1);
    }
}

void draw_clock_info(PhysicsTimeClock* clock) {
    TimeDisplayInfo time_info = get_time_info(clock->clock_seconds);

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%d years %d weeks %d days %d hours %d minutes %d seconds", time_info.years, time_info.weeks, time_info.days, time_info.hours, time_info.minutes, time_info.seconds);

    DrawText(buffer,screenWidth/2-140,10,10,VIOLET);
}

void draw_body(OrbitalTreeNode* node, Camera3D camera, DVector3 center) {
    Vector3 position = TF(vector_from_physical_to_world(DVector3Add(center, node->physical_state.r)));
    DrawSphereWires(position,node->physical_params.radius * KM_TO_RENDER_UNITS,10,10,node->body_color);
}

void draw_sphere_of_influence(OrbitalTreeNode* node, Camera3D camera, DVector3 center) {
    Vector3 position = TF(vector_from_physical_to_world(DVector3Add(center, node->physical_state.r)));
    if (node->draw_sphere_of_influence) {
        DrawSphereWires(position,node->physical_params.sphere_of_influence * KM_TO_RENDER_UNITS,10,10,(Color){.r=255, .b=182, .g=193,.a=50});
    }
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
void draw_orbital_tree_recursive(OrbitalTreeNode* root, OrbitalTreeNode* node, PhysicsTimeClock* clock, Camera3D camera3d, bool should_draw_orbital_features, Matrix proj) {
    bool is_root_node = node->parent == NULL;

    DVector3 center = get_offset_position_for_node(root,node);

    if (!is_root_node && should_draw_orbital_features) {
        draw_orbital_features(node, clock, camera3d, center);
    }

    BeginMode3D(camera3d);
    rlSetMatrixProjection(proj);
    if (should_draw_orbital_features) {
        draw_orbital_lines(node->orbital_lines, node, camera3d, center);
    }
    draw_sphere_of_influence(node,camera3d,center);
    draw_body(node,camera3d,center);
    EndMode3D();

    // Iterate over children & recurse!
    for (int i = 0; i < darray_length(node->children); i++) {
        OrbitalTreeNode** item = (OrbitalTreeNode**)darray_get(node->children,i);
        draw_orbital_tree_recursive(root, *item, clock, camera3d, should_draw_orbital_features,proj);
    }
}

// Star field data
#define NUM_STARS 2500
static Vector3 stars[NUM_STARS];
static Color star_colors[NUM_STARS];
static bool stars_initialized = false;

// Generate random stars on a sphere around the origin
void generate_star_field(void) {
    if (stars_initialized) return;
    
    SetRandomSeed(12345); // Fixed seed for consistent star field
    float star_distance = 1000000.0f; // Very far away
    
    for (int i = 0; i < NUM_STARS; i++) {
        // Generate uniform random point on unit sphere (avoiding pole bunching)
        float theta = GetRandomValue(0, 360) * DEG2RAD;  // Azimuth: 0 to 2π
        float u = GetRandomValue(0, 100) / 100.0f;       // Uniform 0 to 1
        float phi = acosf(1.0f - 2.0f * u);              // Proper elevation distribution
        
        stars[i].x = star_distance * sinf(phi) * cosf(theta);
        stars[i].y = star_distance * cosf(phi);
        stars[i].z = star_distance * sinf(phi) * sinf(theta);
        
        // Generate star colors with much better brightness distribution
        int brightness_type = GetRandomValue(0, 100);
        int brightness;
        
        if (brightness_type < 10) {
            // Very bright stars (10%)
            brightness = GetRandomValue(240, 255);
        } else if (brightness_type < 30) {
            // Bright stars (20%)
            brightness = GetRandomValue(200, 240);
        } else if (brightness_type < 70) {
            // Medium stars (40%)
            brightness = GetRandomValue(160, 200);
        } else {
            // Dim stars (30%)
            brightness = GetRandomValue(120, 160);
        }
        
        int color_type = GetRandomValue(0, 100);
        
        if (color_type < 70) {
            // White stars (most common)
            star_colors[i] = (Color){brightness, brightness, brightness, 255};
        } else if (color_type < 85) {
            // Blue-white stars
            star_colors[i] = (Color){brightness-20, brightness-10, brightness, 255};
        } else if (color_type < 95) {
            // Yellow-white stars
            star_colors[i] = (Color){brightness, brightness, brightness-30, 255};
        } else {
            // Red stars
            star_colors[i] = (Color){brightness, brightness-40, brightness-60, 255};
        }
    }
    
    stars_initialized = true;
}

// Draw the star field
void draw_star_field(Camera3D camera) {
    if (!stars_initialized) {
        generate_star_field();
    }
    
    // Stars are at infinite distance, so always draw all of them
    // Only adjust size slightly based on zoom for visual comfort
    float camera_distance = Vector3Length(camera.position);
    int stars_to_draw = NUM_STARS;  // Always draw all stars
    float star_size = 1200.0f;
    
    // Slight size adjustment for visual comfort only
    if (camera_distance < 30000.0f) {
        star_size = 1000.0f;  // Slightly smaller when close
    } else if (camera_distance > 150000.0f) {
        star_size = 1400.0f;  // Slightly larger when far
    }
    
    // Draw stars as crosses for better visibility
    for (int i = 0; i < stars_to_draw; i++) {
        // Translate stars relative to camera position for infinite distance effect
        Vector3 star_pos = {
            stars[i].x + camera.position.x,
            stars[i].y + camera.position.y,
            stars[i].z + camera.position.z
        };
        
        // Draw as crosses for much better visibility
        Vector3 h1 = {star_pos.x - star_size, star_pos.y, star_pos.z};
        Vector3 h2 = {star_pos.x + star_size, star_pos.y, star_pos.z};
        Vector3 v1 = {star_pos.x, star_pos.y - star_size, star_pos.z};
        Vector3 v2 = {star_pos.x, star_pos.y + star_size, star_pos.z};
        
        DrawLine3D(h1, h2, star_colors[i]);
        DrawLine3D(v1, v2, star_colors[i]);
    }
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Logger
    InitializeLogger(LOG_LEVEL,true);

    // We need to free this, it was alloc'ed
    ProgramState* program_state = load_program_state();

    //--------------------------------------------------------------------------------------
    InitWindow(screenWidth, screenHeight, "C Orbit");

    // Load the font
    customFont = LoadFontEx("FiraCode-Bold.ttf", 32, 0, 0);

    // Wait for font texture to be ready
    SetTextureFilter(customFont.texture, TEXTURE_FILTER_BILINEAR);

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

    // Indicates if the camera
    bool is_click_to_drag_on = true;

    Matrix matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,10000000.0);

    EnableCursor();                     // Start with cursor visible for click-to-drag mode

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

        if(IsKeyPressed(KEY_U)) {
            if (is_click_to_drag_on) {
                DisableCursor();  // Switch to free look mode (hide cursor)
            } else {
                EnableCursor();   // Switch to click-to-drag mode (show cursor)
            }
            is_click_to_drag_on = !is_click_to_drag_on;
        }

        if(IsKeyDown(KEY_D) || IsKeyPressed(KEY_D)) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                program_state->clock->scale *= 1.1;
            } else {
                program_state->clock->scale *= 2;
            }
        }

        if(IsKeyDown(KEY_A) || IsKeyPressed(KEY_A)) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                program_state->clock->scale /= 1.5;
            } else {
                program_state->clock->scale /= 2;
            }
        }

        // If H is pressed, move to the left sibling of the current node
        if(IsKeyPressed(KEY_H)) {
            Error("H pressed.\n");
            if (program_state->focused_node->parent != NULL) {
                OrbitalTreeNode* parent = program_state->focused_node->parent;
                // Grab left sibiling
                if (program_state->sibling_index > 0) {
                    program_state->sibling_index--;
                    // Grab sibling at sibling index
                    darray parents_children = parent->children;
                    OrbitalTreeNode** left_sibiling = (OrbitalTreeNode**)darray_get(parents_children,program_state->sibling_index);
                    program_state->focused_node = *left_sibiling;
                }
            }
        }

        // If K is pressed, move to the right sibling of the current node
        if(IsKeyPressed(KEY_L)) {
            Error("L pressed.\n");
            if (program_state->focused_node->parent != NULL) {
                OrbitalTreeNode* parent = program_state->focused_node->parent;
                darray parents_children = parent->children;
                // Grab right sibiling
                if (program_state->sibling_index < darray_length(parents_children) - 1) {
                    program_state->sibling_index++;
                    // Grab sibling at sibling index
                    OrbitalTreeNode** right_sibiling = (OrbitalTreeNode**)darray_get(parents_children,program_state->sibling_index);
                    program_state->focused_node = *right_sibiling;
                }
            }
        }

        // Go down the orbital hierarchy to the first child of the currently focused node
        if(IsKeyPressed(KEY_J)) {
            Error("J pressed.\n");
            bool has_children = program_state->focused_node->children != NULL && darray_length(program_state->focused_node->children) > 0;
            if (has_children) {
                OrbitalTreeNode** first_child = (OrbitalTreeNode**)darray_get(program_state->focused_node->children,0);
                program_state->focused_node = *first_child;
            }
        }

        // Go up the orbital hierarchy to the parent of the currently focused node (if applicable)
        if(IsKeyPressed(KEY_K)) {
            Error("K pressed.\n");
            if (program_state->focused_node->parent != NULL) {
                program_state->focused_node = program_state->focused_node->parent;
            }
        }

        // Apply prograde force
        if (IsKeyDown(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
            DVector3 forward = DVector3Normalize(program_state->focused_node->physical_state.v);
            forward = DVector3Scale(forward,0.01);
            program_state->focused_node->physical_state = apply_force_to(program_state->focused_node->physical_state,forward,30.0);
        }
        // Apply retrograde force
        if (IsKeyDown(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
            DVector3 backward = DVector3Normalize(program_state->focused_node->physical_state.v);
            backward = DVector3Scale(backward,-0.01);
            program_state->focused_node->physical_state = apply_force_to(program_state->focused_node->physical_state,backward,30.0);
        }
        // Apply normal force
        if (IsKeyDown(KEY_UP) || IsKeyPressed(KEY_UP)) {
            DVector3 up = DVector3Normalize(DVector3CrossProduct(program_state->focused_node->physical_state.r, program_state->focused_node->physical_state.v));
            up = DVector3Scale(up,0.01);
            program_state->focused_node->physical_state = apply_force_to(program_state->focused_node->physical_state,up,300.0);
        }
        // Apply anti-normal force
        if (IsKeyDown(KEY_DOWN) || IsKeyPressed(KEY_DOWN)) {
            DVector3 down = DVector3Normalize(DVector3CrossProduct(program_state->focused_node->physical_state.r, program_state->focused_node->physical_state.v));
            down = DVector3Scale(down,-0.01);
            program_state->focused_node->physical_state = apply_force_to(program_state->focused_node->physical_state,down,300.0);
        }

        UpdatePhysicsClock(program_state->clock, delta);

        restructure_orbital_tree_recursive(program_state->tree,program_state->tree);
        update_orbital_tree_recursive(program_state->tree,program_state->tree,program_state->clock);
        darray list = darray_init(10,sizeof(OrbitalTreeNode**));
        darray bodies = dfs_orbital_tree_nodes(program_state->tree, list);

        for (int i = 0; darray_length(bodies) > 0 && i < darray_length(bodies);i++) {
            OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(bodies,i);
        }

        if(IsKeyPressed(KEY_V)) {
            should_draw_orbital_features = !should_draw_orbital_features;
        }

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            if (program_state != NULL && program_state->focused_node != NULL) {
                int base_len = 14;
                base_len += strlen(program_state->focused_node->body_name);
                char buffer[base_len];
                sprintf(buffer, "Focused node = %s", program_state->focused_node->body_name);
                draw_element_str(buffer, program_state->focused_node->body_name, screenWidth/2-MeasureText(buffer, 10)/2, 20, BLUE);
            }

            ClearBackground(BLACK);

            PhysicalState focus_rv = global_physical_state(program_state->tree, program_state->focused_node);

            DVector3 world_r = vector_from_physical_to_world(focus_rv.r);

            camera.target = TF(world_r);

            spherical_camera_system(world_r, &r, &theta, &phi, &camera, !is_click_to_drag_on);

            draw_orbital_tree_recursive(program_state->tree,program_state->tree,program_state->clock,camera,should_draw_orbital_features,matProj);
            draw_orbital_hierarchy(program_state->focused_node,program_state->tree, 0, 0);
            if (program_state->focused_node->parent != NULL) {
                draw_textual_orbital_elements(program_state->focused_node->orbital_elements, darray_length(program_state->focused_node->orbital_lines));
            }
            BeginMode3D(camera);
                rlSetMatrixProjection(matProj);

                // Draw star field in the background
                draw_star_field(camera);

                if (should_draw_orbital_features) {
                    DrawLine3D((Vector3){0,0,0},TF(vector_from_physical_to_world((DVector3){EARTH_RADIUS_KM*10,0,0})),RED);
                    DrawLine3D((Vector3){0,0,0},TF(vector_from_physical_to_world((DVector3){0,EARTH_RADIUS_KM*10,0})),GREEN);
                    DrawLine3D((Vector3){0,0,0},TF(vector_from_physical_to_world((DVector3){0,0,EARTH_RADIUS_KM*10})),BLUE);
                }

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
