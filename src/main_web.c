/*******************************************************************************************
*
*   C Orbit
*
*   david4shure
*
*   MIT License, give me credit pls
*
********************************************************************************************/

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "physics/corbit_math.h"
#include "physics/orbital_lines.h"
#include "physics/time.h"
#include "physics/kepler.h"
#include "physics/propagation.h"
#include "physics/constants.h"
#include "utils/logger.h"
#include "utils/darray.h"
#include "tree.h"
#include "camera.h"
#include "emscripten.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define LOG_LEVEL DEBUG



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
    // Use integer arithmetic to avoid floating point precision issues
    long long total_secs = (long long)total_seconds;
    
    int years = total_secs / SECONDS_IN_YEAR;
    total_secs %= SECONDS_IN_YEAR;
    
    int weeks = total_secs / SECONDS_IN_WEEK;
    total_secs %= SECONDS_IN_WEEK;
    
    int days = total_secs / SECONDS_IN_DAY;
    total_secs %= SECONDS_IN_DAY;
    
    int hours = total_secs / SECONDS_IN_HOUR;
    total_secs %= SECONDS_IN_HOUR;
    
    int minutes = total_secs / SECONDS_IN_MINUTE;
    total_secs %= SECONDS_IN_MINUTE;
    
    int seconds = (int)total_secs; // Now guaranteed to be 0-59

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

        // Make orbital lines more visible with higher alpha
        Color bright_line_color = node->line_color;
        bright_line_color.a = 255; // Full opacity
        
        if (DVector3Length(current_pos->point) > r_at_sphere_of_influence) {
            Color bright_red = {255, 100, 100, 255};
            DrawLine3D(TF(vector_from_physical_to_world(DVector3Add(current_pos->point,center))), TF(vector_from_physical_to_world(DVector3Add(prev_pos->point,center))), bright_red);
        } else {
            DrawLine3D(TF(vector_from_physical_to_world(DVector3Add(current_pos->point,center))), TF(vector_from_physical_to_world(DVector3Add(prev_pos->point,center))), bright_line_color);
        }

        prev_pos = current_pos;
    }

    Color bright_yellow = (Color){ 255, 255, 0, 255}; // Full opacity yellow

    if (node->orbital_elements.eccentricity < 1.0) {
        DrawLine3D(TF(vector_from_physical_to_world(DVector3Add(node->asc_desc.asc,center))), TF(vector_from_physical_to_world(DVector3Add(node->asc_desc.desc,center))), bright_yellow);
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

    // Body and velocity labels are now drawn separately in draw_body_labels() - removed duplicate code

    if (!is_object_behind_camera(camera.position, camera.target, TF(periapsis_point))) {
        Vector2 pa_size = MeasureTextEx(customFont, "Pa", 20, 0);
        Vector2 pa_pos = {
            (int)periapsis_point_camera.x - pa_size.x / 2.0f,
            (int)periapsis_point_camera.y - pa_size.y - 15
        };
        Rectangle pa_bg = {pa_pos.x - 10, pa_pos.y - 6, pa_size.x + 20, pa_size.y + 12};
        DrawRectangleRec(pa_bg, (Color){0, 0, 0, 250});
        DrawRectangleLinesEx(pa_bg, 3, (Color){0, 255, 0, 255});
        DrawTextEx(customFont, "Pa", pa_pos, 20, 0, (Color){0, 255, 0, 255});
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(apoapsis_point))) {
        Vector2 apo_size = MeasureTextEx(customFont, "Apo", 20, 0);
        Vector2 apo_pos = {
            (int)apoapsis_point_camera.x - apo_size.x / 2.0f,
            (int)apoapsis_point_camera.y - apo_size.y - 15
        };
        Rectangle apo_bg = {apo_pos.x - 10, apo_pos.y - 6, apo_size.x + 20, apo_size.y + 12};
        DrawRectangleRec(apo_bg, (Color){0, 0, 0, 250});
        DrawRectangleLinesEx(apo_bg, 3, (Color){255, 0, 0, 255});
        DrawTextEx(customFont, "Apo", apo_pos, 20, 0, (Color){255, 0, 0, 255});
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_x)) && customFont.texture.id != 0) {
        Vector2 x_size = MeasureTextEx(customFont, "+X", 16, 0);
        Vector2 x_pos = {(int)positive_x_direction.x - x_size.x/2, (int)positive_x_direction.y - x_size.y/2};
        Rectangle x_bg = {x_pos.x - 8, x_pos.y - 4, x_size.x + 16, x_size.y + 8};
        DrawRectangleRec(x_bg, (Color){0, 0, 0, 240});
        DrawRectangleLinesEx(x_bg, 2, (Color){255, 100, 100, 255});
        DrawTextEx(customFont, "+X", x_pos, 16, 0, (Color){255, 100, 100, 255});
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_y)) && customFont.texture.id != 0) {
        Vector2 y_size = MeasureTextEx(customFont, "+Y", 16, 0);
        Vector2 y_pos = {(int)positive_y_direction.x - y_size.x/2, (int)positive_y_direction.y - y_size.y/2};
        Rectangle y_bg = {y_pos.x - 8, y_pos.y - 4, y_size.x + 16, y_size.y + 8};
        DrawRectangleRec(y_bg, (Color){0, 0, 0, 240});
        DrawRectangleLinesEx(y_bg, 2, (Color){100, 255, 100, 255});
        DrawTextEx(customFont, "+Y", y_pos, 16, 0, (Color){100, 255, 100, 255});
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(positive_z)) && customFont.texture.id != 0) {
        Vector2 z_size = MeasureTextEx(customFont, "+Z", 16, 0);
        Vector2 z_pos = {(int)positive_z_direction.x - z_size.x/2, (int)positive_z_direction.y - z_size.y/2};
        Rectangle z_bg = {z_pos.x - 8, z_pos.y - 4, z_size.x + 16, z_size.y + 8};
        DrawRectangleRec(z_bg, (Color){0, 0, 0, 240});
        DrawRectangleLinesEx(z_bg, 2, (Color){100, 100, 255, 255});
        DrawTextEx(customFont, "+Z", z_pos, 16, 0, (Color){100, 100, 255, 255});
    }

    if (node->orbital_elements.eccentricity >= 1.0) return;

    if (!is_object_behind_camera(camera.position, camera.target, TF(asc_node_point))) {
        Vector2 asc_size = MeasureTextEx(customFont, "Asc", 12, 0);
        Vector2 asc_pos = {
            (int)asc_point_camera.x - asc_size.x / 2.0f,
            (int)asc_point_camera.y - asc_size.y - 20
        };
        Rectangle asc_bg = {asc_pos.x - 4, asc_pos.y - 2, asc_size.x + 8, asc_size.y + 4};
        DrawRectangleRec(asc_bg, (Color){0, 0, 0, 180});
        DrawRectangleLinesEx(asc_bg, 1, (Color){135, 206, 235, 180});
        DrawTextEx(customFont, "Asc", asc_pos, 12, 0, (Color){135, 206, 235, 200});
    }

    if (!is_object_behind_camera(camera.position, camera.target, TF(desc_node_point))) {
        Vector2 desc_size = MeasureTextEx(customFont, "Desc", 12, 0);
        Vector2 desc_pos = {
            (int)desc_point_camera.x - desc_size.x / 2.0f,
            (int)desc_point_camera.y + 15
        };
        Rectangle desc_bg = {desc_pos.x - 4, desc_pos.y - 2, desc_size.x + 8, desc_size.y + 4};
        DrawRectangleRec(desc_bg, (Color){0, 0, 0, 180});
        DrawRectangleLinesEx(desc_bg, 1, (Color){255, 0, 255, 180});
        DrawTextEx(customFont, "Desc", desc_pos, 12, 0, (Color){255, 0, 255, 200});
    }

}

void draw_element_str(char* format_text, char* value, int x, int y, Color color) {
    if (customFont.texture.id == 0) {
        return; // Don't draw if font not loaded
    }
    
    char buffer[256];  // Larger buffer for safety
    snprintf(buffer, sizeof(buffer), format_text, value);

    Vector2 pos = {(float)x, (float)y};
    DrawTextEx(customFont, buffer, pos, 15, 0, color);
}

void draw_element(char* format_text, double value, int x, int y, Color color) {
    if (customFont.texture.id == 0) {
        return; // Don't draw if font not loaded
    }
    
    char buffer[256];  // Larger buffer for safety
    
    // Handle special cases for extreme values
    if (isnan(value) || isinf(value)) {
        snprintf(buffer, sizeof(buffer), "Invalid");
    } else {
        snprintf(buffer, sizeof(buffer), format_text, value);
    }

    Vector2 pos = {(float)x, (float)y};
    DrawTextEx(customFont, buffer, pos, 18, 0, color); // Bigger font
}

void draw_textual_orbital_elements(ClassicalOrbitalElements oe, int num_lines, const char* body_name) {
    if (customFont.texture.id == 0) {
        return; // Don't draw if font not loaded
    }
    
    int left_padding = 25;
    int padding_between_rows = 35;
    Color text_color = (Color){255, 255, 255, 255}; // Pure white for better contrast
    Color background_color = (Color){10, 15, 30, 240}; // Darker blue-black

    // Draw background for better readability - equal padding like markers
    int standard_padding = 25;
    Rectangle bg_rect = {
        25, // Same distance from left as markers from right
        20, // Same distance from top as markers
        400 + standard_padding * 2, // Width + equal padding on both sides
        310 + standard_padding * 2  // Height + equal padding top and bottom
    };
    
    // Add subtle gradient effect with multiple rectangles
    DrawRectangleRec(bg_rect, background_color);
    Rectangle highlight_rect = {bg_rect.x, bg_rect.y, bg_rect.width, 3};
    DrawRectangleRec(highlight_rect, (Color){80, 150, 255, 100});
    DrawRectangleLinesEx(bg_rect, 3, (Color){80, 150, 255, 255}); // Brighter blue border
    
    // Draw section title with body name - with proper padding
    Vector2 title_pos = {left_padding + standard_padding, 20 + standard_padding}; // Equal padding from edges
    char title_buffer[128];
    snprintf(title_buffer, sizeof(title_buffer), "ORBITAL ELEMENTS: %s", body_name);
    DrawTextEx(customFont, title_buffer, title_pos, 18, 0, WHITE); // White like other section headers

    // Draw orbital elements with better contrast and color coding  
    // Color code eccentricity: green -> red -> purple based on value
    Color ecc_color;
    if (oe.eccentricity < 0.5) {
        // Green to yellow (0 to 0.5)
        float ratio = oe.eccentricity / 0.5f;
        ecc_color = (Color){(int)(ratio * 255), 255, 0, 255};
    } else if (oe.eccentricity < 1.0) {
        // Yellow to red (0.5 to 1.0)
        float ratio = (oe.eccentricity - 0.5f) / 0.5f;
        ecc_color = (Color){255, (int)(255 * (1.0f - ratio)), 0, 255};
    } else {
        // Red to purple (1.0+)
        float ratio = fminf((oe.eccentricity - 1.0f) / 2.0f, 1.0f);
        ecc_color = (Color){255, 0, (int)(ratio * 128), 255};
    }
    draw_element("Eccentricity: %.6f", oe.eccentricity, left_padding + standard_padding, 75, ecc_color);
    draw_element("Semi-major axis: %.1f km", oe.semimajor_axis, left_padding + standard_padding, 110, WHITE);
    
    // Handle angle wrapping and extreme values
    double true_anomaly_deg = fmod(oe.true_anomaly * RAD2DEG + 360.0, 360.0);
    draw_element("True anomaly: %.2f deg", true_anomaly_deg, left_padding + standard_padding, 145, WHITE);
    
    // Color code periapsis-related elements to match Pa marker
    double arg_periapsis_deg = fmod(oe.arg_of_periapsis * RAD2DEG + 360.0, 360.0);
    draw_element("Arg of periapsis: %.2f deg", arg_periapsis_deg, left_padding + standard_padding, 180, WHITE);
    
    double inclination_deg = fmod(oe.inclination * RAD2DEG + 360.0, 360.0);
    draw_element("Inclination: %.2f deg", inclination_deg, left_padding + standard_padding, 215, WHITE);
    
    // Color code ascending node related elements to match Asc marker  
    double long_asc_node_deg = fmod(oe.long_of_asc_node * RAD2DEG + 360.0, 360.0);
    draw_element("Long. asc. node: %.2f deg", long_asc_node_deg, left_padding + standard_padding, 250, WHITE); // White like others
    

    draw_element("Orbital lines: %d", num_lines, left_padding + standard_padding, 285, WHITE); // White like others

    // Draw orbital period with better formatting
    TimeDisplayInfo time_info = get_time_info(oe.period);
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "Period: %d years, %d weeks, %d days, %d hours", 
             time_info.years, time_info.weeks, time_info.days, time_info.hours);
    Vector2 pos = {(float)(left_padding + standard_padding), 320}; // Equal padding from bottom
    DrawTextEx(customFont, buffer, pos, 18, 0, (Color){255, 255, 255, 255}); // Match other elements font size
}

void draw_orbital_hierarchy(OrbitalTreeNode* focus, OrbitalTreeNode* tree, int depth, int width) {
    if (tree == NULL || customFont.texture.id == 0) {
        return;
    }

    int vertical_padding = GetScreenHeight() - 150; // More room for content
    int horizontal_padding = 25;

    int horizontal_indentation_amount = 30; // More clear indentation
    int vertical_indentation_amount = 18; // Much tighter spacing to fit in box

    int vertical_offset = depth * vertical_indentation_amount + vertical_padding;
    int horizontal_offset = width * horizontal_indentation_amount + horizontal_padding;

    // Add background for hierarchy visibility
    if (depth == 0) {
        Rectangle hier_bg = {
            horizontal_padding - 15, 
            vertical_padding - 15, 
            250, // Wide enough for all text
            120  // Tall enough for all 3 bodies + proper padding
        };
        DrawRectangleRec(hier_bg, (Color){25, 15, 35, 250}); // Darker, more opaque
        Rectangle hier_highlight = {hier_bg.x, hier_bg.y, hier_bg.width, 3};
        DrawRectangleRec(hier_highlight, (Color){120, 200, 255, 150}); // Brighter highlight
        DrawRectangleLinesEx(hier_bg, 2, (Color){120, 200, 255, 255}); // Thinner, brighter border
    }
    
    if (tree == focus) {
        Vector2 pos = {horizontal_offset-35,vertical_offset};
        DrawTextEx(customFont,"-> ",pos,20,0,(Color){120, 200, 255, 255}); // ASCII arrow
        pos.x = horizontal_offset;
        pos.y = vertical_offset;
        DrawTextEx(customFont,tree->body_name,pos,20,0,(Color){255, 255, 255, 255}); // White for contrast
    } else {
        Vector2 pos = {horizontal_offset,vertical_offset};
        DrawTextEx(customFont,tree->body_name,pos,18,0,(Color){180, 220, 180, 255}); // Slightly dimmed green
    }

    for (int j = 0; tree->children != NULL && j < darray_length(tree->children); j++) {
        depth++;
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(tree->children,j);

        draw_orbital_hierarchy(focus,*child, depth+1, width+1);
    }
}

void draw_legend() {
    if (customFont.texture.id == 0) {
        return; // Don't draw if font not loaded
    }
    
    // Legend items
    const char* legend_items[] = {
        "ORBITAL MARKERS:",
        "Pa - Periapsis",
        "Apo - Apoapsis", 
        "Asc - Ascending Node",
        "Desc - Descending Node",
        "+X/+Y/+Z - Coordinate Axes"
    };
    Color legend_colors[] = {
        WHITE,
        LIME,
        RED,
        SKYBLUE,
        MAGENTA,
        WHITE
    };
    
    int legend_count = 6;
    int line_height = 28;
    int padding = 25; // Consistent with other sections
    int font_size = 18;
    
    // Calculate legend box size
    int max_width = 0;
    for (int i = 0; i < legend_count; i++) {
        int width = MeasureTextEx(customFont, legend_items[i], font_size, 0).x;
        if (width > max_width) max_width = width;
    }
    
    // Draw legend background (top-right corner) - bigger size
    Rectangle legend_bg = {
        GetScreenWidth() - max_width - padding * 2 - 25, // Consistent right margin 
        20, 
        max_width + padding * 2, 
        line_height * legend_count + padding * 2
    };
    DrawRectangleRec(legend_bg, (Color){30, 10, 10, 240}); // Darker red-black
    Rectangle legend_highlight = {legend_bg.x, legend_bg.y, legend_bg.width, 3};
    DrawRectangleRec(legend_highlight, (Color){255, 100, 100, 120});
    DrawRectangleLinesEx(legend_bg, 3, (Color){255, 100, 100, 255}); // Brighter red border
    
    // Draw legend text
    for (int i = 0; i < legend_count; i++) {
        Vector2 pos = {
            legend_bg.x + padding,
            legend_bg.y + padding + i * line_height
        };
        DrawTextEx(customFont, legend_items[i], pos, font_size, 0, legend_colors[i]);
    }
}

void draw_controls() {
    if (customFont.texture.id == 0) {
        return; // Don't draw if font not loaded
    }

    // Controls items with better explanations
    const char* control_items[] = {
        "CAMERA CONTROLS:",
        "- Mouse Drag: Rotate view around object",
        "- Mouse Scroll: Zoom in and out", 
        "- R Key: Reset camera to default position",
        "",
        "NAVIGATION CONTROLS:",
        "- J/K Keys: Focus parent/child objects",
        "- H/L Keys: Switch between sibling objects",
        "- A/D Keys: Slow down/speed up time",
        "- U Key: Toggle mouse camera control",
        "- V Key: Toggle orbital features display"
    };
    Color control_colors[] = {
        WHITE, // White for section headers
        (Color){200, 200, 200, 255},
        (Color){200, 200, 200, 255},
        (Color){200, 200, 200, 255},
        WHITE,
        WHITE, // White for section headers
        (Color){200, 200, 200, 255},
        (Color){200, 200, 200, 255},
        (Color){200, 200, 200, 255},
        (Color){200, 200, 200, 255},
        (Color){200, 200, 200, 255}
    };
    
    int control_count = 11;
    int line_height = 30; // Even more spacing
    int padding = 25; // Consistent with other sections
    int font_size = 18; // Match other sections
    
    // Calculate controls box size
    int max_width = 0;
    for (int i = 0; i < control_count; i++) {
        int width = MeasureTextEx(customFont, control_items[i], font_size, 0).x;
        if (width > max_width) max_width = width;
    }
    
    // Draw controls background (bottom right, avoiding hierarchy) - bigger size
    Rectangle controls_bg = {
        GetScreenWidth() - max_width - padding * 2 - 25, // Consistent right margin 
        GetScreenHeight() - (line_height * control_count + padding * 2) - 180, // Above hierarchy
        max_width + padding * 2, 
        line_height * control_count + padding * 2
    };
    DrawRectangleRec(controls_bg, (Color){10, 30, 10, 240}); // Darker green-black
    Rectangle controls_highlight = {controls_bg.x, controls_bg.y, controls_bg.width, 3};
    DrawRectangleRec(controls_highlight, (Color){100, 255, 100, 120});
    DrawRectangleLinesEx(controls_bg, 3, (Color){100, 255, 100, 255}); // Brighter green border
    
    // Draw controls text
    for (int i = 0; i < control_count; i++) {
        if (strlen(control_items[i]) > 0) { // Skip empty lines
            Vector2 pos = {
                controls_bg.x + padding,
                controls_bg.y + padding + i * line_height
            };
            DrawTextEx(customFont, control_items[i], pos, font_size, 0, control_colors[i]);
        }
    }
}

void draw_clock_info(PhysicsTimeClock* clock) {
    TimeDisplayInfo time_info = get_time_info(clock->clock_seconds);

    char buffer[200];
    snprintf(buffer, sizeof(buffer), "Time: %02dy %02dw %02dd %02dh %02dm %02ds", 
             time_info.years, time_info.weeks, time_info.days, 
             time_info.hours, time_info.minutes, time_info.seconds);

    // Calculate text width for proper centering
    int text_width = MeasureTextEx(customFont, buffer, 16, 0).x;
    
    // Draw background
    Rectangle bg_rect = {
        GetScreenWidth()/2 - text_width/2 - 15, 
        15, 
        text_width + 30, 
        25
    };
    DrawRectangleRec(bg_rect, (Color){30, 10, 30, 240}); // Darker purple-black
    Rectangle clock_highlight = {bg_rect.x, bg_rect.y, bg_rect.width, 3};
    DrawRectangleRec(clock_highlight, (Color){255, 150, 255, 120});
    DrawRectangleLinesEx(bg_rect, 3, (Color){255, 150, 255, 255}); // Brighter purple border
    
    // Draw the text
    Vector2 pos = {(float)(GetScreenWidth()/2 - text_width/2), 20};
    DrawTextEx(customFont, buffer, pos, 16, 0, (Color){200, 150, 255, 255}); // Light purple
}

void draw_body(OrbitalTreeNode* node, Camera3D camera, DVector3 center) {
    Vector3 position = TF(vector_from_physical_to_world(DVector3Add(center, node->physical_state.r)));
    DrawSphereWires(position,node->physical_params.radius * KM_TO_RENDER_UNITS,10,10,node->body_color);
}

void draw_body_labels(OrbitalTreeNode* node, Camera3D camera, DVector3 center) {
    if (customFont.texture.id == 0) return;
    
    Vector3 body_pos_world = TF(vector_from_physical_to_world(DVector3Add(center, node->physical_state.r)));
    Vector2 body = GetWorldToScreen(body_pos_world, camera);
    
    // Create velocity string
    DVector3 velocity = node->physical_state.v;
    char velocity_str[64];
    snprintf(velocity_str, sizeof(velocity_str), "V: %.2f km/s", DVector3Length(velocity));
    
    if (!is_object_behind_camera(camera.position, camera.target, body_pos_world)) {
        // Measure text sizes
        Vector2 bodyLabelSize = MeasureTextEx(customFont, node->body_name, 18, 0);
        Vector2 velocityLabelSize = MeasureTextEx(customFont, velocity_str, 16, 0);

        // Layout spacing
        float labelSpacing = 6;
        float verticalOffset = 25;

        // Compute label positions
        Vector2 bodyLabelPos = {
            body.x - bodyLabelSize.x / 2.0f,
            body.y - bodyLabelSize.y - verticalOffset
        };

        Vector2 velocityLabelPos = {
            body.x - velocityLabelSize.x / 2.0f,
            bodyLabelPos.y - velocityLabelSize.y - labelSpacing
        };

        // Velocity label with clean background - no borders
        Rectangle vel_bg = {velocityLabelPos.x - 6, velocityLabelPos.y - 3, velocityLabelSize.x + 12, velocityLabelSize.y + 6};
        DrawRectangleRec(vel_bg, (Color){0, 0, 0, 200});
        DrawTextEx(customFont, velocity_str, velocityLabelPos, 16, 0, LIME);
        
        // Body name label with clean background - no borders
        Rectangle body_bg = {bodyLabelPos.x - 6, bodyLabelPos.y - 3, bodyLabelSize.x + 12, bodyLabelSize.y + 6};
        DrawRectangleRec(body_bg, (Color){0, 0, 0, 200});
        DrawTextEx(customFont, node->body_name, bodyLabelPos, 18, 0, WHITE);
    }
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
        draw_sphere_of_influence(node,camera3d,center);
    }
    
    // Draw the 3D body sphere only
    Vector3 position = TF(vector_from_physical_to_world(DVector3Add(center, node->physical_state.r)));
    DrawSphereWires(position,node->physical_params.radius * KM_TO_RENDER_UNITS,10,10,node->body_color);
    EndMode3D();
    
    // Draw 2D labels OUTSIDE 3D context
    draw_body_labels(node, camera3d, center);

    // Iterate over children & recurse!
    for (int i = 0; i < darray_length(node->children); i++) {
        OrbitalTreeNode** item = (OrbitalTreeNode**)darray_get(node->children,i);
        draw_orbital_tree_recursive(root, *item, clock, camera3d, should_draw_orbital_features,proj);
    }
}

// Global variables for the game state
static ProgramState* program_state = NULL;
static Font customFont;
static Camera camera;
static float r = 100000.0;
static float theta = 0.0;
static float phi = PI;
static bool is_click_to_drag_on = true;
static Matrix matProj;
static bool should_draw_orbital_features = true;
static bool game_initialized = false;

// Game loop callback function for Emscripten
void game_loop(void) {
    // Basic safety checks
    if (!game_initialized || !program_state) {
        return;
    }
    
    // Safety check for font
    if (customFont.texture.id == 0) {
        return;
    }
    
    // Safety check for focused node
    if (!program_state->focused_node) {
        return;
    }
    
    // Try to run the game loop, but catch any errors
    // This is a simple error handling approach for web

    float time = GetTime();
    float delta = GetFrameTime();
    
    

    
    // Handle input
    if(IsKeyPressed(KEY_U)) {
        if (is_click_to_drag_on) {
            EnableCursor();
        } else {
            DisableCursor();
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

    // Safety checks for tree operations
    if (program_state->tree) {
        restructure_orbital_tree_recursive(program_state->tree,program_state->tree);
        update_orbital_tree_recursive(program_state->tree,program_state->tree,program_state->clock);
    }
    
    darray list = darray_init(10,sizeof(OrbitalTreeNode**));
    if (program_state->tree) {
        darray bodies = dfs_orbital_tree_nodes(program_state->tree, list);

        for (int i = 0; darray_length(bodies) > 0 && i < darray_length(bodies);i++) {
            OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(bodies,i);
        }
    }

    if(IsKeyPressed(KEY_V)) {
        should_draw_orbital_features = !should_draw_orbital_features;
    }

    // Draw
    BeginDrawing();
        if (program_state != NULL && program_state->focused_node != NULL && customFont.texture.id != 0) {
            char buffer[256];
            snprintf(buffer, sizeof(buffer), "Focused: %s", program_state->focused_node->body_name);
            
            // Draw background for focused node display
            int text_width = MeasureTextEx(customFont, buffer, 22, 0).x;
            Rectangle bg_rect = {
                GetScreenWidth()/2 - text_width/2 - 20, 
                50, 
                text_width + 40, 
                35
            };
            DrawRectangleRec(bg_rect, (Color){10, 20, 40, 240}); // Darker blue-black
            Rectangle focus_highlight = {bg_rect.x, bg_rect.y, bg_rect.width, 3};
            DrawRectangleRec(focus_highlight, (Color){120, 200, 255, 150});
            DrawRectangleLinesEx(bg_rect, 3, (Color){120, 200, 255, 255}); // Brighter blue border
            
            // Draw the text
            Vector2 pos = {(float)(GetScreenWidth()/2 - text_width/2), 57};
            DrawTextEx(customFont, buffer, pos, 22, 0, (Color){120, 200, 255, 255}); // Light blue
        }

        ClearBackground(BLACK);

        // Safety checks for drawing operations
        if (program_state->tree && program_state->focused_node) {
            PhysicalState focus_rv = global_physical_state(program_state->tree, program_state->focused_node);
            DVector3 world_r = vector_from_physical_to_world(focus_rv.r);
            camera.target = TF(world_r);
            spherical_camera_system(world_r, &r, &theta, &phi, &camera,is_click_to_drag_on);

            draw_orbital_tree_recursive(program_state->tree,program_state->tree,program_state->clock,camera,should_draw_orbital_features,matProj);
        }
        BeginMode3D(camera);
            rlSetMatrixProjection(matProj);

            if (should_draw_orbital_features) {
                // Safety check for 3D drawing
                DVector3 x_axis = vector_from_physical_to_world((DVector3){EARTH_RADIUS_KM*10,0,0});
                DVector3 y_axis = vector_from_physical_to_world((DVector3){0,EARTH_RADIUS_KM*10,0});
                DVector3 z_axis = vector_from_physical_to_world((DVector3){0,0,EARTH_RADIUS_KM*10});
                
                DrawLine3D((Vector3){0,0,0}, TF(x_axis), RED);
                DrawLine3D((Vector3){0,0,0}, TF(y_axis), GREEN);
                DrawLine3D((Vector3){0,0,0}, TF(z_axis), BLUE);
            }
        EndMode3D();

        // Draw orbital hierarchy and elements after 3D rendering
        if (program_state->tree && program_state->focused_node) {
            draw_orbital_hierarchy(program_state->focused_node, program_state->tree, 0, 0);
            if (program_state->focused_node->parent != NULL) {
                int orbital_lines_count = 0;
                if (program_state->focused_node->orbital_lines != NULL) {
                    orbital_lines_count = darray_length(program_state->focused_node->orbital_lines);
                }
                draw_textual_orbital_elements(program_state->focused_node->orbital_elements, orbital_lines_count, program_state->focused_node->body_name);
            } else {

            }
        }

        // Draw clock info
        if (program_state && program_state->clock) {
            draw_clock_info(program_state->clock);
        }

        // Draw legend
        draw_legend();
        
        // Draw controls section
        draw_controls();

    EndDrawing();
}

// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    // Initialize Logger
    InitializeLogger(LOG_LEVEL,true);

    // We need to free this, it was alloc'ed
    program_state = load_program_state();

    //--------------------------------------------------------------------------------------
    InitWindow(2560, 1440, "C Orbit Web");

    // Load the font with fallback to default
    printf("INFO: Attempting to load custom font: resources/FiraCode-Bold.ttf\n");
    // Load font with basic ASCII character set (32-126)
    int codepointsCount = 95; // ASCII printable characters
    int *codepoints = (int*)malloc(codepointsCount * sizeof(int));
    for (int i = 0; i < codepointsCount; i++) {
        codepoints[i] = 32 + i; // ASCII range 32-126
    }
    customFont = LoadFontEx("resources/FiraCode-Bold.ttf", 48, codepoints, codepointsCount);
    free(codepoints);
    
    // If custom font fails, use default font
    if (customFont.texture.id == 0) {
        printf("WARNING: Custom font failed to load, using default font\n");
        customFont = GetFontDefault();
        // Scale up the default font to match our design
        customFont.baseSize = 48;
    } else {
        printf("INFO: Custom font loaded successfully\n");
    }

    // Wait for font texture to be ready and apply filtering
    if (customFont.texture.id != 0) {
        SetTextureFilter(customFont.texture, TEXTURE_FILTER_BILINEAR);
    }

    // Define the camera to look into our 3d world (position, target, up vector)
    camera.position = (Vector3){ 0.0f, 2.0f, 4.0f };    // Camera position
    camera.target = (Vector3){ 0.0f, 2.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 60.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    matProj = MatrixPerspective(camera.fovy*DEG2RAD, ((double)GetScreenWidth()/(double)GetScreenHeight()), 0.1,10000000.0);

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second

    // Mark game as initialized
    game_initialized = true;

    // Set up the game loop callback for Emscripten
    emscripten_set_main_loop(game_loop, 0, 1);

    return 0;
}
 
