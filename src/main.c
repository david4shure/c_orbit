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

#define MAX_COLUMNS 20

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void) {
    /* Vector3 r = { 0, 1, 1 }; */
    /* Vector3 v = { 1, 1, 0 }; */
    /* OrbitalState state = {r,v}; */
    /* OrbitalElements eles = orb_elems_from_rv(state, 0.55243); */

/*     // Initialization */
    Vector3 R = {-6045, -3490, 2500};
    Vector3 V = {-3.457, 6.618, 2.533};


    OrbitalState state = {R,V};
    OrbitalElements eles = orb_elems_from_rv(state, 0.55243);
    PhysicsTimeClock clock = { .tick_interval_seconds = 86400, .mode = Elapsing, .scale = 500.0, .delta_seconds = 0.0 };

    float M_naught = 2.35585;
    float t_naught = 0.0;
    float T = 2360591.0;
    printf("a = %.2f\n",eles.a);
    printf("e = %.2f\n",eles.e);
    printf("E = %.2f\n",eles.E);
    float eta = 1e-10;

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

    Model model = LoadModelFromMesh(GenMeshSphere(25.0, 80, 80));

    Texture texture = LoadTexture("resources/2k_earth_daymap.png");

    SetTextureWrap(texture, TEXTURE_WRAP_CLAMP);

    // Assign texture to default model material
    model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = SKYBLUE;

    DisableCursor();                    // Limit cursor to relative movement inside the window

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    float far = 1000.0f;

    // Field of View (in radians), aspect ratio, near and far planes
    Matrix projMatrix = MatrixPerspective(70.0f*DEG2RAD, (float)screenWidth/(float)screenHeight, 0.1f, far);
    rlSetMatrixProjection(projMatrix);

    float prev_far = far;

    // Main game loop
    while (!WindowShouldClose())        // Detect window close button or ESC key
    {
        time = GetTime();

        UpdatePhysicsClock(&clock, (float)time);

        float M = mean_anom(M_naught,clock.clock_seconds, t_naught,T);

        if (M > 2 * PI) {
            M = M - 2 * PI;
        }

        float e = 0.5;

        float E = kepler_E_newt(e, M, 50);
        float TA = ecc_anom_to_true_anom(e, E);
        float dist = distance(e,eles.a,E);

        float x = dist * cos(TA);
        float y = dist * sin(TA);


        printf("M = %.2f, E = %.2f, TA = %2f\n",M,E,TA);

        // Update camera computes movement internally depending on the camera mode
        // Some default standard keyboard/mouse inputs are hardcoded to simplify use
        // For advance camera controls, it's reecommended to compute camera movement manually
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // theta = clampf(theta,0+eta, 2*PI-eta);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            BeginMode3D(camera);

                Vector3 sphere_pos = {0.0,0.0,0.0};

                // sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
                float dist = Vector3Distance(sphere_pos, camera.position);

                float lod = 1.0;

                if (IsKeyDown(KEY_F1)) {
                    prev_far = far;
                    far += 50;
                }

                if (IsKeyDown(KEY_F2)) {
                    prev_far = far;
                    far -= 50;
                }

                rlSetMatrixProjection(MatrixPerspective(70.0f*DEG2RAD, (float)screenWidth/(float)screenHeight, 0.1f, far));

                if (far != prev_far) {
                    printf("changed far = %.2f\n",far);
                    if (far > 10.0) {
                        prev_far = far;
                    }
                }

                SphericalCameraSystem(&r, &theta, &phi, &camera);

                DrawModel(model,sphere_pos,1.0,SKYBLUE);
                DrawGridOfColor(250,500.0,GREEN); // Draw ground

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
