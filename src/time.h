#pragma once

typedef enum PhysicsTimeMode {
    Elapsing,
    StopTick,
} PhysicsTimeMode;

typedef struct PhysicsTimeClock {
    enum PhysicsTimeMode mode;
    float tick_interval_seconds;
    float delta_seconds;
    float clock_seconds;
    float scale;
} PhysicsTimeClock;

// Updates the physics clock based on the delta seconds that have elapsed since the last update
void UpdatePhysicsClock(PhysicsTimeClock* clock, float delta_seconds);
