#pragma once

typedef enum PhysicsTimeMode {
    Elapsing,
    StopTick,
} PhysicsTimeMode;

typedef struct PhysicsTimeClock {
    enum PhysicsTimeMode mode;
    double tick_interval_seconds;
    double delta_seconds;
    double clock_seconds;
    double scale;
} PhysicsTimeClock;

// Updates the physics clock based on the delta seconds that have elapsed since the last update
void UpdatePhysicsClock(PhysicsTimeClock* clock, float delta_seconds);
