#include "time.h"
#include "raylib.h"

void UpdatePhysicsClockElapsing(PhysicsTimeClock *clock, float raylib_delta_seconds) {
    if (clock->mode == StopTick) {
        return;
    }

    clock->delta_seconds = clock->scale * raylib_delta_seconds;
    clock->clock_seconds += clock->delta_seconds;
}

void UpdatePhysicsClockStopTick(PhysicsTimeClock *clock, float raylib_delta_seconds) {
    // TODO: come back and fix this and integrate stop tick updates here with keyboard inputs
    // Nothing to do update if there is stop tick
    if (clock->mode == Elapsing) {
        return;
    }

    // Jump forward/back clock->tick_interval_seconds
    if(IsKeyDown(KEY_RIGHT)) {
        clock->clock_seconds += clock->tick_interval_seconds;
    } else if (IsKeyDown(KEY_LEFT)) {
        clock->clock_seconds -= clock->tick_interval_seconds;
    }
}

void UpdatePhysicsClock(PhysicsTimeClock *clock, float raylib_delta_seconds) {
    if (clock->mode == Elapsing) {
        UpdatePhysicsClockElapsing(clock, raylib_delta_seconds);
    } else if (clock->mode == StopTick) { // TODO implement me
        UpdatePhysicsClockStopTick(clock, raylib_delta_seconds);
    }
}
