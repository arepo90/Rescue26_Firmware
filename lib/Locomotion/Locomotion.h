#pragma once
#include <stdint.h>

// ─── Locomotion ───────────────────────────────────────────────────────────────
// Provides high-level movement commands for the traction and flipper systems.
// The public API is robot-agnostic; hardware-specific output (LEDC PWM for
// ROBOT_MAIN, or future CAN motor controllers for ROBOT_SECONDARY) is
// encapsulated inside the private apply* methods.
//
// All normalised values are in the range [-1.0, +1.0] unless otherwise noted.

class Locomotion {
public:
    static void begin();

    // High-level commands (platform-independent) ──────────────────────────────

    // Tank-drive mixing: forward ∈ [-1,1], turn ∈ [-1,1]
    // Computes left/right track speeds and routes to applyTrackSpeeds().
    static void setDriveCommand(float forward, float turn);

    // Set both tracks directly (e.g. from a speed controller).
    static void setTrackSpeeds(float left_norm, float right_norm);

    // Command flipper to a target angle (degrees).
    // Clamps to [FLIPPER_ANGLE_MIN, FLIPPER_ANGLE_MAX].
    static void setFlipperTarget(float angle_deg);

    // Immediately neutralise all outputs (tracks + flipper → stopped).
    static void neutralise();

private:
    // Low-level output (platform-specific) ───────────────────────────────────

    // Write normalised [-1,1] speed to left and right track actuators.
    static void applyTrackSpeeds(float left_norm, float right_norm);

    // Write normalised [-1,1] command to flipper actuator.
    static void applyFlipperPWM(float norm);

    // Convert normalised [-1,1] → PWM duty count for LEDC peripheral.
    static uint32_t normToDuty(float norm);

    static float  s_flipper_target_deg;
};
