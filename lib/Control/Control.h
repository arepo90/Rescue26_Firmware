#pragma once
#include "robot_types.h"

// ─── Control ──────────────────────────────────────────────────────────────────
// Top-level state machine.  Runs on Core 1 at PRIO_CONTROL.
//
// State transitions:
//
//  INIT ──(hw ready)──► STANDBY ──(PPM link)──► MAIN
//                                                  │  ▲
//                                          Ch5 hi  │  │ Ch5 lo
//                                                  ▼  │
//                                                  ARM
//
//  Any state ──(ESTOP from mini PC)──► ESTOP ──(ESTOP_CLEAR)──► STANDBY
//
// The control loop runs at CONTROL_LOOP_HZ.  In ARM mode, joint commands
// from the mini PC are forwarded to CANInterface; the ESP32 never tries to
// solve IK locally.

class Control {
public:
    // One-time setup: register callbacks and store latest state.
    static void begin();

    // Main loop body — call from the control FreeRTOS task.
    static void tick();

    // ── Setters called from other tasks / callbacks ───────────────────────────
    static void triggerEstop();
    static void clearEstop();
    static void setArmJoints(const ArmJointsPayload& payload);
    static void setSensorMask(uint8_t mask);

    static RobotMode getMode();
    static void      getSystemStatus(SystemStatus& out);

private:
    // Per-mode update functions
    static void updateMainMode(const PPMFrame& ppm, const EncoderState& enc);
    static void updateArmMode(const PPMFrame& ppm);

    // Determine mode from Ch5 pulse width
    static bool isArmModeRequested(const PPMFrame& ppm);

    static RobotMode  s_mode;
    static ArmJoints  s_arm_joints;
    static uint8_t    s_sensor_mask;
    // s_mux lives as a file-level static in Control.cpp (avoids FreeRTOS header in .h)
};
