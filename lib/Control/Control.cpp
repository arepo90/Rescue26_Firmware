#include "Control.h"
#include "config.h"
#include "RC.h"
#include "Encoders.h"
#include "Locomotion.h"
#include "Sensors.h"
#include "CANInterface.h"
#include "Comms.h"
#include <Arduino.h>

// ─── Static members ───────────────────────────────────────────────────────────
RobotMode    Control::s_mode        = RobotMode::INIT;
ArmJoints    Control::s_arm_joints  = {};
uint8_t      Control::s_sensor_mask = 0;

// File-level mutex: keeps FreeRTOS types out of the class header
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ─── begin() — called once from setup() ──────────────────────────────────────
void Control::begin() {
    // Wire up callbacks from Comms to Control setters
    Comms::onArmJoints(   [](const ArmJointsPayload& p) { Control::setArmJoints(p); });
    Comms::onSensorEnable([](uint8_t mask)              { Control::setSensorMask(mask); });
    Comms::onEstop(       [](bool active)               {
        if (active) Control::triggerEstop();
        else        Control::clearEstop();
    });

    s_mode = RobotMode::STANDBY;
}

// ─── tick() — called from control task at CONTROL_LOOP_HZ ────────────────────
void Control::tick() {
    PPMFrame     ppm;
    EncoderState enc;
    bool         have_ppm = RC::getFrame(ppm);
    Encoders::getState(enc);

    // ── ESTOP: override everything ────────────────────────────────────────────
    portENTER_CRITICAL(&s_mux);
    RobotMode current = s_mode;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        Locomotion::neutralise();
        return;
    }

    // ── PPM link watchdog: go to STANDBY if signal lost ──────────────────────
    if (!RC::isConnected()) {
        Locomotion::neutralise();
        portENTER_CRITICAL(&s_mux);
        s_mode = RobotMode::STANDBY;
        portEXIT_CRITICAL(&s_mux);
        return;
    }

    // ── Mode transition from Ch5 ──────────────────────────────────────────────
    if (have_ppm) {
        bool arm_requested = isArmModeRequested(ppm);
        portENTER_CRITICAL(&s_mux);
        if (arm_requested && s_mode == RobotMode::MAIN) {
            s_mode = RobotMode::ARM;
        } else if (!arm_requested && s_mode == RobotMode::ARM) {
            s_mode = RobotMode::MAIN;
        } else if (s_mode == RobotMode::STANDBY) {
            s_mode = arm_requested ? RobotMode::ARM : RobotMode::MAIN;
        }
        current = s_mode;
        portEXIT_CRITICAL(&s_mux);
    }

    // ── Per-mode update ───────────────────────────────────────────────────────
    if (have_ppm) {
        if (current == RobotMode::MAIN) {
            updateMainMode(ppm, enc);
        } else if (current == RobotMode::ARM) {
            updateArmMode(ppm);
        }
    }

    // ── Periodic telemetry to mini PC ─────────────────────────────────────────
    // (Triggered by the comms task reading the shared state; nothing to do here.)

    // ── Sensor data forwarding (when enabled) ────────────────────────────────
    // Sent from the sensor task after each successful reading cycle.
}

// ─── MAIN mode: RC drives tracks + flipper ───────────────────────────────────
void Control::updateMainMode(const PPMFrame& ppm, const EncoderState& /*enc*/) {
    // Normalise driving channels to [-1, +1]
    float forward = ppmNormalise(ppm.ch[PPM_CH_FORWARD - 1]);
    float turn    = ppmNormalise(ppm.ch[PPM_CH_TURN    - 1]);

    // Apply a small deadband to prevent drift at stick centre
    constexpr float kDeadband = 0.05f;
    if (fabsf(forward) < kDeadband) forward = 0.0f;
    if (fabsf(turn)    < kDeadband) turn    = 0.0f;

    Locomotion::setDriveCommand(forward, turn);

    // Flipper: map channel 1 to target angle
    float flipper_norm  = ppmNormalise(ppm.ch[PPM_CH_FLIPPER - 1]);
    float flipper_angle = flipper_norm * FLIPPER_ANGLE_MAX;   // simple proportional mapping
    Locomotion::setFlipperTarget(flipper_angle);
}

// ─── ARM mode: RC channels → end-effector target → mini PC for IK ────────────
void Control::updateArmMode(const PPMFrame& ppm) {
    // Halt tracks while the arm is active
    Locomotion::setDriveCommand(0.0f, 0.0f);

    // Build end-effector deltas from RC sticks.
    // Mapping (adjust to taste):
    //   Ch2 → Δy (forward/back in robot frame)
    //   Ch4 → Δx (left/right)
    //   Ch1 → Δz (up/down via flipper channel repurposed for arm)
    //   Ch3 → Δyaw  (if available)
    //   Ch6 → Δpitch
    //
    // We send the raw normalised channel values so the mini PC can integrate
    // or use them directly in its Cartesian IK.
    TelemetryPayload telem;
    telem.mode  = static_cast<uint8_t>(RobotMode::ARM);
    telem.flags = 0;
    for (int i = 0; i < PPM_CHANNELS; i++) telem.ppm[i] = ppm.ch[i];
    // Encoder fields left at 0 — the comms task populates them from shared state.
    Comms::sendTelemetry(telem);

    // Apply the latest arm joint angles received from mini PC via CAN
    portENTER_CRITICAL(&s_mux);
    ArmJoints joints = s_arm_joints;
    portEXIT_CRITICAL(&s_mux);

    if (joints.valid) {
        CANInterface::sendArmJoints(joints.angle_deg);
    }
}

// ─── Callbacks ────────────────────────────────────────────────────────────────
void Control::triggerEstop() {
    portENTER_CRITICAL(&s_mux);
    s_mode = RobotMode::ESTOP;
    portEXIT_CRITICAL(&s_mux);
    Locomotion::neutralise();
}

void Control::clearEstop() {
    portENTER_CRITICAL(&s_mux);
    if (s_mode == RobotMode::ESTOP) s_mode = RobotMode::STANDBY;
    portEXIT_CRITICAL(&s_mux);
}

void Control::setArmJoints(const ArmJointsPayload& payload) {
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 6; i++) {
        s_arm_joints.angle_deg[i] = payload.joint[i] * 0.01f;   // ×100 → degrees
    }
    s_arm_joints.valid = true;
    portEXIT_CRITICAL(&s_mux);
}

void Control::setSensorMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_sensor_mask = mask;
    portEXIT_CRITICAL(&s_mux);
    Sensors::setEnabledMask(mask);
}

// ─── Accessors ────────────────────────────────────────────────────────────────
RobotMode Control::getMode() {
    portENTER_CRITICAL(&s_mux);
    RobotMode m = s_mode;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

void Control::getSystemStatus(SystemStatus& out) {
    portENTER_CRITICAL(&s_mux);
    out.mode         = s_mode;
    out.sensor_mask  = s_sensor_mask;
    out.estop        = (s_mode == RobotMode::ESTOP);
    portEXIT_CRITICAL(&s_mux);
    out.ppm_connected   = RC::isConnected();
    out.minipc_connected = Comms::isConnected();
    out.can_ok           = CANInterface::isOk();
    out.uptime_ms        = millis();
}

// ─── Helper ───────────────────────────────────────────────────────────────────
bool Control::isArmModeRequested(const PPMFrame& ppm) {
    // Ch5 > midpoint → arm mode.  Typical switch: 1000 µs = off, 2000 µs = on.
    return ppm.ch[PPM_CH_MODE - 1] > ((PPM_MIN_US + PPM_MAX_US) / 2);
}
