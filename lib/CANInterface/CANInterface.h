#pragma once
#include <stdint.h>
#include <stdbool.h>

// ─── CANInterface ─────────────────────────────────────────────────────────────
// Wraps SPI-to-CAN communication (MCP2515 assumed).
//
// ┌─ STUB NOTICE ───────────────────────────────────────────────────────────────┐
// │ Arbitration IDs, data layouts, and command protocols are all TODO.          │
// │ Function signatures define the stable interface; flesh out the bodies in    │
// │ CANInterface.cpp once those details are confirmed.                           │
// └─────────────────────────────────────────────────────────────────────────────┘
//
// Robot-type usage:
//   ROBOT_MAIN      — sendArmJoints() relays mini-PC IK results to the arm ESP32.
//   ROBOT_SECONDARY — sendArmJoints(), sendTrackSpeeds(), sendFlipperSpeeds()
//                     drive all actuators directly from this ESP32.

class CANInterface {
public:
    // Initialise MCP2515.  Returns false on SPI/comms failure.
    static bool begin();

    // Send 6 joint angle targets to the arm over CAN.
    // angles_deg[0..5] → joint 1..6, degrees.
    // Returns false if the TX buffer is busy.
    // ROBOT_MAIN: relays angles received from the mini PC to the dedicated arm ESP32.
    // ROBOT_SECONDARY: drives the arm controllers directly.
    // TODO: confirm arbitration IDs and data layout.
    static bool sendArmJoints(const float angles_deg[6]);

    // ── ROBOT_SECONDARY traction + flipper ────────────────────────────────────

    // Send normalised [-1,1] speeds to the left and right track CAN controllers.
    // TODO: confirm arbitration IDs and data layout.
    static bool sendTrackSpeeds(float left_norm, float right_norm);

    // Send normalised [-1,1] speed to each of the four independent flipper CAN
    // controllers.  Order: front-left, front-right, rear-left, rear-right.
    // TODO: confirm arbitration IDs and data layout.
    static bool sendFlipperSpeeds(float fl, float fr, float rl, float rr);

    // Poll for any incoming CAN frames (e.g. arm status / fault flags).
    // Should be called periodically from the CAN task.
    static void poll();

    // True if the last begin() succeeded and no bus-off error has occurred.
    static bool isOk();

private:
    static bool s_ok;
};