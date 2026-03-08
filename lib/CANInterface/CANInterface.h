#pragma once
#include <stdint.h>
#include <stdbool.h>

// ─── CANInterface ─────────────────────────────────────────────────────────────
// Wraps SPI-to-CAN communication (MCP2515 assumed) for robotic arm control.
//
// ┌─ STUB NOTICE ───────────────────────────────────────────────────────────────┐
// │ The exact CAN arbitration IDs, data layout, and command protocol depend on  │
// │ the specific arm controller.  The function signatures below define a stable  │
// │ interface; flesh out the bodies in CANInterface.cpp once those details are  │
// │ confirmed.                                                                   │
// └─────────────────────────────────────────────────────────────────────────────┘
//
// Call begin() once from setup; then sendArmJoints() from the CAN task whenever
// new joint targets arrive from the mini PC.

class CANInterface {
public:
    // Initialise MCP2515.  Returns false on SPI/comms failure.
    static bool begin();

    // Send 6 joint angle targets to the arm over CAN.
    // angles_deg[0..5] → joint 1..6, degrees.
    // Returns false if the TX buffer is busy.
    static bool sendArmJoints(const float angles_deg[6]);

    // Poll for any incoming CAN frames (e.g. arm status / fault flags).
    // Should be called periodically from the CAN task.
    static void poll();

    // True if the last begin() succeeded and no bus-off error has occurred.
    static bool isOk();

private:
    static bool s_ok;
};