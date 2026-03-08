#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

bool CANInterface::s_ok = false;

// MCP2515 instance: CS pin, clock speed passed to constructor
static MCP2515 s_mcp(PIN_CAN_CS);

// ─── CAN ID map — fill these in once the arm protocol is documented ───────────
// Example layout: one frame per joint, 2-byte signed angle (×100 deg) in bytes 0-1
// static constexpr uint32_t CAN_ID_JOINT_BASE = 0x100;   // 0x100..0x105 for joints 1-6

bool CANInterface::begin() {
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);

    MCP2515::ERROR err = s_mcp.reset();
    if (err != MCP2515::ERROR_OK) { s_ok = false; return false; }

    // Set CAN bitrate — CAN_CLOCK_MHZ must match oscillator on module
    CAN_SPEED speed;
    switch (CAN_BITRATE_KBPS) {
        case 125:  speed = CAN_125KBPS; break;
        case 250:  speed = CAN_250KBPS; break;
        case 500:  speed = CAN_500KBPS; break;
        case 1000: speed = CAN_1000KBPS; break;
        default:   speed = CAN_500KBPS; break;
    }
    CAN_CLOCK clk = (CAN_CLOCK_MHZ == 16) ? MCP_16MHZ : MCP_8MHZ;

    err = s_mcp.setBitrate(speed, clk);
    if (err != MCP2515::ERROR_OK) { s_ok = false; return false; }

    s_mcp.setNormalMode();
    s_ok = true;
    return true;
}

bool CANInterface::sendArmJoints(const float angles_deg[6]) {
    if (!s_ok) return false;

    // ── TODO: define arm CAN protocol ──────────────────────────────────────────
    // Options (common patterns):
    //   A) One frame per joint: ID = CAN_ID_JOINT_BASE + joint_index
    //      Data: int16_t angle_deg×100 in bytes [0:1], plus control flags
    //   B) Broadcast frame: all 6 joints packed as int16_t (12 bytes > 8 → need 2 frames)
    //   C) Proprietary protocol from arm manufacturer
    //
    // Placeholder: transmit joints 0-3 in frame A and joints 4-5 in frame B
    // as int16_t (×100).  Replace with real protocol once specified.

    struct can_frame frame;
    frame.can_dlc = 8;

    // Frame A: joints 0-3
    frame.can_id = 0x200;
    for (int j = 0; j < 4; j++) {
        int16_t v = static_cast<int16_t>(angles_deg[j] * 100.0f);
        frame.data[j * 2]     = (v >> 8) & 0xFF;
        frame.data[j * 2 + 1] = v & 0xFF;
    }
    if (s_mcp.sendMessage(&frame) != MCP2515::ERROR_OK) return false;

    // Frame B: joints 4-5
    frame.can_id  = 0x201;
    frame.can_dlc = 4;
    for (int j = 0; j < 2; j++) {
        int16_t v = static_cast<int16_t>(angles_deg[4 + j] * 100.0f);
        frame.data[j * 2]     = (v >> 8) & 0xFF;
        frame.data[j * 2 + 1] = v & 0xFF;
    }
    return s_mcp.sendMessage(&frame) == MCP2515::ERROR_OK;
}

void CANInterface::poll() {
    if (!s_ok) return;

    struct can_frame frame;
    while (s_mcp.readMessage(&frame) == MCP2515::ERROR_OK) {
        // TODO: handle incoming arm status / fault frames
        // Example:
        // if (frame.can_id == CAN_ID_ARM_STATUS) { parseArmStatus(frame); }
        (void)frame;
    }
}

bool CANInterface::isOk() {
    return s_ok;
}