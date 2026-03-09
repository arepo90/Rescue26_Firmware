#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

bool CANInterface::s_ok = false;

// MCP2515 instance: CS pin, clock speed passed to constructor
static MCP2515 s_mcp(PIN_CAN_CS);

// ─── VESC CAN helpers ─────────────────────────────────────────────────────────
// Standard VESC extended-frame protocol (matches VESC firmware source).
// Extended ID = (command << 8) | controller_id, always sent with CAN_EFF_FLAG.

static constexpr uint8_t VESC_CMD_SET_CURRENT = 1;

// Build the VESC extended CAN ID for a given controller and command.
static inline uint32_t vescEID(uint8_t ctrl_id, uint8_t cmd) {
    return ((uint32_t)cmd << 8) | ctrl_id;
}

// Pack a 32-bit signed integer as 4 big-endian bytes into a CAN data buffer.
static inline void putInt32BE(uint8_t* d, int32_t v) {
    d[0] = (v >> 24) & 0xFF;
    d[1] = (v >> 16) & 0xFF;
    d[2] = (v >>  8) & 0xFF;
    d[3] = (v >>  0) & 0xFF;
}

// Send a SET_CURRENT command to one VESC.
// current_a: desired current in amperes (positive = forward, negative = reverse).
// Returns true if the MCP2515 accepted the frame.
static bool vescSendCurrent(MCP2515& mcp, uint8_t ctrl_id, float current_a) {
    struct can_frame tx;
    tx.can_id  = vescEID(ctrl_id, VESC_CMD_SET_CURRENT) | CAN_EFF_FLAG;
    tx.can_dlc = 4;
    putInt32BE(tx.data, static_cast<int32_t>(current_a * 1000.0f));  // A → mA
    return mcp.sendMessage(&tx) == MCP2515::ERROR_OK;
}

// ─── Arm CAN ID map — fill in once arm protocol is documented ────────────────
// static constexpr uint32_t CAN_ID_JOINT_BASE = 0x100;

bool CANInterface::begin() {
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);

    MCP2515::ERROR err = s_mcp.reset();
    if (err != MCP2515::ERROR_OK) { s_ok = false; return false; }

    // 500 kbps, 8 MHz oscillator
    err = s_mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
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

bool CANInterface::sendTrackSpeeds(float left_norm, float right_norm) {
    if (!s_ok) return false;
#ifdef ROBOT_SECONDARY
    // Scale normalised [-1,1] to current in amps and send to each track VESC.
    float left_a  = left_norm  * VESC_TRACK_I_MAX_A;
    float right_a = right_norm * VESC_TRACK_I_MAX_A;
    bool ok = vescSendCurrent(s_mcp, VESC_ID_TRACK_LEFT,  left_a);
    ok     &= vescSendCurrent(s_mcp, VESC_ID_TRACK_RIGHT, right_a);
    return ok;
#else
    (void)left_norm; (void)right_norm;
    return false;
#endif
}

bool CANInterface::sendFlipperSpeeds(float fl, float fr, float rl, float rr) {
    if (!s_ok) return false;
#ifdef ROBOT_SECONDARY
    // One SET_CURRENT frame per flipper VESC.
    bool ok = vescSendCurrent(s_mcp, VESC_ID_FLIPPER_FL, fl * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(s_mcp, VESC_ID_FLIPPER_FR, fr * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(s_mcp, VESC_ID_FLIPPER_RL, rl * VESC_FLIPPER_I_MAX_A);
    ok     &= vescSendCurrent(s_mcp, VESC_ID_FLIPPER_RR, rr * VESC_FLIPPER_I_MAX_A);
    return ok;
#else
    (void)fl; (void)fr; (void)rl; (void)rr;
    return false;
#endif
}

bool CANInterface::isOk() {
    return s_ok;
}