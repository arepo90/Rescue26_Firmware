#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── RC ───────────────────────────────────────────────────────────────────────
// Decodes a standard RC PPM stream using a GPIO interrupt on rising edges.
//
// Timing model (positive PPM, Flysky FS-i6 default):
//   Rising edges are spaced by: [separator_LOW ~300µs] + [channel_HIGH 1000–2000µs]
//   A rising-to-rising interval > PPM_SYNC_US marks the frame boundary.
//   The first PPM_CHANNELS intervals after the sync populate channels[0..5].
//
// Usage:
//   RC::begin(PIN_PPM);
//   RC::getFrame(frame);   // non-blocking; returns false if no new frame

class RC {
public:
    static void begin(uint8_t pin);

    // Returns true if a new, complete frame is available since last call.
    // 'out' is populated with the latest frame's values.
    static bool getFrame(PPMFrame& out);

    // True if a valid frame has arrived within PPM_TIMEOUT_MS.
    static bool isConnected();

private:
    static void isr();   // IRAM_ATTR applied on definition in .cpp
};
