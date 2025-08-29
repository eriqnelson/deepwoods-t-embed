#pragma once
#include <Arduino.h>

namespace deepwoods {

// ---- Lifecycle ----
void startBaseline();   // kicks off 5-minute baseline + starts tasks
void resetBaseline();   // clears sets and restarts baseline window
void stop();            // stops tasks, turns off promiscuous/BLE
void showStatus();      // prints a one-shot status line to Serial

// ---- Telemetry ----
struct Counts {
  uint16_t ble;
  uint16_t wifi;
  uint16_t probe;
  bool     baseline_active;
  uint32_t baseline_ms_remaining;
};
Counts getCounts();

// ---- Compile-time maximums (exposed for UI/tests) ----
enum {
  MAX_BLE   = 200,
  MAX_WIFI  = 200,
  MAX_PROBE = 500
};

} // namespace deepwoods