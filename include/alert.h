#pragma once
#include <Arduino.h>

// Initialize any alert GPIO/PWM; safe to call multiple times.
void alert_init();

// Single “event” alert: fast LED flash + short tone (if enabled)
void alert_pulse();

// Optional: verbose helper that also logs a line to Serial
inline void alert_trip(const char* kind, const char* mac) {
  if (kind && mac) Serial.printf("Detected non-baseline %s: %s\n", kind, mac);
  alert_pulse();
}