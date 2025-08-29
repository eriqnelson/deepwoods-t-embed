#pragma once

// -------- Timings --------
#ifndef DEEPWOODS_BASELINE_MS
#define DEEPWOODS_BASELINE_MS      (5UL * 60UL * 1000UL)  // 5 minutes
#endif

#ifndef DEEPWOODS_SCAN_INTERVAL_MS
#define DEEPWOODS_SCAN_INTERVAL_MS (1200UL)               // Wi-Fi active scan cadence
#endif

#ifndef DEEPWOODS_HOP_MS
#define DEEPWOODS_HOP_MS           (100UL)                // channel hop period
#endif

// -------- Pins (can be overridden via -D in platformio.ini or board JSON) --------
#ifndef ALERT_LED_PIN
#define ALERT_LED_PIN 2           // change to your board’s user LED; -1 disables
#endif

#ifndef ALERT_TONE_PIN
#define ALERT_TONE_PIN -1         // set to a PWM-capable pin; -1 disables tone
#endif

// -------- NimBLE scan parameters --------
#ifndef BLE_SCAN_INTERVAL_MS
#define BLE_SCAN_INTERVAL_MS 60
#endif
#ifndef BLE_SCAN_WINDOW_MS
#define BLE_SCAN_WINDOW_MS   30
#endif