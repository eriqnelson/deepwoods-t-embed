# Deepwoods Device Detection — Standalone Firmware (ESP32‑S3 T‑Embed CC1101)

This is a **standalone firmware** for the LilyGO **T‑Embed‑CC1101** (ESP32‑S3) that performs device environment detection. It is derived from and inspired by the original Arduino application by **colonelpanichacks**:

> Upstream project: https://github.com/colonelpanichacks/deepwoods_device_detection

This firmware ports the core ideas to a single binary you can flash directly to the T‑Embed. It handles Wi‑Fi AP scans, Wi‑Fi probe requests (promiscuous mode), BLE advertisements, a timed baseline window, and on‑device UI.

---

## What’s different from the original

- **Platform target**: purpose‑built for **LilyGO T‑Embed‑CC1101 (ESP32‑S3)**.
- **Display**: on‑device status UI via **LovyanGFX** (ST7789 170×320, rotated, still trying to resolve the color inversion issue).
- **BLE stack**: **NimBLE** on ESP32‑S3, with Classic BT RAM released for stability.
- **Wi‑Fi + BLE coexistence**: Wi‑Fi runs with **modem sleep enabled** (`WiFi.setSleep(true)`) to satisfy S3 coexistence requirements and avoid coex abort loops.
- **Probe request parsing fix**: correct 802.11 mgmt **subtype extraction** from high nibble to count ProbeReq properly.
- **Channel hop**: steady hopping across 1–13 with adjustable dwell via build flag.
- **Build system**: **PlatformIO** environment `deepwoods-t-embed-pn532`.

Not yet included compared to upstream examples:
- Encoder LED ring alert (WS2812) — planned.
- Buzzer/audio alert — planned.
- Meshtastic UART queue + send helper — stub only in this firmware.
- Persistent baseline across reboots — planned (NVS/LittleFS snapshot).

---

## Features

- **Baseline scanning**: 5‑minute collection of Wi‑Fi APs, ProbeReq, and BLE ads.
- **Deviation detection**: any post‑baseline device triggers a visible/logged alert.
- **Wi‑Fi probe detection**: promiscuous sniff with mgmt filter + ISR queue.
- **BLE detection**: continuous NimBLE scan with periodic cache clears.
- **On‑device UI**: black background, green text, larger font for readability.
- **Serial logging**: detailed progress and detection lines over USB CDC.

---

## Hardware

- Board: **LilyGO T‑Embed‑PN532 / CC1101** (ESP32‑S3, ST7789 170×320)
- Display: ST7789 on SPI (pins per LilyGO board JSON); handled by LovyanGFX.
- Backlight: driven by board’s BL pin (optional if you enable it in code/flags).
- Encoder ring LEDs (WS2812): **not yet wired in this firmware** (planned).

---

## Build & Flash (PlatformIO)

1) Install PlatformIO (macOS example):
```bash
brew install platformio
```

2) Build:
```bash
pio run -e deepwoods-t-embed-pn532
```

3) Flash (replace the serial port with yours):
```bash
pio run -e deepwoods-t-embed-pn532 -t upload --upload-port /dev/cu.usbmodemXXXX
```

4) Monitor:
```bash
pio device monitor -e deepwoods-t-embed-pn532
```

The provided `platformio.ini` pins the correct platform/framework and libraries:
- **NimBLE‑Arduino**
- **LovyanGFX**
- **WiFi** (Arduino core for ESP32)

---

## Runtime Behavior

- On boot you’ll see a banner and: *“5 minute Baseline Scan Started”*.
- During the baseline window (default **5 minutes**), counters update periodically.
- After baseline completes, any new Wi‑Fi AP, ProbeReq MAC, or BLE advertiser logs as:
  - `Detected non-baseline WiFi: XX:XX:…`
  - `Detected non-baseline ProbeReq: XX:XX:…`
  - `Detected non-baseline BLE: XX:XX:…`
- Display shows simple counters and status; serial mirror includes details.

---

## Configuration (build flags)

Edit `platformio.ini` under the `deepwoods-t-embed-pn532` environment.

Common flags:

```ini
; Baseline window (ms)
-DDEEPWOODS_BASELINE_MS=300000

; Channel hop dwell (ms)
; -DDEEPWOODS_HOP_MS=100   ; e.g., 100–200

; Optional: set an explicit BL pin to enable backlight flash alerts
; -DALERT_BL_PIN=21
; -DALERT_BL_PWM_CH=7
; -DALERT_BL_FREQ=5000
; -DALERT_BL_DUTY_ON=255
; -DALERT_BL_DUTY_DIM=40
```

> **Note:** For ESP32‑S3 coexistence stability, Wi‑Fi runs with **modem sleep ON** while BLE is enabled. Leave that as configured in the firmware unless you know your coexistence behavior on your specific IDF.

---

## Troubleshooting


- **Probe counter stays 0**
  - Try toggling Wi‑Fi OFF→ON on a nearby phone to generate probes.

- **Display inverted/upside down**
  - Panel is configured with the proper **invert** and **RGB order** for T‑Embed ST7789. If you change boards, adjust those in `main.cpp` (LovyanGFX config) or set rotation.

- **Upload errors / port changes**
  - Close the serial monitor before flashing; set `upload_port` in `platformio.ini` to lock the port.

---

## Roadmap

- Encoder ring **WS2812 flash** alert
- Optional **buzzer** tone on detection
- **Meshtastic** UART integration for sending detection snippets
- **Persist baseline** to NVS/LittleFS and add a “Re‑baseline” button/command
- More detailed **on‑device UI** (pages, recent detections list)

---

## Attribution

- Original concept and code: **colonelpanichacks / deepwoods_device_detection**  
  https://github.com/colonelpanichacks/deepwoods_device_detection

This firmware is an independent re‑implementation for a specific ESP32‑S3 board, with radio coexistence fixes and a built‑in display UI.

## License

Same as the upstream project (MIT), unless specified otherwise in this repo.
