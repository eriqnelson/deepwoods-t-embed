// Deepwoods Device Detection — standalone firmware (ESP32-S3 / LilyGO T-Embed C1101)
// v1: 5-min baseline of BLE advertisers + Wi-Fi AP BSSIDs + Wi-Fi Probe Requests
// Alerts: LED flash + optional short tone
//
// Build notes:
// - PlatformIO env should include NimBLE-Arduino and enable USB CDC on boot.
// - Provide a custom board JSON or use an ESP32-S3 devkit; define ALERT_* pins as needed.
//
// Project headers
#include "config.h"
#include "deepwoods.h"
#include "alert.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_bt.h>
#include <NimBLEDevice.h>
// Helper: treat negative pins as "not present"
static inline bool has_pin(int pin) { return pin >= 0; }
#ifdef USE_LOVYANGFX
#include <LovyanGFX.hpp>
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_SPI _bus;      // SPI bus
  lgfx::Panel_ST7789 _panel; // ST7789 panel (170x320)
public:
  LGFX() {
    { // SPI bus config
      auto cfg = _bus.config();
      cfg.spi_host   = SPI3_HOST;   // VSPI on ESP32-S3
      cfg.spi_mode   = 0;
      cfg.freq_write = 40000000;    // 40 MHz write
      cfg.freq_read  = 16000000;    // 16 MHz read (unused)
      cfg.spi_3wire  = true;        // MOSI only
      cfg.use_lock   = true;
      cfg.dma_channel= 1;
      cfg.pin_sclk   = 11;
      cfg.pin_mosi   = 9;
      cfg.pin_miso   = -1;          // not used
      cfg.pin_dc    = 16;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // panel config
      auto cfg = _panel.config();
      cfg.pin_cs   = 41;
      cfg.pin_rst  = 40;
      cfg.pin_busy = -1;
      cfg.memory_width  = 240;   // ST7789 native RAM
      cfg.memory_height = 320;
      cfg.panel_width   = 170;   // visible area
      cfg.panel_height  = 320;
      cfg.offset_x = 35;         // center 170 within 240
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable = false;
      cfg.invert   = false;
      cfg.rgb_order= true;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};
static LGFX lcd;
static void lcd_begin() {
  lcd.init();
  lcd.setColorDepth(16);
  lcd.setRotation(1);
  lcd.invertDisplay(false);

  // Clean canvas + text state
  lcd.fillScreen(0x0000);                 // black background
  lcd.setTextColor(0x07E0, 0x0000);       // green on black
  lcd.setTextWrap(false);
  lcd.setTextSize(2);

  // Header and mode line
  int16_t lh = lcd.fontHeight();
  lcd.setCursor(10, 10);
  lcd.print("Deepwoods firmware");
  lcd.setCursor(10, 10 + lh + 6);
  lcd.print("Baseline starting...");
}
static void lcd_status(int ble, int wifi, int probe, bool baseline, uint32_t rem_ms) {
  {
    // Full redraw to avoid leftover artifacts
    lcd.fillScreen(0x0000);
    lcd.setTextColor(0x07E0, 0x0000);
    lcd.setTextWrap(false);
    lcd.setTextSize(2);

    // Header + mode
    int16_t lh = lcd.fontHeight();
    lcd.setCursor(10, 10);
    lcd.print("Deepwoods firmware");
    lcd.setCursor(10, 10 + lh + 6);
    lcd.print(baseline ? "Baseline starting..." : "Monitoring");

    // Counters
    int16_t y0 = 10 + (lh + 6) * 2 + 6;
    lcd.setCursor(10, y0);              lcd.printf("BLE: %d",  ble);
    lcd.setCursor(10, y0 + lh + 6);     lcd.printf("WiFi: %d", wifi);
    lcd.setCursor(10, y0 + 2*(lh+6));   lcd.printf("Probe: %d", probe);
    if (baseline) {
      lcd.setCursor(10, y0 + 3*(lh+6));
      lcd.printf("Rem: %u m", (unsigned)((rem_ms + 59999)/60000));
    }
  }
}
#endif

// Board power enable for peripherals (per LilyGO repo): GPIO 15
// Removed per instructions

// ---------- Alert implementation (simple, can be moved into src/alert.cpp later) ----------
void alert_init() {
  if (has_pin(ALERT_LED_PIN)) {
    pinMode(ALERT_LED_PIN, OUTPUT);
    digitalWrite(ALERT_LED_PIN, LOW);
  }
  if (has_pin(ALERT_TONE_PIN)) {
    #ifndef ALERT_TONE_PWM_CH
    #define ALERT_TONE_PWM_CH 6
    #endif
    ledcSetup(ALERT_TONE_PWM_CH, 2000, 8);
    ledcAttachPin(ALERT_TONE_PIN, ALERT_TONE_PWM_CH);
    ledcWrite(ALERT_TONE_PWM_CH, 0);
  }
  #ifdef ALERT_BL_PIN
  if (has_pin(ALERT_BL_PIN)) {
    #ifndef ALERT_BL_PWM_CH
    #define ALERT_BL_PWM_CH 7
    #endif
    #ifndef ALERT_BL_FREQ
    #define ALERT_BL_FREQ 5000
    #endif
    #ifndef ALERT_BL_DUTY_ON
    #define ALERT_BL_DUTY_ON 255
    #endif
    #ifndef ALERT_BL_DUTY_DIM
    #define ALERT_BL_DUTY_DIM 40
    #endif
    ledcSetup(ALERT_BL_PWM_CH, ALERT_BL_FREQ, 8);
    ledcAttachPin(ALERT_BL_PIN, ALERT_BL_PWM_CH);
    ledcWrite(ALERT_BL_PWM_CH, ALERT_BL_DUTY_ON);
  }
  #endif
}

void alert_pulse() {
  if (has_pin(ALERT_LED_PIN)) {
    digitalWrite(ALERT_LED_PIN, HIGH);
    delay(60);
    digitalWrite(ALERT_LED_PIN, LOW);
  }
  #ifdef ALERT_TONE_PWM_CH
  if (has_pin(ALERT_TONE_PIN)) {
    ledcWrite(ALERT_TONE_PWM_CH, 128);
    delay(80);
    ledcWrite(ALERT_TONE_PWM_CH, 0);
  }
  #endif
  #ifdef ALERT_BL_PWM_CH
  if (has_pin(ALERT_BL_PIN)) {
    ledcWrite(ALERT_BL_PWM_CH, ALERT_BL_DUTY_DIM);
    delay(90);
    ledcWrite(ALERT_BL_PWM_CH, ALERT_BL_DUTY_ON);
  }
  #endif
}

// ---------- Deepwoods implementation ----------
namespace deepwoods {

// ---- Internal config ----
static const uint32_t BASELINE_MS      = DEEPWOODS_BASELINE_MS;
static const uint32_t SCAN_INTERVAL_MS = DEEPWOODS_SCAN_INTERVAL_MS;
static const uint32_t HOP_MS           = DEEPWOODS_HOP_MS;

// ---- State ----
static volatile bool s_isBaseline = true;
static uint32_t s_baselineStartMs = 0;

// Fixed-size sets (simple O(n) membership; compact and deterministic)
static char bleSeen[MAX_BLE][18];           static int bleCount = 0;
static char wifiSeen[MAX_WIFI][18];         static int wifiCount = 0;
static char probeSeen[MAX_PROBE][18];       static int probeCount = 0;

static char baseBLE[MAX_BLE][18];           static int baseBLECount = 0;
static char baseWIFI[MAX_WIFI][18];         static int baseWIFICount = 0;
static char basePROB[MAX_PROBE][18];        static int basePROBCount = 0;

// Tasks
static TaskHandle_t tBLE = nullptr, tWiFi = nullptr, tPromisc = nullptr, tHop = nullptr, tStatus = nullptr;
static volatile bool s_bleInit = false;

// Probe queue (ISR -> task)
typedef struct { char mac[18]; } probe_evt_t;
static QueueHandle_t gProbeQ = nullptr;

// ---- Utils ----
static inline uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static bool contains(char (*arr)[18], int count, const char* mac) {
  for (int i = 0; i < count; i++) if (strcmp(arr[i], mac) == 0) return true;
  return false;
}

static bool addIfNew(char (*arr)[18], int &count, int cap, const char* mac) {
  if (!mac || !*mac) return false;
  if (contains(arr, count, mac)) return false;
  if (count >= cap) return false;
  strncpy(arr[count], mac, 17);
  arr[count][17] = '\0';
  count++;
  return true;
}

#ifdef USE_LOVYANGFX
static void raise_alert(const char* kind, const char* mac) {
  if (kind && mac) Serial.printf("Detected non-baseline %s: %s\n", kind, mac);
  alert_pulse();
  // Draw alert banner using dynamic height and position
  int16_t lh = lcd.fontHeight();
  int16_t band_h = lh + 6;
  int16_t band_y = lcd.height() - band_h - 2; // bottom band
  if (band_y < 0) band_y = 0;
  lcd.fillRect(0, band_y, lcd.width(), band_h + 2, 0x0000);
  lcd.setCursor(10, band_y + 2);
}
#else
static void raise_alert(const char* kind, const char* mac) {
  if (kind && mac) Serial.printf("Detected non-baseline %s: %s\n", kind, mac);
  alert_pulse();
}
#endif

// ---- BLE active scan ----
class DWScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    std::string a = adv->getAddress().toString();
    char mac[18]; strncpy(mac, a.c_str(), 17); mac[17] = '\0';
    if (addIfNew(bleSeen, bleCount, MAX_BLE, mac) && !s_isBaseline) {
      if (!contains(baseBLE, baseBLECount, mac)) raise_alert("BLE", mac);
    }
  }
};
static DWScanCB g_bleCB;

static void BLETask(void*) {
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  NimBLEDevice::init("");
  s_bleInit = true;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(BLE_SCAN_INTERVAL_MS);
  scan->setWindow(BLE_SCAN_WINDOW_MS);
  scan->setScanCallbacks(&g_bleCB, true);
  scan->start(0, true); // continuous; NimBLE runs callbacks internally
  vTaskDelete(nullptr);
}

// ---- Wi-Fi active scan ----
static void WiFiScanTask(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);   // IDF requires modem sleep when BLE + WiFi are both on
  for (;;) {
    // Kick off async scan
    WiFi.scanNetworks(true, true, false, 0);
    vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));

    // Read results using esp_wifi API for BSSID access
    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    if (ap_num > 0) {
      wifi_ap_record_t* buf = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * ap_num);
      if (buf) {
        esp_wifi_scan_get_ap_records(&ap_num, buf);
        for (int i = 0; i < ap_num; i++) {
          const uint8_t* b = buf[i].bssid;
          char mac[18];
          sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", b[0], b[1], b[2], b[3], b[4], b[5]);
          if (addIfNew(wifiSeen, wifiCount, MAX_WIFI, mac) && !s_isBaseline) {
            if (!contains(baseWIFI, baseWIFICount, mac)) raise_alert("WiFi", mac);
          }
        }
        free(buf);
      }
    }
    WiFi.scanDelete();
    esp_wifi_set_promiscuous(true);  // re-arm in case scan disabled it
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---- Wi-Fi promiscuous: Probe Requests ----
static void IRAM_ATTR promisc_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t* p = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* hdr = p->payload;
  // Mgmt subtype 4 == Probe Request
  uint8_t subtype = (hdr[0] >> 4) & 0x0F; if (subtype != 4) return;

  probe_evt_t ev;
  sprintf(ev.mac, "%02X:%02X:%02X:%02X:%02X:%02X",
          hdr[10], hdr[11], hdr[12], hdr[13], hdr[14], hdr[15]);
  BaseType_t hpw = pdFALSE;
  if (gProbeQ) xQueueSendFromISR(gProbeQ, &ev, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

static void PromiscTask(void*) {
  // Wait for Wi‑Fi driver to be started before enabling promiscuous
  wifi_promiscuous_filter_t filt{};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT; // only management frames

  esp_err_t err;
  // Retry setting filter until successful
  while ((err = esp_wifi_set_promiscuous_filter(&filt)) != ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  // Retry installing RX callback until accepted
  while ((err = esp_wifi_set_promiscuous_rx_cb(&promisc_cb)) != ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  // Retry enabling promiscuous until it sticks
  while ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  for(;;) {
    probe_evt_t ev;
    if (xQueueReceive(gProbeQ, &ev, portMAX_DELAY) == pdTRUE) {
      if (addIfNew(probeSeen, probeCount, MAX_PROBE, ev.mac) && !s_isBaseline) {
        if (!contains(basePROB, basePROBCount, ev.mac)) raise_alert("ProbeReq", ev.mac);
      }
    }
  }
}

// ---- Channel hopper for promiscuous capture ----
static void ChannelHopTask(void*) {
  uint8_t ch = 1;
  for (;;) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    ch = (ch % 13) + 1;
    vTaskDelay(pdMS_TO_TICKS(HOP_MS));
  }
}

// ---- Status / baseline manager ----
static void StatusTask(void*) {
  uint32_t lastPrint = 0;
  for (;;) {
    uint32_t now = now_ms();
    if (s_isBaseline) {
      uint32_t elapsed = now - s_baselineStartMs;
      if (now - lastPrint > 15000) {
        uint32_t rem = (elapsed < BASELINE_MS) ? (BASELINE_MS - elapsed) : 0;
        Serial.printf("Baseline progress: BLE %d, WiFi %d, Probe %d\n",
                      bleCount, wifiCount, probeCount);
        if (rem) {
          Serial.printf("Baseline Scan has %u minutes remaining.\n",
                        (unsigned)((rem + 59999) / 60000));
        }
        #ifdef USE_LOVYANGFX
        lcd_status(bleCount, wifiCount, probeCount, true, rem);
        #endif
        lastPrint = now;
      }
      if (elapsed >= BASELINE_MS) {
        // Freeze baseline
        baseBLECount  = bleCount;   for (int i = 0; i < bleCount;   i++) strncpy(baseBLE[i],  bleSeen[i],  18);
        baseWIFICount = wifiCount;  for (int i = 0; i < wifiCount;  i++) strncpy(baseWIFI[i], wifiSeen[i], 18);
        basePROBCount = probeCount; for (int i = 0; i < probeCount; i++) strncpy(basePROB[i], probeSeen[i],18);
        s_isBaseline = false;
        Serial.printf("Baseline complete: BLE %d, WiFi %d, Probe %d\n", bleCount, wifiCount, probeCount);
        #ifdef USE_LOVYANGFX
        lcd_status(bleCount, wifiCount, probeCount, false, 0);
        #endif
      }
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ---- Public API ----
void startBaseline() {
  if (tBLE || tWiFi || tPromisc || tHop || tStatus || s_bleInit || gProbeQ) {
    stop();
  }

  // Clear sets
  bleCount = wifiCount = probeCount = 0;
  baseBLECount = baseWIFICount = basePROBCount = 0;

  // Init alerts
  alert_init();

  // Radio setup
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);  // required by ESP32-S3 when BLE + WiFi coexist

  // Avoid NimBLEDevice::deinit crash if not previously initialized
  if (s_bleInit) {
    NimBLEDevice::deinit(true);
    s_bleInit = false;
  }

  // Queue
  if (gProbeQ) vQueueDelete(gProbeQ);
  gProbeQ = xQueueCreate(256, sizeof(probe_evt_t));

  // Baseline start
  s_isBaseline = true;
  s_baselineStartMs = now_ms();

  // Tasks: bring Wi‑Fi up first to avoid coex aborts, then BLE task
  xTaskCreatePinnedToCore(WiFiScanTask,   "dw_wifi",  8192, nullptr, 1, &tWiFi,    1);
  xTaskCreatePinnedToCore(PromiscTask,    "dw_pr",    4096, nullptr, 1, &tPromisc, 1);
  xTaskCreatePinnedToCore(ChannelHopTask, "dw_hop",   2048, nullptr, 1, &tHop,     1);
  xTaskCreatePinnedToCore(StatusTask,     "dw_stat",  4096, nullptr, 1, &tStatus,  0);
  xTaskCreatePinnedToCore(BLETask,        "dw_ble",   8192, nullptr, 1, &tBLE,     0);

  Serial.println("Deepwoods: 5-minute Baseline Scan Started");
}

void resetBaseline() { startBaseline(); }

void stop() {
  auto kill = [](TaskHandle_t& h) { if (h) { vTaskDelete(h); h = nullptr; } };
  kill(tBLE); kill(tWiFi); kill(tPromisc); kill(tHop); kill(tStatus);

  esp_wifi_set_promiscuous(false);
  if (s_bleInit) {
    NimBLEDevice::deinit(true);
    s_bleInit = false;
  }

  if (gProbeQ) { vQueueDelete(gProbeQ); gProbeQ = nullptr; }
}

void showStatus() {
  Serial.printf("Baseline %s | BLE %d | WiFi %d | Probe %d\n",
                s_isBaseline ? "ACTIVE" : "DONE", bleCount, wifiCount, probeCount);
}

Counts getCounts() {
  Counts c;
  c.ble  = bleCount;
  c.wifi = wifiCount;
  c.probe= probeCount;
  c.baseline_active = s_isBaseline;
  if (s_isBaseline) {
    uint32_t elapsed = now_ms() - s_baselineStartMs;
    c.baseline_ms_remaining = (elapsed < BASELINE_MS) ? (BASELINE_MS - elapsed) : 0;
  } else {
    c.baseline_ms_remaining = 0;
  }
  return c;
}

} // namespace deepwoods

// ---------- Arduino lifecycle ----------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== Deepwoods Device Detection (firmware) ===");
  #ifdef USE_LOVYANGFX
    lcd_begin();
  #endif
  // Ensure NVS is initialized before bringing up BLE/Wi‑Fi subsystems
  esp_err_t nvsr = nvs_flash_init();
  if (nvsr == ESP_ERR_NVS_NO_FREE_PAGES || nvsr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  deepwoods::startBaseline();
}

void loop() {
  // All work handled in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
