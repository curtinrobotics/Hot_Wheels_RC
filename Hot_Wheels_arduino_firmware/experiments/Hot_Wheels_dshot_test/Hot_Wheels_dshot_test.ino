// DShot test for drone ESCs (ESP32-S3 / Arduino-ESP32)
// Use this when the ESC only has pads for S (signal) and G (ground) and does not arm on 50Hz servo PWM.
//
// Wiring:
//   ESC G  -> ESP32 GND
//   ESC S  -> ESP32 DSHOT_PIN (3.3V signal is usually OK on drone ESCs)
// Power:
//   ESC powered from its LiPo
//   ESP32 powered from USB (or regulator)
//   Grounds MUST be common
//
// Safety:
//   Remove prop / lift wheels off ground.

#include <Arduino.h>

// ---- Pick a safe GPIO ----
// Avoid boot-strapping / special-function pins if unsure.
// On many ESP32-S3 boards, GPIO4/5/6/7 are good choices.
static const int DSHOT_PIN = 4;

// ---- DShot settings ----
// Common options: 150, 300, 600. Start with 300.
static const int DSHOT_KBPS = 300;

// Throttle range:
// 0 = "disarm/stop" (command 0)
// 1..47 = reserved for special commands
// 48..2047 = throttle
static const int DSHOT_THROTTLE_MIN = 48;
static const int DSHOT_THROTTLE_MAX = 800; // keep conservative for testing

// Send rate (frames per second). Many ESCs are happy around 500-2000.
static const int SEND_HZ = 1000;

// ---- RMT (ESP32) ----
// Supports both legacy (Arduino-ESP32 2.x / IDF4) and new (Arduino-ESP32 3.x / IDF5) RMT APIs.
#if __has_include(<driver/rmt.h>)
  #define USE_LEGACY_RMT 1
  #include <driver/rmt.h>
  static const rmt_channel_t RMT_CH = RMT_CHANNEL_0;
#elif __has_include(<driver/rmt_tx.h>)
  #define USE_LEGACY_RMT 0
  #include <driver/rmt_tx.h>
  #include <driver/rmt_encoder.h>
  static rmt_channel_handle_t tx_chan = nullptr;
  static rmt_encoder_handle_t copy_encoder = nullptr;
#else
  #error "No compatible RMT headers found in this Arduino-ESP32 core"
#endif

// 10MHz resolution -> 0.1us per tick, convenient for DShot timings.
static const uint32_t RMT_RESOLUTION_HZ = 10000000UL;

static inline uint16_t makeDshotPacket(uint16_t throttle, bool telemetry) {
  throttle = (uint16_t)constrain(throttle, 0, 2047);
  uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

  // checksum: XOR of 3 nibbles
  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^= (csum_data & 0xF);
    csum_data >>= 4;
  }
  csum &= 0xF;

  return (packet << 4) | csum;
}

// Convert one 16-bit DShot packet into RMT symbols/items.
// DShot bit encoding (approx):
//   '1' -> ~75% high, 25% low
//   '0' -> ~37.5% high, 62.5% low
// Total bit time depends on DSHOT_KBPS.
static void packetToSymbols(uint16_t packet,
#if USE_LEGACY_RMT
                            rmt_item32_t* items,
#else
                            rmt_symbol_word_t* items,
#endif
                            int itemCount,
                            uint16_t ticksPerBit) {
  // Choose high-times as fractions of ticksPerBit.
  // Using integer ticks; tweak if your ESC is picky.
  const uint16_t t1h = (ticksPerBit * 3) / 4;       // 0.75T
  const uint16_t t1l = ticksPerBit - t1h;
  const uint16_t t0h = (ticksPerBit * 3) / 8;       // 0.375T
  const uint16_t t0l = ticksPerBit - t0h;

  for (int i = 0; i < itemCount; i++) {
    const bool bit = (packet & (1 << (15 - i))) != 0;
    items[i].level0 = 1;
    items[i].duration0 = bit ? t1h : t0h;
    items[i].level1 = 0;
    items[i].duration1 = bit ? t1l : t0l;
  }
}

static bool setupRmtForDshot(uint16_t* outTicksPerBit) {
  const uint32_t ticksPerBit32 = RMT_RESOLUTION_HZ / (uint32_t)(DSHOT_KBPS * 1000);
  if (ticksPerBit32 < 10 || ticksPerBit32 > 1000) {
    return false;
  }

  *outTicksPerBit = (uint16_t)ticksPerBit32;

#if USE_LEGACY_RMT
  // Legacy RMT uses APB clock and clk_div; choose clk_div so tick ~= 0.1us.
  // 80MHz / 8 = 10MHz.
  const int clk_div = 8;

  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX;
  config.channel = RMT_CH;
  config.gpio_num = (gpio_num_t)DSHOT_PIN;
  config.mem_block_num = 1;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.clk_div = clk_div;

  esp_err_t err = rmt_config(&config);
  if (err != ESP_OK) return false;
  err = rmt_driver_install(RMT_CH, 0, 0);
  if (err != ESP_OK) return false;
  return true;
#else
  rmt_tx_channel_config_t tx_cfg = {};
  tx_cfg.gpio_num = (gpio_num_t)DSHOT_PIN;
  tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_cfg.resolution_hz = RMT_RESOLUTION_HZ;
  tx_cfg.mem_block_symbols = 64;
  tx_cfg.trans_queue_depth = 4;
  tx_cfg.flags.invert_out = 0;
  tx_cfg.flags.with_dma = 0;

  esp_err_t err = rmt_new_tx_channel(&tx_cfg, &tx_chan);
  if (err != ESP_OK) return false;

  err = rmt_new_copy_encoder(nullptr, &copy_encoder);
  if (err != ESP_OK) return false;

  err = rmt_enable(tx_chan);
  if (err != ESP_OK) return false;

  return true;
#endif
}

static void sendDshotThrottle(uint16_t throttle, uint16_t ticksPerBit) {
  const uint16_t packet = makeDshotPacket(throttle, false);

#if USE_LEGACY_RMT
  rmt_item32_t items[16];
  packetToSymbols(packet, items, 16, ticksPerBit);
  // wait_tx_done=true blocks until finished.
  rmt_write_items(RMT_CH, items, 16, true);
#else
  rmt_symbol_word_t items[16];
  packetToSymbols(packet, items, 16, ticksPerBit);

  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count = 0;

  // Copy encoder will transmit the raw symbols buffer.
  rmt_transmit(tx_chan, copy_encoder, items, sizeof(items), &tx_cfg);
  rmt_tx_wait_all_done(tx_chan, 5 / portTICK_PERIOD_MS);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("DShot ESC test starting...");
  Serial.printf("DSHOT_PIN=%d\n", DSHOT_PIN);
  Serial.printf("DSHOT_KBPS=%d\n", DSHOT_KBPS);
  Serial.printf("SEND_HZ=%d\n", SEND_HZ);

  // Drive pin low immediately to reduce floating behavior on reset.
  pinMode(DSHOT_PIN, OUTPUT);
  digitalWrite(DSHOT_PIN, LOW);

  uint16_t ticksPerBit = 0;
  if (!setupRmtForDshot(&ticksPerBit)) {
    Serial.println("RMT setup failed (check Arduino-ESP32 core / pin / DSHOT_KBPS)");
    while (true) delay(1000);
  }

  Serial.printf("ticksPerBit=%u (RMT_RESOLUTION_HZ=%lu)\n", ticksPerBit, (unsigned long)RMT_RESOLUTION_HZ);

  // Many ESCs require some time of 0-throttle frames to arm.
  Serial.println("Sending 0 throttle for 3 seconds (arming)...");
  const uint32_t periodUs = 1000000UL / SEND_HZ;
  const uint32_t startMs = millis();
  while (millis() - startMs < 3000) {
    sendDshotThrottle(0, ticksPerBit);
    delayMicroseconds(periodUs);
  }

  Serial.println("Armed sequence done. Ramping throttle...");
}

void loop() {
  // ticksPerBit is stable for our chosen RMT resolution.
  static const uint16_t ticksPerBit = (uint16_t)(RMT_RESOLUTION_HZ / (uint32_t)(DSHOT_KBPS * 1000));

  const uint32_t periodUs = 1000000UL / SEND_HZ;

  // Ramp up and down between MIN and MAX.
  for (uint16_t t = DSHOT_THROTTLE_MIN; t <= DSHOT_THROTTLE_MAX; t += 10) {
    sendDshotThrottle(t, ticksPerBit);
    delayMicroseconds(periodUs);

    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs > 200) {
      Serial.printf("Throttle=%u\n", t);
      lastPrintMs = millis();
    }
  }

  for (int t = DSHOT_THROTTLE_MAX; t >= DSHOT_THROTTLE_MIN; t -= 10) {
    sendDshotThrottle((uint16_t)t, ticksPerBit);
    delayMicroseconds(periodUs);
  }

  // Stop briefly
  for (int i = 0; i < SEND_HZ / 2; i++) {
    sendDshotThrottle(0, ticksPerBit);
    delayMicroseconds(periodUs);
  }
}
