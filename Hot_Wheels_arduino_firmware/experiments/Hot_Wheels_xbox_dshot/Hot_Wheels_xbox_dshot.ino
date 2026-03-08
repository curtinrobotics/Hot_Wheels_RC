// Xbox controller -> Drone ESC via DShot (ESP32-S3 + Bluepad32)
//
// Wiring:
//   ESC G  -> ESP32 GND
//   ESC S  -> ESP32 DSHOT_PIN
//
// Controls:
//   - Right trigger: throttle
//   - Left trigger: kill (forces 0 throttle)
//
// Notes:
//   - Many drone ESCs do NOT support 50Hz servo PWM; they use DShot.
//   - This sketch outputs DShot (default DShot300) using ESP32 RMT.
//
// Safety:
//   Remove prop / lift wheels off ground before testing.

#include <Arduino.h>
#include <Bluepad32.h>

// ---- GPIO ----
// Pick a safe GPIO for your ESP32-S3 Super Mini.
static const int DSHOT_PIN = 4;

// ---- DShot settings ----
static const int DSHOT_KBPS = 300;     // try 150/300/600 if needed
static const int SEND_HZ = 1000;       // frames per second
static const uint32_t BOOT_ARM_MS = 3000;
static const uint32_t CONNECT_ARM_MS = 1000;
static const uint32_t FAILSAFE_NO_DATA_MS = 250;

// DShot throttle mapping
// 0 = stop/disarm frame
// 48..2047 = throttle
static const int DSHOT_THROTTLE_MIN = 48;
static const int DSHOT_THROTTLE_MAX = 2047;   // conservative for bench testing
static const int SPEED_LIMIT_PERCENT = 100;  // reduce if needed (e.g. 30)

// Xbox trigger mapping
static const int TRIGGER_MAX = 1023;
static const int TRIGGER_DEADZONE = 20;

// ---- RMT backend (legacy + new) ----
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

// 10MHz -> 0.1us per tick
static const uint32_t RMT_RESOLUTION_HZ = 10000000UL;

static inline uint16_t makeDshotPacket(uint16_t throttle, bool telemetry) {
  throttle = (uint16_t)constrain(throttle, 0, 2047);
  uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^= (csum_data & 0xF);
    csum_data >>= 4;
  }
  csum &= 0xF;

  return (packet << 4) | csum;
}

static void packetToSymbols(uint16_t packet,
#if USE_LEGACY_RMT
                            rmt_item32_t* items,
#else
                            rmt_symbol_word_t* items,
#endif
                            int itemCount,
                            uint16_t ticksPerBit) {
  const uint16_t t1h = (ticksPerBit * 3) / 4;
  const uint16_t t1l = ticksPerBit - t1h;
  const uint16_t t0h = (ticksPerBit * 3) / 8;
  const uint16_t t0l = ticksPerBit - t0h;

  for (int i = 0; i < itemCount; i++) {
    const bool bit = (packet & (1 << (15 - i))) != 0;
    items[i].level0 = 1;
    items[i].duration0 = bit ? t1h : t0h;
    items[i].level1 = 0;
    items[i].duration1 = bit ? t1l : t0l;
  }
}

static bool setupRmt(uint16_t* outTicksPerBit) {
  const uint32_t ticksPerBit32 = RMT_RESOLUTION_HZ / (uint32_t)(DSHOT_KBPS * 1000);
  if (ticksPerBit32 < 10 || ticksPerBit32 > 1000) {
    return false;
  }
  *outTicksPerBit = (uint16_t)ticksPerBit32;

#if USE_LEGACY_RMT
  const int clk_div = 8;  // 80MHz / 8 = 10MHz ~ 0.1us

  rmt_config_t cfg = {};
  cfg.rmt_mode = RMT_MODE_TX;
  cfg.channel = RMT_CH;
  cfg.gpio_num = (gpio_num_t)DSHOT_PIN;
  cfg.mem_block_num = 1;
  cfg.tx_config.loop_en = false;
  cfg.tx_config.carrier_en = false;
  cfg.tx_config.idle_output_en = true;
  cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  cfg.clk_div = clk_div;

  if (rmt_config(&cfg) != ESP_OK) return false;
  if (rmt_driver_install(RMT_CH, 0, 0) != ESP_OK) return false;
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

  if (rmt_new_tx_channel(&tx_cfg, &tx_chan) != ESP_OK) return false;
  if (rmt_new_copy_encoder(nullptr, &copy_encoder) != ESP_OK) return false;
  if (rmt_enable(tx_chan) != ESP_OK) return false;
  return true;
#endif
}

static void sendDshot(uint16_t throttle, uint16_t ticksPerBit) {
  const uint16_t packet = makeDshotPacket(throttle, false);

#if USE_LEGACY_RMT
  rmt_item32_t items[16];
  packetToSymbols(packet, items, 16, ticksPerBit);
  rmt_write_items(RMT_CH, items, 16, true);
#else
  rmt_symbol_word_t items[16];
  packetToSymbols(packet, items, 16, ticksPerBit);

  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count = 0;
  rmt_transmit(tx_chan, copy_encoder, items, sizeof(items), &tx_cfg);
  rmt_tx_wait_all_done(tx_chan, 5 / portTICK_PERIOD_MS);
#endif
}

// ---- Controller state ----
ControllerPtr controller = nullptr;
uint32_t connectedAtMs = 0;
uint32_t lastDataAtMs = 0;

uint16_t ticksPerBit = 0;

void onConnectedController(ControllerPtr ctl) {
  if (controller == nullptr) {
    controller = ctl;
    connectedAtMs = millis();
    lastDataAtMs = connectedAtMs;

    Serial.print("Controller connected, index=");
    Serial.println(ctl->index());
    ctl->setColorLED(0, 255, 0);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (controller == ctl) {
    Serial.print("Controller disconnected, index=");
    Serial.println(ctl->index());
    controller = nullptr;
    connectedAtMs = 0;
    lastDataAtMs = 0;
  }
}

static inline uint16_t mapTriggerToDshot(int trigger) {
  if (trigger < TRIGGER_DEADZONE) return 0;

  const int limitedMax = (DSHOT_THROTTLE_MAX * SPEED_LIMIT_PERCENT) / 100;
  const int maxThrottle = constrain(limitedMax, DSHOT_THROTTLE_MIN, 2047);

  // Map 0..1023 -> MIN..MAX
  const int t = constrain(trigger, 0, TRIGGER_MAX);
  const int out = map(t, 0, TRIGGER_MAX, DSHOT_THROTTLE_MIN, maxThrottle);
  return (uint16_t)constrain(out, 0, 2047);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Xbox -> DShot ESC controller starting...");
  Serial.printf("DSHOT_PIN=%d\n", DSHOT_PIN);
  Serial.printf("DSHOT_KBPS=%d\n", DSHOT_KBPS);
  Serial.printf("SEND_HZ=%d\n", SEND_HZ);

  // Hold low immediately so the pad isn't floating during boot.
  pinMode(DSHOT_PIN, OUTPUT);
  digitalWrite(DSHOT_PIN, LOW);

  if (!setupRmt(&ticksPerBit)) {
    Serial.println("RMT setup failed. Try different DSHOT_KBPS or update Arduino-ESP32 core.");
    while (true) delay(1000);
  }
  Serial.printf("ticksPerBit=%u\n", ticksPerBit);

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Boot arming window: send 0 throttle for a few seconds.
  Serial.printf("Sending 0 throttle for %lums (boot arming)...\n", (unsigned long)BOOT_ARM_MS);
  const uint32_t periodUs = 1000000UL / SEND_HZ;
  const uint32_t start = millis();
  while (millis() - start < BOOT_ARM_MS) {
    sendDshot(0, ticksPerBit);
    delayMicroseconds(periodUs);
  }

  Serial.println("Ready. Connect controller; use RT to throttle, LT to kill.");
}

void loop() {
  BP32.update();

  const uint32_t nowMs = millis();

  // Determine desired throttle
  uint16_t desired = 0;

  if (controller && controller->isConnected()) {
    if (controller->hasData()) {
      lastDataAtMs = nowMs;
    }

    // Failsafe if controller stops sending updates
    if (lastDataAtMs && (nowMs - lastDataAtMs > FAILSAFE_NO_DATA_MS)) {
      desired = 0;
    } else {
      // Extra arming window right after connect
      if (nowMs - connectedAtMs < CONNECT_ARM_MS) {
        desired = 0;
      } else {
        const int rt = controller->throttle();
        const int lt = controller->brake();

        if (lt > TRIGGER_DEADZONE) {
          desired = 0;
        } else {
          desired = mapTriggerToDshot(rt);
        }

        static uint32_t lastPrint = 0;
        if (nowMs - lastPrint > 250) {
          Serial.printf("rt=%d lt=%d -> dshot=%u\n", rt, lt, desired);
          lastPrint = nowMs;
        }
      }
    }
  } else {
    desired = 0;
  }

  // Send at a steady rate
  static uint32_t nextUs = 0;
  const uint32_t periodUs = 1000000UL / SEND_HZ;
  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextUs) >= 0) {
    sendDshot(desired, ticksPerBit);
    nextUs = nowUs + periodUs;
  }
}
