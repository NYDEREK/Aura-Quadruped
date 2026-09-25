#include <Arduino.h>
#include <ps5Controller.h>
#include <atomic>

static std::atomic<uint32_t> reports{0}, lastReportMs{0};
static void onInput() { ++reports; lastReportMs.store(millis()); }

void setup() {
  Serial.begin(115200);
  pinMode(33, OUTPUT);
  Serial.println("Aura isolated reference: esp-ps5 / Arduino-ESP32 3.3.6; Wi-Fi OFF. External power required.");
  ps5.attach(onInput);
  // Known controller; no GPIO peripherals, Wi-Fi, servo or LED-strip drivers.
  ps5.begin("a0:fa:9c:68:71:46");
}

void loop() {
  bool connected = ps5.isConnected(); // Library drives its handshake/reconnect here.
  static uint32_t printed = 0;
  uint32_t now = millis();
  digitalWrite(33, (now / 500) & 1);
  if (now - printed >= 500) {
    printed = now;
    uint32_t count = reports.load(), age = now - lastReportMs.load();
    Serial.printf("PAD connected=%d reports=%lu age_ms=%lu LX=%d LY=%d RX=%d RY=%d L2=%u R2=%u X=%d heap=%lu\n",
      connected, (unsigned long)count, (unsigned long)age,
      ps5.lx, ps5.ly, ps5.rx, ps5.ry, ps5.l2, ps5.r2,
      (bool)ps5.cross, (unsigned long)ESP.getFreeHeap());
  }
  delay(10);
}
