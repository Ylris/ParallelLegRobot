#include <Arduino.h>

static constexpr int LED_PIN = 10;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("ParallelLegRobot board smoke test");
  Serial.println("If you can read this, USB serial and firmware boot are OK.");
}

void loop() {
  static uint32_t count = 0;
  digitalWrite(LED_PIN, (count % 2) == 0 ? HIGH : LOW);
  Serial.printf("alive %lu ms=%lu\n", static_cast<unsigned long>(count),
                static_cast<unsigned long>(millis()));
  count++;
  delay(500);
}
