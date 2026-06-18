#include <Arduino.h>
#include <Wire.h>

static constexpr int kSdaPin = 4;
static constexpr int kSclPin = 3;

static void printLevels(const char *label) {
  pinMode(kSdaPin, INPUT_PULLUP);
  pinMode(kSclPin, INPUT_PULLUP);
  delay(5);
  Serial.printf("%s: SDA GPIO%d=%d, SCL GPIO%d=%d\n",
                label,
                kSdaPin,
                digitalRead(kSdaPin),
                kSclPin,
                digitalRead(kSclPin));
}

static void printGpioSurvey() {
  Serial.println("GPIO survey with internal pullups:");
  for (int pin = 0; pin <= 10; ++pin) {
    pinMode(pin, INPUT_PULLUP);
    delay(2);
    Serial.printf("  GPIO%-2d = %d\n", pin, digitalRead(pin));
  }
}

static void clockBusFree() {
  pinMode(kSdaPin, INPUT_PULLUP);
  pinMode(kSclPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(kSclPin, HIGH);
  delay(2);

  for (int i = 0; i < 18; ++i) {
    digitalWrite(kSclPin, LOW);
    delayMicroseconds(20);
    digitalWrite(kSclPin, HIGH);
    delayMicroseconds(20);
  }

  pinMode(kSclPin, INPUT_PULLUP);
  delay(2);
}

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void scanExpected() {
  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  delay(20);

  const uint8_t addrs[] = {0x12, 0x36, 0x68};
  for (uint8_t addr : addrs) {
    const bool ok = probe(addr);
    Serial.printf("probe 0x%02X: %s\n", addr, ok ? "online" : "no");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println();
  Serial.println("I2C line diagnostic, ESP32C3 GPIO4=SDA GPIO3=SCL");
  printLevels("idle with pullups");
  printGpioSurvey();
  clockBusFree();
  printLevels("after 18 SCL pulses");
  scanExpected();
  Serial.println("done");
}

void loop() {
  delay(5000);
  printLevels("idle");
  printGpioSurvey();
}
