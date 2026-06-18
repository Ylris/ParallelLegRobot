#include <Arduino.h>
#include <Wire.h>

#ifndef WHEEL_I2C_ADDR
#define WHEEL_I2C_ADDR 0x12
#endif

#ifndef WHEEL_PWM_LIMIT
#define WHEEL_PWM_LIMIT 1000
#endif

static constexpr uint8_t kCmdSetPwm = 0x10;
static constexpr uint32_t kCommandTimeoutMs = 100;

static constexpr uint8_t kLeftPwmPin = PA0;
static constexpr uint8_t kRightPwmPin = PA1;
static constexpr uint8_t kLeftDirPin = PA2;
static constexpr uint8_t kRightDirPin = PA3;
static constexpr uint8_t kStatusLedPin = PC13;
static constexpr uint8_t kI2cSclPin = PB6;
static constexpr uint8_t kI2cSdaPin = PB7;

static volatile int16_t requested_left_pwm = 0;
static volatile int16_t requested_right_pwm = 0;
static volatile uint32_t last_command_ms = 0;
static volatile bool command_seen = false;

static int16_t clampPwm(int16_t value) {
  if (value > WHEEL_PWM_LIMIT) return WHEEL_PWM_LIMIT;
  if (value < -WHEEL_PWM_LIMIT) return -WHEEL_PWM_LIMIT;
  return value;
}

static int16_t readInt16LE(uint8_t lo, uint8_t hi) {
  return static_cast<int16_t>(static_cast<uint16_t>(lo) |
                              (static_cast<uint16_t>(hi) << 8));
}

static void applyOneWheel(uint8_t pwm_pin, uint8_t dir_pin, int16_t pwm) {
  const bool forward = pwm >= 0;
  const uint16_t duty = static_cast<uint16_t>(abs(pwm));

  digitalWrite(dir_pin, forward ? HIGH : LOW);
  analogWrite(pwm_pin, duty);
}

static void applyWheelPwm(int16_t left_pwm, int16_t right_pwm) {
  applyOneWheel(kLeftPwmPin, kLeftDirPin, left_pwm);
  applyOneWheel(kRightPwmPin, kRightDirPin, right_pwm);
}

static void stopWheels() {
  requested_left_pwm = 0;
  requested_right_pwm = 0;
  applyWheelPwm(0, 0);
}

static void onI2cReceive(int count) {
  if (count < 1) return;

  const uint8_t cmd = Wire.read();
  count--;

  if (cmd == kCmdSetPwm && count >= 4) {
    const uint8_t l0 = Wire.read();
    const uint8_t l1 = Wire.read();
    const uint8_t r0 = Wire.read();
    const uint8_t r1 = Wire.read();

    requested_left_pwm = clampPwm(readInt16LE(l0, l1));
    requested_right_pwm = clampPwm(readInt16LE(r0, r1));
    last_command_ms = millis();
    command_seen = true;
  }

  while (Wire.available()) {
    (void)Wire.read();
  }
}

void setup() {
  pinMode(kLeftPwmPin, OUTPUT);
  pinMode(kRightPwmPin, OUTPUT);
  pinMode(kLeftDirPin, OUTPUT);
  pinMode(kRightDirPin, OUTPUT);
  pinMode(kStatusLedPin, OUTPUT);

  analogWriteResolution(10);
  stopWheels();

  Wire.setSCL(kI2cSclPin);
  Wire.setSDA(kI2cSdaPin);
  Wire.begin(WHEEL_I2C_ADDR);
  Wire.onReceive(onI2cReceive);

  last_command_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  if (!command_seen || now - last_command_ms > kCommandTimeoutMs) {
    stopWheels();
  } else {
    applyWheelPwm(requested_left_pwm, requested_right_pwm);
  }

  digitalWrite(kStatusLedPin, command_seen && (now - last_command_ms <= kCommandTimeoutMs) ? LOW : HIGH);
  delay(5);
}
