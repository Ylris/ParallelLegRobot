#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#ifndef WHEEL_I2C_ADDR
#define WHEEL_I2C_ADDR 0x12
#endif

#ifndef WHEEL_PWM_LIMIT
#define WHEEL_PWM_LIMIT 1000
#endif

#ifndef MOTOR_POLE_PAIRS
#define MOTOR_POLE_PAIRS 11
#endif

#ifndef POWER_SUPPLY_VOLTAGE
#define POWER_SUPPLY_VOLTAGE 12.0f
#endif

static constexpr uint8_t kCmdSetPwm = 0x10;
static constexpr uint32_t kCommandTimeoutMs = 100;

static constexpr uint8_t kStatusLedPin = PC13;
static constexpr uint8_t kI2cSclPin = PB6;
static constexpr uint8_t kI2cSdaPin = PB7;

// Custom external sensor class that reads angle sent over I2C from ESP32-C3
class ExternalSensor : public Sensor {
public:
  float current_angle = 0.0f;
  float getSensorAngle() override {
    return current_angle;
  }
};

static ExternalSensor sensor_left;
static ExternalSensor sensor_right;

// SimpleFOC BLDC Motors (Torque/Voltage mode)
static BLDCMotor motor_left = BLDCMotor(MOTOR_POLE_PAIRS);
static BLDCDriver3PWM driver_left = BLDCDriver3PWM(PA0, PA1, PA2, PA4);

static BLDCMotor motor_right = BLDCMotor(MOTOR_POLE_PAIRS);
static BLDCDriver3PWM driver_right = BLDCDriver3PWM(PA8, PA9, PA10, PA5);

static volatile float target_voltage_left = 0.0f;
static volatile float target_voltage_right = 0.0f;
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

static void onI2cReceive(int count) {
  if (count < 1) return;

  const uint8_t cmd = Wire.read();
  count--;

  // Payload: 1-byte command (0x10) + 2-byte left_pwm + 2-byte right_pwm + 2-byte left_encoder_raw + 2-byte right_encoder_raw
  if (cmd == kCmdSetPwm && count >= 8) {
    const uint8_t l0 = Wire.read();
    const uint8_t l1 = Wire.read();
    const uint8_t r0 = Wire.read();
    const uint8_t r1 = Wire.read();
    const uint8_t le0 = Wire.read();
    const uint8_t le1 = Wire.read();
    const uint8_t re0 = Wire.read();
    const uint8_t re1 = Wire.read();

    int16_t pwm_l = clampPwm(readInt16LE(l0, l1));
    int16_t pwm_r = clampPwm(readInt16LE(r0, r1));
    uint16_t enc_l = static_cast<uint16_t>(le0 | (le1 << 8));
    uint16_t enc_r = static_cast<uint16_t>(re0 | (re1 << 8));

    // Convert PWM (-1000..1000) to target phase voltage
    target_voltage_left = (float)pwm_l / 1000.0f * POWER_SUPPLY_VOLTAGE;
    target_voltage_right = (float)pwm_r / 1000.0f * POWER_SUPPLY_VOLTAGE;

    // Convert raw 12-bit angle (0..4095) to radians (0..2pi)
    sensor_left.current_angle = (float)(enc_l & 0x0FFF) * _2PI / 4096.0f;
    sensor_right.current_angle = (float)(enc_r & 0x0FFF) * _2PI / 4096.0f;

    last_command_ms = millis();
    command_seen = true;
  }

  while (Wire.available()) {
    (void)Wire.read();
  }
}

void setup() {
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, HIGH);

  // Initialize I2C Slave
  Wire.setSCL(kI2cSclPin);
  Wire.setSDA(kI2cSdaPin);
  Wire.begin(WHEEL_I2C_ADDR);
  Wire.onReceive(onI2cReceive);

  // Initialize SimpleFOC Drivers
  driver_left.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver_left.init();
  motor_left.linkDriver(&driver_left);

  driver_right.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver_right.init();
  motor_right.linkDriver(&driver_right);

  // Link external sensors
  motor_left.linkSensor(&sensor_left);

  motor_right.linkSensor(&sensor_right);

  // Set motor control parameters
  motor_left.controller = MotionControlType::torque;
  motor_right.controller = MotionControlType::torque;

  // Initialize FOC
  motor_left.init();
  motor_left.initFOC();

  motor_right.init();
  motor_right.initFOC();

  last_command_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  if (!command_seen || now - last_command_ms > kCommandTimeoutMs) {
    // Timeout or no command: disable PWM outputs
    motor_left.target = 0.0f;
    motor_right.target = 0.0f;
    digitalWrite(kStatusLedPin, HIGH);
  } else {
    motor_left.target = target_voltage_left;
    motor_right.target = target_voltage_right;
    digitalWrite(kStatusLedPin, LOW);
  }

  // Run SimpleFOC loops
  motor_left.loopFOC();
  motor_right.loopFOC();

  motor_left.move();
  motor_right.move();
}
