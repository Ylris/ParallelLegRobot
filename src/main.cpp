#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include "LegConfig.h"
#include "LegKinematics.h"
#include "ImuManager.h"

#ifndef YYT_CAN_RX_PIN
#define YYT_CAN_RX_PIN 7
#endif

#ifndef YYT_CAN_TX_PIN
#define YYT_CAN_TX_PIN 6
#endif

#ifndef YYT_SAFE_MV_LIMIT
#define YYT_SAFE_MV_LIMIT 12000
#endif

#ifndef YYT_HOLD_MV_LIMIT
#define YYT_HOLD_MV_LIMIT 12000
#endif

#ifndef YYT_MIN_TEST_MV
#define YYT_MIN_TEST_MV 0
#endif

#ifndef YYT_DISABLE_I2C
#define YYT_DISABLE_I2C 0
#endif

#ifndef YYT_ALLOW_UNTESTED_DIRS
#define YYT_ALLOW_UNTESTED_DIRS 0
#endif

static constexpr int kLedPin = 10;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kTwoPi = 2.0f * kPi;
static constexpr uint32_t kCommandPeriodMs = 20;
static constexpr uint32_t kSummaryPeriodMs = 1000;
static constexpr uint32_t kImuStreamPeriodMs = 50;
static constexpr uint32_t kOnlineTimeoutMs = 500;
static constexpr uint32_t kWheelStatePeriodMs = 100;
static constexpr int16_t kWheelPwmLimit = 1000;
static constexpr float kStandbyXmm = 0.0f;
static constexpr float kStandbyYmm = -100.0f;
static constexpr float kHeightMinMm = 80.0f;
static constexpr float kHeightMaxMm = 120.0f;
static constexpr float kTargetRampRadPerSec = 0.18f;
static constexpr float kJointSoftLimitRad = 3.40f;
static constexpr float kHoldKpMvPerRad = 12000.0f;
static constexpr float kHoldKdMvPerRadSec = 300.0f;
static constexpr float kHoldStaticBoostMv = 0.0f;
static constexpr float kHoldStaticBoostThresholdRad = 0.08f;
static constexpr float kHoldFullPushThresholdRad = 9.9f;
static constexpr float kStandRelativeStepRad = 0.10f;
static constexpr float kFeedbackJumpRejectRad = 0.80f;
static constexpr int16_t kZeroHoldMvLimit = YYT_HOLD_MV_LIMIT;
static constexpr int16_t kOpenLoopSpinMv = 4321;
static constexpr int16_t kDriveZeroHoldMv = 2345;
static constexpr uint32_t kCanRecoveryCooldownMs = 500;
static constexpr uint32_t kCanTxPauseMs = 500;
static constexpr uint32_t kCanTxQueueHighWater = 8;
static constexpr uint32_t kCanTxErrorRecoveryThreshold = 96;

struct MotorConfig {
  int id;
  const char *name;
  float zero_rad;
  int sign;
  int drive_sign;
};

struct MotorState {
  int32_t raw_angle_mrad = 0;
  int16_t speed_rpm_x10 = 0;
  uint16_t adc4_raw = 0;
  uint32_t last_ms = 0;
  bool feedback_seen = false;
  uint32_t rejected_jumps = 0;
  float angle_rad = 0.0f;
  float joint_rad = 0.0f;
  float speed_rad_s = 0.0f;
};

static constexpr MotorConfig kMotors[] = {
    {1, "left_front_upper", 4.860f, -1, 1},
    {2, "left_rear_lower", 5.553f, 1, -1},
    {5, "right_front_upper", 0.161f, -1, 1},
    {6, "right_rear_lower", 1.991f, 1, -1},
};

static MotorState motor_state[9];
static int16_t command_mv[9] = {};
static int motor_sign[9] = {};
static bool direction_tested[9] = {};
static bool armed = true;
static bool direction_confirmed = true;
static bool height_hold_enabled = false;
static float desired_target[9] = {};
static float ramped_target[9] = {};
static uint32_t last_command_ms = 0;
static uint32_t last_summary_ms = 0;
static uint32_t last_imu_stream_ms = 0;
static uint32_t last_wheel_stream_ms = 0;
static uint32_t last_wheel_state_ms = 0;
static uint32_t last_hold_update_ms = 0;
static uint32_t tx_fail_count = 0;
static uint32_t can_recovery_count = 0;
static uint32_t last_can_recovery_ms = 0;
static uint32_t can_tx_pause_until_ms = 0;
static uint32_t height_hold_start_tx_fail = 0;
static uint32_t height_hold_start_bus_error = 0;
static int16_t active_hold_mv_limit = YYT_HOLD_MV_LIMIT;
static String serial_line;
static LegKinematics kinematics;
static ImuManager imu;
static bool imu_stream_enabled = false;
static bool wheel_armed = false;
static bool wheel_encoder_online = false;
static bool wheel_coprocessor_online = false;
static bool wheel_stream_enabled = false;
static uint16_t wheel_encoder_raw = 0;
static float wheel_angle_rad = 0.0f;
static int16_t wheel_left_pwm = 0;
static int16_t wheel_right_pwm = 0;

static float normalizeRad(float rad) {
  while (rad > kPi) rad -= kTwoPi;
  while (rad < -kPi) rad += kTwoPi;
  return rad;
}

static int16_t clampMv(long mv, int limit = YYT_SAFE_MV_LIMIT) {
  if (mv > limit) return static_cast<int16_t>(limit);
  if (mv < -limit) return static_cast<int16_t>(-limit);
  return static_cast<int16_t>(mv);
}

static int16_t clampTestMv(long mv) {
  const int16_t limited = clampMv(mv);
  if (limited == 0) return 0;
  if (abs(limited) == kDriveZeroHoldMv) return limited;
  if (abs(limited) == kOpenLoopSpinMv) return limited;
  if (abs(limited) < YYT_MIN_TEST_MV) {
    return limited > 0 ? YYT_MIN_TEST_MV : -YYT_MIN_TEST_MV;
  }
  return limited;
}

static const MotorConfig *findMotor(int id) {
  for (const auto &motor : kMotors) {
    if (motor.id == id) return &motor;
  }
  return nullptr;
}

static bool isMotorOnline(int id) {
  return motor_state[id].last_ms != 0 && millis() - motor_state[id].last_ms < kOnlineTimeoutMs;
}

static bool allLegMotorsOnline() {
  for (const auto &motor : kMotors) {
    if (!isMotorOnline(motor.id)) return false;
  }
  return true;
}

static bool anyLegMotorOnline() {
  for (const auto &motor : kMotors) {
    if (isMotorOnline(motor.id)) return true;
  }
  return false;
}

static bool allDirectionsTested() {
  for (const auto &motor : kMotors) {
    if (!direction_tested[motor.id]) return false;
  }
  return true;
}

static void clearCommands() {
  for (int i = 0; i < 9; ++i) command_mv[i] = 0;
}

static bool hasActiveMotorCommand() {
  for (const auto &motor : kMotors) {
    if (command_mv[motor.id] != 0) return true;
  }
  return false;
}

static void stopHeightHold(const char *reason) {
  height_hold_enabled = false;
  clearCommands();
  Serial.printf("height hold stopped: %s\n", reason);
}

static const char *twaiStateName(twai_state_t state) {
  switch (state) {
    case TWAI_STATE_STOPPED:
      return "stopped";
    case TWAI_STATE_RUNNING:
      return "running";
    case TWAI_STATE_BUS_OFF:
      return "bus_off";
    case TWAI_STATE_RECOVERING:
      return "recovering";
    default:
      return "unknown";
  }
}

static void putInt16LE(uint8_t *data, int slot, int16_t value) {
  const int offset = slot * 2;
  data[offset] = static_cast<uint8_t>(value & 0xff);
  data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

static int32_t getInt32LE(const uint8_t *data) {
  return static_cast<int32_t>(static_cast<uint32_t>(data[0]) |
                              (static_cast<uint32_t>(data[1]) << 8) |
                              (static_cast<uint32_t>(data[2]) << 16) |
                              (static_cast<uint32_t>(data[3]) << 24));
}

static int16_t getInt16LE(const uint8_t *data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8));
}

static bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void scanI2cBus() {
  Serial.printf("I2C scan on SDA=GPIO%d SCL=GPIO%d:\n", I2C_SDA_PIN, I2C_SCL_PIN);
  int found = 0;
  for (uint8_t addr = 1; addr < 0x7f; ++addr) {
    if (i2cDevicePresent(addr)) {
      Serial.printf("  found 0x%02X", addr);
      if (addr == MPU6050_I2C_ADDR) Serial.print(" MPU6050");
      if (addr == WHEEL_PWM_COPROCESSOR_I2C_ADDR) Serial.print(" STM32F103-wheel-pwm");
      if (addr == WHEEL_ENCODER_I2C_ADDR) Serial.print(" wheel-encoder");
      Serial.println();
      found++;
    }
  }
  Serial.printf("I2C scan done, %d device(s)\n", found);
}

static bool readWheelEncoder() {
  // Default wheel encoder protocol is AS5600-compatible:
  // register 0x0C = raw angle high bits, 0x0D = raw angle low bits.
  Wire.beginTransmission(WHEEL_ENCODER_I2C_ADDR);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) {
    wheel_encoder_online = false;
    return false;
  }

  const uint8_t got = Wire.requestFrom(WHEEL_ENCODER_I2C_ADDR, static_cast<uint8_t>(2));
  if (got != 2 || Wire.available() < 2) {
    wheel_encoder_online = false;
    return false;
  }

  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();
  wheel_encoder_raw = static_cast<uint16_t>(((high & 0x0f) << 8) | low);
  wheel_angle_rad = static_cast<float>(wheel_encoder_raw) * kTwoPi / 4096.0f;
  wheel_encoder_online = true;
  return true;
}

static bool sendWheelPwmToCoprocessor(int16_t left_pwm, int16_t right_pwm) {
  if (YYT_DISABLE_I2C) {
    wheel_coprocessor_online = false;
    return false;
  }

  left_pwm = constrain(left_pwm, -kWheelPwmLimit, kWheelPwmLimit);
  right_pwm = constrain(right_pwm, -kWheelPwmLimit, kWheelPwmLimit);

  Wire.beginTransmission(WHEEL_PWM_COPROCESSOR_I2C_ADDR);
  Wire.write(0x10);  // command register: set signed wheel PWM pair
  Wire.write(static_cast<uint8_t>(left_pwm & 0xff));
  Wire.write(static_cast<uint8_t>((left_pwm >> 8) & 0xff));
  Wire.write(static_cast<uint8_t>(right_pwm & 0xff));
  Wire.write(static_cast<uint8_t>((right_pwm >> 8) & 0xff));
  const bool ok = Wire.endTransmission() == 0;

  wheel_coprocessor_online = ok;
  if (ok) {
    wheel_left_pwm = left_pwm;
    wheel_right_pwm = right_pwm;
  }
  return ok;
}

static void stopWheelPwm() {
  wheel_left_pwm = 0;
  wheel_right_pwm = 0;
  (void)sendWheelPwmToCoprocessor(0, 0);
}

static void updateWheelDevices() {
  (void)readWheelEncoder();
  wheel_coprocessor_online = i2cDevicePresent(WHEEL_PWM_COPROCESSOR_I2C_ADDR);

  if (!wheel_armed && (wheel_left_pwm != 0 || wheel_right_pwm != 0)) {
    stopWheelPwm();
  }
}

static void printWheelStatus() {
  Serial.printf("wheel: armed=%s f103=%s encoder=%s addr_f103=0x%02X addr_enc=0x%02X pwm_l=%d pwm_r=%d enc_raw=%u angle=%.4f rad\n",
                wheel_armed ? "yes" : "no",
                wheel_coprocessor_online ? "online" : "offline",
                wheel_encoder_online ? "online" : "offline",
                WHEEL_PWM_COPROCESSOR_I2C_ADDR,
                WHEEL_ENCODER_I2C_ADDR,
                wheel_left_pwm,
                wheel_right_pwm,
                wheel_encoder_raw,
                wheel_angle_rad);
}

static bool sendGroup(uint32_t can_id, int first_motor_id) {
  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.data_length_code = 8;
  msg.flags = TWAI_MSG_FLAG_NONE;

  for (int slot = 0; slot < 4; ++slot) {
    const int id = first_motor_id + slot;
    const int16_t mv = armed ? command_mv[id] : 0;
    putInt16LE(msg.data, slot, mv);
  }

  const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
  if (err != ESP_OK) {
    tx_fail_count++;
    return false;
  }
  return true;
}

static void serviceCanRecovery() {
  twai_status_info_t info = {};
  if (twai_get_status_info(&info) != ESP_OK) {
    return;
  }

  const uint32_t now = millis();
  if (info.state == TWAI_STATE_BUS_OFF) {
    if (now - last_can_recovery_ms >= kCanRecoveryCooldownMs) {
      if (height_hold_enabled) {
        stopHeightHold("CAN bus off");
      }
      clearCommands();
      if (twai_initiate_recovery() == ESP_OK) {
        can_recovery_count++;
        last_can_recovery_ms = now;
        can_tx_pause_until_ms = now + kCanTxPauseMs;
      }
    }
    return;
  }

  if (info.state == TWAI_STATE_RECOVERING) {
    can_tx_pause_until_ms = now + kCanTxPauseMs;
    return;
  }

  if (info.state == TWAI_STATE_STOPPED) {
    if (now - last_can_recovery_ms >= kCanRecoveryCooldownMs &&
        twai_start() == ESP_OK) {
      can_recovery_count++;
      last_can_recovery_ms = now;
      can_tx_pause_until_ms = now + kCanTxPauseMs;
    }
    return;
  }

  if (info.state != TWAI_STATE_RUNNING) {
    return;
  }

  const bool idle_offline_bus = !anyLegMotorOnline() && !hasActiveMotorCommand();
  const bool tx_backed_up = info.msgs_to_tx >= kCanTxQueueHighWater ||
                            info.tx_error_counter >= kCanTxErrorRecoveryThreshold;
  if (idle_offline_bus && tx_backed_up) {
    if (now - last_can_recovery_ms >= kCanRecoveryCooldownMs) {
      (void)twai_clear_transmit_queue();
      can_recovery_count++;
      last_can_recovery_ms = now;
    }
    can_tx_pause_until_ms = now + kCanTxPauseMs;
  }
}

static void sendMotorCommands() {
  const uint32_t now = millis();
  if (can_tx_pause_until_ms != 0 &&
      static_cast<int32_t>(now - can_tx_pause_until_ms) < 0) {
    return;
  }

  sendGroup(0x100, 1);
  sendGroup(0x200, 5);
}

static void receiveFeedback() {
  twai_message_t msg = {};
  int frames_processed = 0;
  while (frames_processed < 64 && twai_receive(&msg, 0) == ESP_OK) {
    frames_processed++;
    if (msg.extd || msg.data_length_code < 6) continue;
    if (msg.identifier < 0x101 || msg.identifier > 0x108) continue;

    const int id = static_cast<int>(msg.identifier - 0x100);
    MotorState &state = motor_state[id];
    const MotorConfig *cfg = findMotor(id);
    if (cfg == nullptr) continue;

    const int32_t next_raw_angle_mrad = getInt32LE(&msg.data[0]);
    const float next_angle_rad = next_raw_angle_mrad / 1000.0f;
    const float next_joint_rad = normalizeRad(next_angle_rad - cfg->zero_rad) * motor_sign[id];

    if (state.feedback_seen &&
        fabsf(normalizeRad(next_joint_rad - state.joint_rad)) > kFeedbackJumpRejectRad) {
      state.rejected_jumps++;
      continue;
    }

    state.raw_angle_mrad = next_raw_angle_mrad;
    state.speed_rpm_x10 = getInt16LE(&msg.data[4]);
    state.adc4_raw = static_cast<uint16_t>(getInt16LE(&msg.data[6]));
    state.last_ms = millis();
    state.feedback_seen = true;
    state.angle_rad = next_angle_rad;
    state.joint_rad = next_joint_rad;

    if (cfg != nullptr) {
      const float rpm = state.speed_rpm_x10 / 10.0f;
      state.speed_rad_s = rpm * kTwoPi / 60.0f * motor_sign[id];
    }
  }
}

static bool computeTargetsFromHeight(float height_mm, float target[9]) {
  if (height_mm < kHeightMinMm || height_mm > kHeightMaxMm) return false;

  float base_upper = 0.0f;
  float base_lower = 0.0f;
  float next_upper = 0.0f;
  float next_lower = 0.0f;
  if (!kinematics.inverseKinematics(kStandbyXmm, kStandbyYmm, base_upper, base_lower)) return false;
  if (!kinematics.inverseKinematics(0.0f, -height_mm, next_upper, next_lower)) return false;

  const float upper_delta = normalizeRad(next_upper - base_upper);
  const float lower_delta = normalizeRad(next_lower - base_lower);
  if (fabsf(upper_delta) > kJointSoftLimitRad || fabsf(lower_delta) > kJointSoftLimitRad) return false;

  target[1] = upper_delta;
  target[2] = lower_delta;
  target[5] = upper_delta;
  target[6] = lower_delta;
  return true;
}

static void updateHeightHold() {
  if (!height_hold_enabled) return;

  if (!armed) {
    stopHeightHold("disarmed");
    return;
  }
  if (!direction_confirmed) {
    stopHeightHold("directions not confirmed");
    return;
  }
  bool any_online = false;
  for (const auto &motor : kMotors) {
    if (isMotorOnline(motor.id)) {
      any_online = true;
    }
  }
  if (!any_online) {
    stopHeightHold("all motors offline");
    return;
  }
  if (tx_fail_count != height_hold_start_tx_fail) {
    stopHeightHold("CAN transmit failed during height hold");
    return;
  }

  twai_status_info_t can_status = {};
  if (twai_get_status_info(&can_status) != ESP_OK ||
      can_status.state != TWAI_STATE_RUNNING ||
      can_status.bus_error_count != height_hold_start_bus_error) {
    stopHeightHold("CAN bus error during height hold");
    return;
  }

  const uint32_t now = millis();
  const float dt = last_hold_update_ms == 0 ? 0.02f : (now - last_hold_update_ms) / 1000.0f;
  last_hold_update_ms = now;
  const float step = kTargetRampRadPerSec * dt;

  for (const auto &motor : kMotors) {
    const int id = motor.id;
    if (!isMotorOnline(id)) {
      command_mv[id] = 0;
      continue;
    }
    if (fabsf(motor_state[id].joint_rad) > kJointSoftLimitRad) {
      stopHeightHold("joint soft limit exceeded");
      return;
    }

    const float delta = desired_target[id] - ramped_target[id];
    if (delta > step) {
      ramped_target[id] += step;
    } else if (delta < -step) {
      ramped_target[id] -= step;
    } else {
      ramped_target[id] = desired_target[id];
    }

    const float err = normalizeRad(ramped_target[id] - motor_state[id].joint_rad);
    float mv = kHoldKpMvPerRad * err - kHoldKdMvPerRadSec * motor_state[id].speed_rad_s;
    if (fabsf(err) > kHoldFullPushThresholdRad) {
      mv = err > 0.0f ? active_hold_mv_limit : -active_hold_mv_limit;
    } else if (fabsf(err) > kHoldStaticBoostThresholdRad) {
      mv += err > 0.0f ? kHoldStaticBoostMv : -kHoldStaticBoostMv;
    }
    command_mv[id] = clampMv(lroundf(mv * motor.drive_sign), active_hold_mv_limit);
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("ParallelLegRobot CAN leg controller");
  Serial.println("commands:");
  Serial.println("  help                  show this menu");
  Serial.println("  status                print raw and zeroed joint angles");
  Serial.println("  can                   print ESP32 TWAI/CAN controller status");
  Serial.println("  i2cscan               scan shared I2C bus on SDA GPIO4 / SCL GPIO3");
  Serial.println("  imu                   print MPU6050 pitch/roll/yaw once");
  Serial.println("  imustream <on|off>    stream imu:pitch,roll,yaw for PioPulse/plotters");
  Serial.println("  wheel                 print wheel I2C/PWM/encoder status");
  Serial.println("  wheelarm              allow STM32F103 wheel PWM commands");
  Serial.println("  wheeldisarm           force wheel PWM to 0 and disable wheel commands");
  Serial.println("  wheelpwm <l> <r>      send signed left/right wheel PWM, -1000..1000");
  Serial.println("  wheelstream <on|off>  stream wheel:angle,pwm_l,pwm_r");
  Serial.println("  dirs                  print direction-test checklist and current q");
  Serial.println("  arm                   allow CAN voltage output, still sends 0 mV first");
  Serial.println("  disarm                stop all output and leave safe mode");
  Serial.println("  stop                  set all commands to 0 mV");
  Serial.println("  test <id> <mv> <ms>   pulse one motor, nonzero mv is limited to +/-6000..12000");
  Serial.println("  v <id> <mv>           manual voltage; 2345=drive zero hold, +/-4321=open-loop spin");
  Serial.println("  sign <id> <1|-1>      set runtime joint direction, example: sign 2 -1");
  Serial.println("  invert <id>           flip one runtime joint direction");
  Serial.println("  confirm_dirs          unlock height hold after manual direction check");
  Serial.println("  zero6                 ID6 drive-side zero-position hold; stop/disarm exits");
  Serial.println("  zero <id>             drive-side zero-position hold for specific ID (1|2|5|6)");
  Serial.println("  zero                  slowly return all four joints to q=0 and hold");
  Serial.println("  height <mm>           slow hold, allowed range 80..120, example: height 100");
  Serial.println("  stand                 relative mirrored leg pose from current q, then hold");
  Serial.println("  holdoff               disable height hold, keep armed state");
  Serial.println();
}

static void printCanStatus() {
  twai_status_info_t info = {};
  const esp_err_t err = twai_get_status_info(&info);
  if (err != ESP_OK) {
    Serial.printf("CAN status unavailable: %d\n", err);
    return;
  }

  const uint32_t now = millis();
  const bool tx_paused = can_tx_pause_until_ms != 0 &&
                         static_cast<int32_t>(now - can_tx_pause_until_ms) < 0;
  Serial.printf("CAN state=%s tx_fail=%lu recoveries=%lu tx_pause=%s bus_error=%lu tx_err=%lu rx_err=%lu tx_queue=%lu rx_queue=%lu rx_missed=%lu rx_overrun=%lu arb_lost=%lu\n",
                twaiStateName(info.state),
                static_cast<unsigned long>(tx_fail_count),
                static_cast<unsigned long>(can_recovery_count),
                tx_paused ? "yes" : "no",
                static_cast<unsigned long>(info.bus_error_count),
                static_cast<unsigned long>(info.tx_error_counter),
                static_cast<unsigned long>(info.rx_error_counter),
                static_cast<unsigned long>(info.msgs_to_tx),
                static_cast<unsigned long>(info.msgs_to_rx),
                static_cast<unsigned long>(info.rx_missed_count),
                static_cast<unsigned long>(info.rx_overrun_count),
                static_cast<unsigned long>(info.arb_lost_count));
}

static void printStatus() {
  Serial.printf("armed=%s dirs=%s hold=%s safe=+/-%d mV test_min=%d mV hold=+/-%d mV tx_fail=%lu CAN_RX=GPIO%d CAN_TX=GPIO%d\n",
                armed ? "yes" : "no",
                direction_confirmed ? "confirmed" : "no",
                height_hold_enabled ? "on" : "off",
                YYT_SAFE_MV_LIMIT,
                YYT_MIN_TEST_MV,
                YYT_HOLD_MV_LIMIT,
                static_cast<unsigned long>(tx_fail_count),
                YYT_CAN_RX_PIN,
                YYT_CAN_TX_PIN);

  const uint32_t now = millis();
  for (const auto &motor : kMotors) {
    const int id = motor.id;
    const bool online = isMotorOnline(id);
    const uint32_t age = motor_state[id].last_ms == 0 ? 999999UL : now - motor_state[id].last_ms;
    Serial.printf("  ID%d %-18s %s sign=%+d tested=%s age=%lu ms raw=%.3f rad q=%+.3f rad spd=%+.2f rad/s adc4=%u target=%+.3f cmd=%+d mV\n",
                  id,
                  motor.name,
                  online ? "online " : "offline",
                  motor_sign[id],
                  direction_tested[id] ? "yes" : "no",
                  static_cast<unsigned long>(age),
                  motor_state[id].angle_rad,
                  motor_state[id].joint_rad,
                  motor_state[id].speed_rad_s,
                  motor_state[id].adc4_raw,
                  ramped_target[id],
                  command_mv[id]);
  }
  Serial.printf("  IMU %s stream=%s pitch=%+.2f roll=%+.2f yaw=%+.2f deg\n",
                imu.isConnected() ? "online" : "offline",
                imu_stream_enabled ? "on" : "off",
                imu.getPitch(),
                imu.getRoll(),
                imu.getYaw());
  Serial.printf("  wheel f103=%s encoder=%s wheel_armed=%s pwm_l=%d pwm_r=%d angle=%.4f rad\n",
                wheel_coprocessor_online ? "online" : "offline",
                wheel_encoder_online ? "online" : "offline",
                wheel_armed ? "yes" : "no",
                wheel_left_pwm,
                wheel_right_pwm,
                wheel_angle_rad);
}

static void printImuStatus() {
  if (!imu.isConnected()) {
    Serial.println("IMU offline: MPU6050 not detected on I2C");
    return;
  }
  Serial.printf("IMU pitch=%+.2f roll=%+.2f yaw=%+.2f deg stream=%s\n",
                imu.getPitch(),
                imu.getRoll(),
                imu.getYaw(),
                imu_stream_enabled ? "on" : "off");
}

static void printDirectionCheck() {
  Serial.println("direction check helper:");
  Serial.println("  1) type: arm");
  Serial.println("  2) test each joint with: test <id> 80 100");
  Serial.println("  3) write down dq. Positive mv should move in the expected positive joint direction.");
  Serial.println("  4) only after all four are correct, type: confirm_dirs");
  Serial.println("current q:");
  for (const auto &motor : kMotors) {
    Serial.printf("  ID%d %-18s sign=%+d tested=%s q=%+.3f rad online=%s\n",
                  motor.id,
                  motor.name,
                  motor_sign[motor.id],
                  direction_tested[motor.id] ? "yes" : "no",
                  motor_state[motor.id].joint_rad,
                  isMotorOnline(motor.id) ? "yes" : "no");
  }
}

static void runPulseTest(int id, int mv, int duration_ms) {
  if (findMotor(id) == nullptr) {
    Serial.println("bad id, use one of: 1 2 5 6");
    return;
  }
  if (!armed) {
    Serial.println("refused: type arm first");
    return;
  }

  duration_ms = constrain(duration_ms, 20, 500);
  height_hold_enabled = false;
  clearCommands();

  receiveFeedback();
  const float before_q = motor_state[id].joint_rad;
  const bool was_online = isMotorOnline(id);

  command_mv[id] = clampTestMv(mv);
  Serial.printf("pulse ID%d at %+d mV for %d ms, then auto stop\n", id, command_mv[id], duration_ms);

  const uint32_t start = millis();
  while (millis() - start < static_cast<uint32_t>(duration_ms)) {
    receiveFeedback();
    sendMotorCommands();
    delay(10);
  }

  clearCommands();
  sendMotorCommands();

  const uint32_t settle_start = millis();
  while (millis() - settle_start < 120) {
    receiveFeedback();
    delay(10);
  }

  const float after_q = motor_state[id].joint_rad;
  const float dq = normalizeRad(after_q - before_q);
  direction_tested[id] = was_online && isMotorOnline(id);
  direction_confirmed = false;
  Serial.printf("pulse done, all commands back to 0 mV. ID%d q_before=%+.4f q_after=%+.4f dq=%+.4f rad online_before=%s online_now=%s\n",
                id,
                before_q,
                after_q,
                dq,
                was_online ? "yes" : "no",
                isMotorOnline(id) ? "yes" : "no");
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }
  if (line.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }
  if (line.equalsIgnoreCase("can")) {
    printCanStatus();
    return;
  }
  if (line.equalsIgnoreCase("i2cscan")) {
    scanI2cBus();
    updateWheelDevices();
    return;
  }
  if (line.equalsIgnoreCase("imu")) {
    printImuStatus();
    return;
  }
  if (line.equalsIgnoreCase("imustream on")) {
    imu_stream_enabled = true;
    Serial.println("IMU stream enabled");
    return;
  }
  if (line.equalsIgnoreCase("imustream off")) {
    imu_stream_enabled = false;
    Serial.println("IMU stream disabled");
    return;
  }
  if (line.equalsIgnoreCase("balance")) {
    Serial.println("balance is intentionally disabled in safe bring-up firmware; use imu/imustream first");
    return;
  }
  if (line.equalsIgnoreCase("wheel")) {
    updateWheelDevices();
    printWheelStatus();
    return;
  }
  if (line.equalsIgnoreCase("wheelarm")) {
    wheel_armed = true;
    stopWheelPwm();
    Serial.println("wheel armed: PWM output is allowed, current wheel PWM is 0");
    return;
  }
  if (line.equalsIgnoreCase("wheeldisarm")) {
    wheel_armed = false;
    stopWheelPwm();
    Serial.println("wheel disarmed: PWM forced to 0");
    return;
  }
  if (line.equalsIgnoreCase("wheelstream on")) {
    wheel_stream_enabled = true;
    Serial.println("wheel stream enabled");
    return;
  }
  if (line.equalsIgnoreCase("wheelstream off")) {
    wheel_stream_enabled = false;
    Serial.println("wheel stream disabled");
    return;
  }
  if (line.equalsIgnoreCase("dirs")) {
    printDirectionCheck();
    return;
  }
  if (line.equalsIgnoreCase("arm")) {
    clearCommands();
    armed = true;
    Serial.println("armed: output is enabled, current commands are still 0 mV");
    return;
  }
  if (line.equalsIgnoreCase("disarm")) {
    height_hold_enabled = false;
    clearCommands();
    armed = false;
    sendMotorCommands();
    wheel_armed = false;
    stopWheelPwm();
    Serial.println("disarmed: all outputs forced to 0 mV");
    return;
  }
  if (line.equalsIgnoreCase("stop")) {
    height_hold_enabled = false;
    clearCommands();
    sendMotorCommands();
    stopWheelPwm();
    Serial.println("stopped: all commands set to 0 mV");
    return;
  }
  if (line.equalsIgnoreCase("confirm_dirs")) {
    if (!YYT_ALLOW_UNTESTED_DIRS && !allDirectionsTested()) {
      Serial.println("refused: test ID1, ID2, ID5, and ID6 before confirm_dirs");
      printDirectionCheck();
      return;
    }
    bool any_online = false;
    for (const auto &motor : kMotors) {
      if (isMotorOnline(motor.id)) any_online = true;
    }
    if (!any_online) {
      Serial.println("refused: no leg motors are online");
      return;
    }
    direction_confirmed = true;
    if (YYT_ALLOW_UNTESTED_DIRS && !allDirectionsTested()) {
      Serial.println("direction check bypassed by build flag: height hold is now allowed");
    } else {
      Serial.println("direction check confirmed: height hold is now allowed");
    }
    return;
  }
  int zero_id = 0;
  bool do_drive_zero = false;
  if (line.equalsIgnoreCase("zero6")) {
    zero_id = 6;
    do_drive_zero = true;
  } else if (sscanf(line.c_str(), "zero %d", &zero_id) == 1) {
    if (zero_id == 1 || zero_id == 2 || zero_id == 5 || zero_id == 6) {
      do_drive_zero = true;
    } else {
      Serial.println("refused: invalid joint motor ID for drive-side zero hold, use 1, 2, 5, or 6");
      return;
    }
  }

  if (do_drive_zero) {
    if (!armed) {
      Serial.println("refused: type arm first");
      return;
    }
    if (!isMotorOnline(zero_id)) {
      Serial.printf("refused: ID%d is offline\n", zero_id);
      return;
    }
    if (fabsf(motor_state[zero_id].joint_rad) > kJointSoftLimitRad) {
      Serial.printf("refused: ID%d current joint is outside soft limit\n", zero_id);
      return;
    }
    twai_status_info_t can_status = {};
    if (twai_get_status_info(&can_status) != ESP_OK || can_status.state != TWAI_STATE_RUNNING) {
      Serial.println("refused: CAN controller is not running");
      return;
    }
    height_hold_enabled = false;
    clearCommands();
    desired_target[zero_id] = 0.0f;
    ramped_target[zero_id] = motor_state[zero_id].joint_rad;
    command_mv[zero_id] = kDriveZeroHoldMv;
    Serial.printf("ID%d drive-side zero hold enabled: command %+d mV sentinel, drive target raw %.3f rad, stop/disarm exits\n",
                  zero_id,
                  command_mv[zero_id],
                  findMotor(zero_id)->zero_rad);
    return;
  }
  if (line.equalsIgnoreCase("zero")) {
    if (!armed) {
      Serial.println("refused: type arm first");
      return;
    }
    if (!direction_confirmed) {
      Serial.println("refused: run single-joint tests, then type confirm_dirs");
      return;
    }
    bool any_online = false;
    for (const auto &motor : kMotors) {
      if (isMotorOnline(motor.id)) any_online = true;
    }
    if (!any_online) {
      Serial.println("refused: no leg motors are online");
      return;
    }
    for (const auto &motor : kMotors) {
      if (!isMotorOnline(motor.id)) continue;
      if (fabsf(motor_state[motor.id].joint_rad) > kJointSoftLimitRad) {
        Serial.println("refused: current joint is outside soft limit");
        return;
      }
      desired_target[motor.id] = 0.0f;
      ramped_target[motor.id] = motor_state[motor.id].joint_rad;
    }
    twai_status_info_t can_status = {};
    if (twai_get_status_info(&can_status) != ESP_OK || can_status.state != TWAI_STATE_RUNNING) {
      Serial.println("refused: CAN controller is not running");
      return;
    }
    active_hold_mv_limit = kZeroHoldMvLimit;
    height_hold_start_tx_fail = tx_fail_count;
    height_hold_start_bus_error = can_status.bus_error_count;
    height_hold_enabled = true;
    last_hold_update_ms = millis();
    Serial.printf("zero hold enabled: ramping all online joints to q=0, limit +/- %d mV\n",
                  active_hold_mv_limit);
    return;
  }
  if (line.equalsIgnoreCase("holdoff")) {
    stopHeightHold("user command");
    return;
  }

  int id = 0;
  int mv = 0;
  int ms = 100;
  int sign = 0;
  int left_pwm = 0;
  int right_pwm = 0;
  if (sscanf(line.c_str(), "wheelpwm %d %d", &left_pwm, &right_pwm) == 2) {
    if (!wheel_armed) {
      Serial.println("refused: type wheelarm first");
      return;
    }
    const bool ok = sendWheelPwmToCoprocessor(static_cast<int16_t>(left_pwm),
                                              static_cast<int16_t>(right_pwm));
    Serial.printf("wheel PWM %s: left=%d right=%d\n",
                  ok ? "sent" : "failed",
                  wheel_left_pwm,
                  wheel_right_pwm);
    return;
  }

  if (sscanf(line.c_str(), "test %d %d %d", &id, &mv, &ms) >= 2) {
    runPulseTest(id, mv, ms);
    return;
  }

  if (sscanf(line.c_str(), "sign %d %d", &id, &sign) == 2) {
    if (findMotor(id) == nullptr) {
      Serial.println("bad id, use one of: 1 2 5 6");
      return;
    }
    if (sign != 1 && sign != -1) {
      Serial.println("bad sign, use 1 or -1");
      return;
    }
    height_hold_enabled = false;
    direction_confirmed = false;
    motor_sign[id] = sign;
    direction_tested[id] = false;
    receiveFeedback();
    Serial.printf("ID%d runtime sign set to %+d. Re-test this joint before confirm_dirs.\n", id, motor_sign[id]);
    return;
  }

  if (sscanf(line.c_str(), "invert %d", &id) == 1) {
    if (findMotor(id) == nullptr) {
      Serial.println("bad id, use one of: 1 2 5 6");
      return;
    }
    height_hold_enabled = false;
    direction_confirmed = false;
    motor_sign[id] = -motor_sign[id];
    direction_tested[id] = false;
    receiveFeedback();
    Serial.printf("ID%d runtime sign inverted to %+d. Re-test this joint before confirm_dirs.\n", id, motor_sign[id]);
    return;
  }

  if (sscanf(line.c_str(), "v %d %d", &id, &mv) == 2) {
    if (findMotor(id) == nullptr) {
      Serial.println("bad id, use one of: 1 2 5 6");
      return;
    }
    if (!armed) {
      Serial.println("refused: type arm first");
      return;
    }
    height_hold_enabled = false;
    command_mv[id] = clampTestMv(mv);
    Serial.printf("ID%d command set to %+d mV\n", id, command_mv[id]);
    return;
  }

  float height_mm = 0.0f;
  if (line == "stand") {
    if (!armed) {
      Serial.println("refused: type arm first");
      return;
    }
    if (!direction_confirmed) {
      Serial.println("refused: run single-joint tests, then type confirm_dirs");
      return;
    }
    bool any_online = false;
    for (const auto &motor : kMotors) {
      if (isMotorOnline(motor.id)) any_online = true;
    }
    if (!any_online) {
      Serial.println("refused: no leg motors are online");
      return;
    }

    desired_target[1] = motor_state[1].joint_rad - kStandRelativeStepRad;
    desired_target[2] = motor_state[2].joint_rad + kStandRelativeStepRad;
    desired_target[5] = motor_state[5].joint_rad + kStandRelativeStepRad;
    desired_target[6] = motor_state[6].joint_rad - kStandRelativeStepRad;
    for (const auto &motor : kMotors) {
      if (!isMotorOnline(motor.id)) continue;
      if (fabsf(desired_target[motor.id]) > kJointSoftLimitRad) {
        Serial.println("refused: relative stand target would exceed joint soft limit");
        return;
      }
      ramped_target[motor.id] = motor_state[motor.id].joint_rad;
    }

    twai_status_info_t can_status = {};
    if (twai_get_status_info(&can_status) != ESP_OK || can_status.state != TWAI_STATE_RUNNING) {
      Serial.println("refused: CAN controller is not running");
      return;
    }
    active_hold_mv_limit = YYT_HOLD_MV_LIMIT;
    height_hold_start_tx_fail = tx_fail_count;
    height_hold_start_bus_error = can_status.bus_error_count;
    height_hold_enabled = true;
    last_hold_update_ms = millis();
    Serial.printf("relative stand hold enabled: step %.2f rad from current q\n", kStandRelativeStepRad);
    return;
  }

  if (sscanf(line.c_str(), "height %f", &height_mm) == 1) {
    if (!armed) {
      Serial.println("refused: type arm first");
      return;
    }
    if (!direction_confirmed) {
      Serial.println("refused: run single-joint tests, then type confirm_dirs");
      return;
    }
    bool any_online = false;
    for (const auto &motor : kMotors) {
      if (isMotorOnline(motor.id)) any_online = true;
    }
    if (!any_online) {
      Serial.println("refused: no leg motors are online");
      return;
    }
    if (!computeTargetsFromHeight(height_mm, desired_target)) {
      Serial.println("height IK target unavailable; using relative airborne hold target");
      desired_target[1] = motor_state[1].joint_rad - kStandRelativeStepRad;
      desired_target[2] = motor_state[2].joint_rad + kStandRelativeStepRad;
      desired_target[5] = motor_state[5].joint_rad + kStandRelativeStepRad;
      desired_target[6] = motor_state[6].joint_rad - kStandRelativeStepRad;
    }
    if (YYT_ALLOW_UNTESTED_DIRS) {
      desired_target[1] = motor_state[1].joint_rad - kStandRelativeStepRad;
      desired_target[2] = motor_state[2].joint_rad + kStandRelativeStepRad;
      desired_target[5] = motor_state[5].joint_rad + kStandRelativeStepRad;
      desired_target[6] = motor_state[6].joint_rad - kStandRelativeStepRad;
      Serial.println("height command mapped to relative airborne hold target");
    }
    for (const auto &motor : kMotors) {
      if (!isMotorOnline(motor.id)) continue;
      if (fabsf(desired_target[motor.id]) > kJointSoftLimitRad) {
        Serial.println("refused: height/relative target would exceed joint soft limit");
        return;
      }
    }
    for (const auto &motor : kMotors) {
      if (!isMotorOnline(motor.id)) continue;
      ramped_target[motor.id] = motor_state[motor.id].joint_rad;
    }
    twai_status_info_t can_status = {};
    if (twai_get_status_info(&can_status) != ESP_OK || can_status.state != TWAI_STATE_RUNNING) {
      Serial.println("refused: CAN controller is not running");
      return;
    }
    active_hold_mv_limit = YYT_HOLD_MV_LIMIT;
    height_hold_start_tx_fail = tx_fail_count;
    height_hold_start_bus_error = can_status.bus_error_count;
    height_hold_enabled = true;
    last_hold_update_ms = millis();
    Serial.printf("height hold enabled: target %.1f mm, ramping slowly\n", height_mm);
    return;
  }

  Serial.println("unknown command, type help");
}

static bool setupCan() {
  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(YYT_CAN_TX_PIN),
                                  static_cast<gpio_num_t>(YYT_CAN_RX_PIN),
                                  TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 10;
  general_config.rx_queue_len = 30;

  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&general_config, &timing_config, &filter_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", err);
    return false;
  }

  return true;
}

void setup() {
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);

  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("ParallelLegRobot safe CAN leg controller");
  Serial.println("reference: foc-wheel-legged-robot control layering, adapted for 4 airborne leg motors");
  Serial.printf("CAN RX GPIO%d, CAN TX GPIO%d, 1 Mbps\n", YYT_CAN_RX_PIN, YYT_CAN_TX_PIN);
  Serial.printf("shared I2C SDA GPIO%d, SCL GPIO%d, %lu Hz\n",
                I2C_SDA_PIN,
                I2C_SCL_PIN,
                static_cast<unsigned long>(I2C_BUS_HZ));
  Serial.println("boot policy: armed by default, waiting for all motors to activate zero hold");

  for (const auto &motor : kMotors) {
    motor_sign[motor.id] = motor.sign;
  }

  clearCommands();
  if (setupCan()) {
    Serial.println("CAN started");
  } else {
    Serial.println("CAN start failed; check ESP32C3 pins and transceiver wiring");
  }

  if (YYT_DISABLE_I2C) {
    Serial.println("I2C disabled by build flag; leg CAN bring-up only");
  } else {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_BUS_HZ);
    Wire.setTimeOut(20);
    Serial.println("I2C started. Type i2cscan to scan devices.");

    if (!imu.begin()) {
      Serial.println("IMU init skipped/failed; motor CAN bring-up can still continue");
    }
    updateWheelDevices();
    stopWheelPwm();
  }

  if (computeTargetsFromHeight(100.0f, desired_target)) {
    for (const auto &motor : kMotors) {
      ramped_target[motor.id] = desired_target[motor.id];
    }
  }
  printHelp();
}

void loop() {
  receiveFeedback();

  static bool auto_zero_triggered = false;
  if (!auto_zero_triggered && allLegMotorsOnline()) {
    auto_zero_triggered = true;
    for (const auto &motor : kMotors) {
      desired_target[motor.id] = 0.0f;
      ramped_target[motor.id] = motor_state[motor.id].joint_rad;
    }
    active_hold_mv_limit = kZeroHoldMvLimit;
    height_hold_start_tx_fail = tx_fail_count;
    twai_status_info_t can_status = {};
    if (twai_get_status_info(&can_status) == ESP_OK) {
      height_hold_start_bus_error = can_status.bus_error_count;
    }
    height_hold_enabled = true;
    last_hold_update_ms = millis();
    Serial.println("Auto-trigger zero hold: all leg motors online, ramping to q=0");
  }

  if (!YYT_DISABLE_I2C) {
    imu.update();
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      handleCommand(serial_line);
      serial_line = "";
    } else if (serial_line.length() < 120) {
      serial_line += c;
    }
  }

  const uint32_t now = millis();
  if (now - last_command_ms >= kCommandPeriodMs) {
    last_command_ms = now;
    serviceCanRecovery();
    updateHeightHold();
    sendMotorCommands();
    // Breathing LED using sine wave. Faster when armed.
    float speed = armed ? 0.006f : 0.002f;
    float breathingVal = sin(now * speed);
    int brightness = (int)(127.5f * (1.0f + breathingVal));
    analogWrite(kLedPin, brightness);
  }

  if (!YYT_DISABLE_I2C && now - last_wheel_state_ms >= kWheelStatePeriodMs) {
    last_wheel_state_ms = now;
    updateWheelDevices();
  }

  if (!YYT_DISABLE_I2C && imu_stream_enabled && imu.isConnected() && now - last_imu_stream_ms >= kImuStreamPeriodMs) {
    last_imu_stream_ms = now;
    Serial.printf("imu:%.2f,%.2f,%.2f,0.0,0.0,0.0\n",
                  imu.getPitch(),
                  imu.getRoll(),
                  imu.getYaw());
  }

  if (wheel_stream_enabled && now - last_wheel_stream_ms >= kImuStreamPeriodMs) {
    last_wheel_stream_ms = now;
    Serial.printf("wheel:%.4f,%d,%d,%s,%s\n",
                  wheel_angle_rad,
                  wheel_left_pwm,
                  wheel_right_pwm,
                  wheel_coprocessor_online ? "f103_online" : "f103_offline",
                  wheel_encoder_online ? "enc_online" : "enc_offline");
  }

  if (now - last_summary_ms >= kSummaryPeriodMs) {
    last_summary_ms = now;
    Serial.printf("online: ID1=%s ID2=%s ID5=%s ID6=%s armed=%s hold=%s wheel_f103=%s wheel_enc=%s\n",
                  isMotorOnline(1) ? "yes" : "no",
                  isMotorOnline(2) ? "yes" : "no",
                  isMotorOnline(5) ? "yes" : "no",
                  isMotorOnline(6) ? "yes" : "no",
                  armed ? "yes" : "no",
                  height_hold_enabled ? "on" : "off",
                  wheel_coprocessor_online ? "yes" : "no",
                  wheel_encoder_online ? "yes" : "no");
  }
}
