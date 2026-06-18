#include <Arduino.h>
#include "driver/twai.h"

#ifndef YYT_CAN_RX_PIN
#define YYT_CAN_RX_PIN 3
#endif

#ifndef YYT_CAN_TX_PIN
#define YYT_CAN_TX_PIN 4
#endif

#ifndef YYT_SAFE_MV_LIMIT
#define YYT_SAFE_MV_LIMIT 500
#endif

struct MotorFeedback {
  int32_t angle_mrad = 0;
  int16_t speed_rpm_x10 = 0;
  uint32_t last_ms = 0;
};

static constexpr int kLedPin = 10;
static constexpr uint32_t kCmdPeriodMs = 20;
static constexpr uint32_t kPrintPeriodMs = 250;
static constexpr int kFirstId = 1;
static constexpr int kLastId = 8;
static constexpr int kLegMotorIds[] = {1, 2, 5, 6};

static int16_t command_mv[kLastId + 1] = {};
static MotorFeedback feedback[kLastId + 1];
static bool armed = false;
static uint32_t last_cmd_ms = 0;
static uint32_t last_print_ms = 0;
static String serial_line;

static int16_t clampMv(long mv) {
  if (mv > YYT_SAFE_MV_LIMIT) return YYT_SAFE_MV_LIMIT;
  if (mv < -YYT_SAFE_MV_LIMIT) return -YYT_SAFE_MV_LIMIT;
  return static_cast<int16_t>(mv);
}

static bool isLegMotorId(int id) {
  for (int leg_id : kLegMotorIds) {
    if (id == leg_id) return true;
  }
  return false;
}

static void clearCommands() {
  for (int id = kFirstId; id <= kLastId; ++id) {
    command_mv[id] = 0;
  }
}

static void putInt16LE(uint8_t *data, int slot, int16_t value) {
  const int i = slot * 2;
  data[i] = static_cast<uint8_t>(value & 0xff);
  data[i + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

static bool sendGroup(uint32_t can_id, int first_motor_id) {
  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.data_length_code = 8;
  msg.flags = TWAI_MSG_FLAG_NONE;

  for (int slot = 0; slot < 4; ++slot) {
    const int motor_id = first_motor_id + slot;
    const int16_t mv = armed ? command_mv[motor_id] : 0;
    putInt16LE(msg.data, slot, mv);
  }

  const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  return err == ESP_OK;
}

static void sendMotorCommands() {
  sendGroup(0x100, 1);
  sendGroup(0x200, 5);
}

static int32_t getInt32LE(const uint8_t *data) {
  return static_cast<int32_t>(
      (static_cast<uint32_t>(data[0])) |
      (static_cast<uint32_t>(data[1]) << 8) |
      (static_cast<uint32_t>(data[2]) << 16) |
      (static_cast<uint32_t>(data[3]) << 24));
}

static int16_t getInt16LE(const uint8_t *data) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
}

static void receiveFeedback() {
  twai_message_t msg = {};
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.extd || msg.data_length_code < 6) continue;
    if (msg.identifier < 0x101 || msg.identifier > 0x108) continue;

    const int id = static_cast<int>(msg.identifier - 0x100);
    feedback[id].angle_mrad = getInt32LE(&msg.data[0]);
    feedback[id].speed_rpm_x10 = getInt16LE(&msg.data[4]);
    feedback[id].last_ms = millis();
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("YYT CAN master commands:");
  Serial.println("  help              show this menu");
  Serial.println("  status            print angles, speeds, and commanded mV");
  Serial.println("  arm               enable periodic CAN output");
  Serial.println("  disarm            send zeros and ignore voltage commands");
  Serial.println("  stop              set all motor commands to 0 mV");
  Serial.println("  v <id> <mv>       set one leg motor voltage, id 1/2/5/6, mv limited by safety");
  Serial.println("  all <mv>          set all leg motor voltages, mv limited by safety");
  Serial.println("  test <id> <mv>    200 ms pulse, then auto stop");
  Serial.println();
}

static void printStatus() {
  Serial.printf("armed=%s, safe_limit=%d mV, can_rx=%d, can_tx=%d\n",
                armed ? "yes" : "no", YYT_SAFE_MV_LIMIT, YYT_CAN_RX_PIN,
                YYT_CAN_TX_PIN);

  const uint32_t now = millis();
  for (int id : kLegMotorIds) {
    const bool online = feedback[id].last_ms != 0 && now - feedback[id].last_ms < 500;
    Serial.printf("  id%d cmd=%d mV angle=%.3f rad speed=%.1f rpm %s age=%lu ms\n",
                  id,
                  command_mv[id],
                  feedback[id].angle_mrad / 1000.0f,
                  feedback[id].speed_rpm_x10 / 10.0f,
                  online ? "online" : "offline",
                  feedback[id].last_ms == 0 ? 999999UL
                                             : static_cast<unsigned long>(now - feedback[id].last_ms));
  }
}

static void runPulseTest(int id, int mv) {
  if (!isLegMotorId(id)) {
    Serial.println("bad id, use one of: 1 2 5 6");
    return;
  }

  armed = true;
  clearCommands();
  command_mv[id] = clampMv(mv);
  Serial.printf("pulse id%d at %d mV for 200 ms\n", id, command_mv[id]);

  const uint32_t start = millis();
  while (millis() - start < 200) {
    receiveFeedback();
    sendMotorCommands();
    delay(20);
  }

  clearCommands();
  sendMotorCommands();
  Serial.println("pulse done, all commands back to 0 mV");
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
  if (line.equalsIgnoreCase("arm")) {
    armed = true;
    Serial.println("armed: CAN output enabled");
    return;
  }
  if (line.equalsIgnoreCase("disarm")) {
    clearCommands();
    armed = false;
    sendMotorCommands();
    Serial.println("disarmed: commands are zero");
    return;
  }
  if (line.equalsIgnoreCase("stop")) {
    clearCommands();
    sendMotorCommands();
    Serial.println("all commands set to 0 mV");
    return;
  }

  int id = 0;
  int mv = 0;
  if (sscanf(line.c_str(), "v %d %d", &id, &mv) == 2) {
    if (!isLegMotorId(id)) {
      Serial.println("bad id, use one of: 1 2 5 6");
      return;
    }
    command_mv[id] = clampMv(mv);
    Serial.printf("id%d command set to %d mV\n", id, command_mv[id]);
    return;
  }

  if (sscanf(line.c_str(), "all %d", &mv) == 1) {
    const int16_t safe_mv = clampMv(mv);
    for (int motor_id : kLegMotorIds) {
      command_mv[motor_id] = safe_mv;
    }
    Serial.printf("all leg motor commands set to %d mV\n", safe_mv);
    return;
  }

  if (sscanf(line.c_str(), "test %d %d", &id, &mv) == 2) {
    runPulseTest(id, mv);
    return;
  }

  Serial.println("unknown command, type: help");
}

static bool setupCan() {
  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(YYT_CAN_TX_PIN),
                                  static_cast<gpio_num_t>(YYT_CAN_RX_PIN),
                                  TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 10;
  general_config.rx_queue_len = 20;

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
  Serial.println("ParallelLegRobot YYT CAN master");
  Serial.printf("CAN RX GPIO%d, CAN TX GPIO%d, safety limit +/- %d mV\n",
                YYT_CAN_RX_PIN, YYT_CAN_TX_PIN, YYT_SAFE_MV_LIMIT);

  clearCommands();
  if (setupCan()) {
    Serial.println("CAN started at 1 Mbps");
  } else {
    Serial.println("CAN start failed; check pins/build flags");
  }

  printHelp();
}

void loop() {
  receiveFeedback();

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      handleCommand(serial_line);
      serial_line = "";
    } else if (serial_line.length() < 96) {
      serial_line += c;
    }
  }

  const uint32_t now = millis();
  if (now - last_cmd_ms >= kCmdPeriodMs) {
    last_cmd_ms = now;
    sendMotorCommands();
    // Breathing LED using sine wave. Faster when armed.
    float speed = armed ? 0.006f : 0.002f;
    float breathingVal = sin(now * speed);
    int brightness = (int)(127.5f * (1.0f + breathingVal));
    analogWrite(kLedPin, brightness);
  }

  if (now - last_print_ms >= kPrintPeriodMs) {
    last_print_ms = now;
    receiveFeedback();
  }
}
