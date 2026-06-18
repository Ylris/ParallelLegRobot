#include <Arduino.h>
#include "driver/twai.h"

#ifndef YYT_CAN_RX_PIN
#define YYT_CAN_RX_PIN 3
#endif

#ifndef YYT_CAN_TX_PIN
#define YYT_CAN_TX_PIN 4
#endif

#ifndef YYT_ZERO_TX_ENABLE
#define YYT_ZERO_TX_ENABLE 1
#endif

struct MotorFeedback {
  int32_t angle_mrad = 0;
  int16_t speed_rpm_x10 = 0;
  uint32_t last_ms = 0;
};

static constexpr int kLedPin = 10;
static constexpr int kIds[] = {1, 2, 5, 6};
static constexpr uint32_t kSendZeroPeriodMs = 20;
static constexpr uint32_t kPrintPeriodMs = 1000;

static MotorFeedback feedback[9];
static String serial_line;
static uint32_t last_zero_send_ms = 0;
static uint32_t last_print_ms = 0;
static uint32_t tx_ok_count = 0;
static uint32_t tx_fail_count = 0;
static uint32_t rx_total_count = 0;
static uint32_t rx_yyt_count = 0;
static uint32_t last_rx_id = 0;
static uint8_t last_rx_dlc = 0;
static bool last_rx_extended = false;

static void putInt16LE(uint8_t *data, int slot, int16_t value) {
  const int i = slot * 2;
  data[i] = static_cast<uint8_t>(value & 0xff);
  data[i + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

static void sendZeroGroup(uint32_t can_id) {
  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.data_length_code = 8;
  for (int slot = 0; slot < 4; ++slot) {
    putInt16LE(msg.data, slot, 0);
  }
  if (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    ++tx_ok_count;
  } else {
    ++tx_fail_count;
  }
}

static void sendZeroCommands() {
  sendZeroGroup(0x100);
  sendZeroGroup(0x200);
}

static int32_t getInt32LE(const uint8_t *data) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(data[0]) |
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
    ++rx_total_count;
    last_rx_id = msg.identifier;
    last_rx_dlc = msg.data_length_code;
    last_rx_extended = msg.extd;

    if (msg.extd || msg.data_length_code < 6) continue;
    if (msg.identifier < 0x101 || msg.identifier > 0x108) continue;

    const int id = static_cast<int>(msg.identifier - 0x100);
    ++rx_yyt_count;
    feedback[id].angle_mrad = getInt32LE(&msg.data[0]);
    feedback[id].speed_rpm_x10 = getInt16LE(&msg.data[4]);
    feedback[id].last_ms = millis();
  }
}

static bool isOnline(int id) {
  return feedback[id].last_ms != 0 && millis() - feedback[id].last_ms < 500;
}

static void printStatus() {
  twai_status_info_t twai_status = {};
  if (twai_get_status_info(&twai_status) == ESP_OK) {
    Serial.printf("twai: state=%d tx_error=%lu rx_error=%lu msgs_to_tx=%lu msgs_to_rx=%lu tx_failed=%lu bus_error=%lu\n",
                  static_cast<int>(twai_status.state),
                  static_cast<unsigned long>(twai_status.tx_error_counter),
                  static_cast<unsigned long>(twai_status.rx_error_counter),
                  static_cast<unsigned long>(twai_status.msgs_to_tx),
                  static_cast<unsigned long>(twai_status.msgs_to_rx),
                  static_cast<unsigned long>(twai_status.tx_failed_count),
                  static_cast<unsigned long>(twai_status.bus_error_count));
  }
  Serial.printf("diag: tx_ok=%lu tx_fail=%lu rx_total=%lu rx_yyt=%lu last_rx=%s0x%03lX dlc=%u\n",
                static_cast<unsigned long>(tx_ok_count),
                static_cast<unsigned long>(tx_fail_count),
                static_cast<unsigned long>(rx_total_count),
                static_cast<unsigned long>(rx_yyt_count),
                last_rx_extended ? "ext:" : "std:",
                static_cast<unsigned long>(last_rx_id),
                static_cast<unsigned>(last_rx_dlc));
  const uint32_t now = millis();
  Serial.println("id,online,angle_rad,speed_rpm,age_ms");
  for (int id : kIds) {
    const uint32_t age = feedback[id].last_ms == 0 ? 999999UL : now - feedback[id].last_ms;
    Serial.printf("%d,%s,%.6f,%.1f,%lu\n",
                  id,
                  isOnline(id) ? "yes" : "no",
                  feedback[id].angle_mrad / 1000.0f,
                  feedback[id].speed_rpm_x10 / 10.0f,
                  static_cast<unsigned long>(age));
  }
}

static void captureZeros() {
  Serial.println("=== CAN ZERO CAPTURE ===");
  Serial.println("Copy these four lines to me:");
  for (int id : kIds) {
    Serial.printf("calibration_csv: id=%d,online=%s,angle=%.6f,speed=%.1f\n",
                  id,
                  isOnline(id) ? "yes" : "no",
                  feedback[id].angle_mrad / 1000.0f,
                  feedback[id].speed_rpm_x10 / 10.0f);
  }
  Serial.println("========================");
}

static void printHelp() {
  Serial.println();
  Serial.println("YYT CAN zero calibrator");
  Serial.println("This firmware reads YYT feedback for zero calibration.");
#if YYT_ZERO_TX_ENABLE
  Serial.println("TX mode: sending only 0 mV commands.");
#else
  Serial.println("TX mode: disabled, receiving feedback only.");
#endif
  Serial.println("Commands:");
  Serial.println("  help    show this menu");
  Serial.println("  status  print ID1/2/5/6 current angles");
  Serial.println("  z       capture current ID1/2/5/6 angles as zero candidates");
  Serial.println();
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.equalsIgnoreCase("help")) {
    printHelp();
  } else if (line.equalsIgnoreCase("status") || line.equalsIgnoreCase("s")) {
    printStatus();
  } else if (line.equalsIgnoreCase("z")) {
    captureZeros();
  } else {
    Serial.println("Unknown command. Type: help");
  }
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
  Serial.println("ParallelLegRobot YYT CAN zero calibrator");
  Serial.printf("CAN RX GPIO%d, CAN TX GPIO%d, baud 1Mbps\n", YYT_CAN_RX_PIN, YYT_CAN_TX_PIN);

  if (setupCan()) {
#if YYT_ZERO_TX_ENABLE
    Serial.println("CAN started. Safe mode: sending only 0 mV.");
#else
    Serial.println("CAN started. Listen-for-feedback mode: no motor commands sent.");
#endif
  }
  printHelp();
}

void loop() {
  receiveFeedback();

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      handleLine(serial_line);
      serial_line = "";
    } else if (serial_line.length() < 64) {
      serial_line += c;
    }
  }

  const uint32_t now = millis();
  if (YYT_ZERO_TX_ENABLE && now - last_zero_send_ms >= kSendZeroPeriodMs) {
    last_zero_send_ms = now;
    sendZeroCommands();
    digitalWrite(kLedPin, (now / 1000) & 1);
  }

  if (now - last_print_ms >= kPrintPeriodMs) {
    last_print_ms = now;
    Serial.print("online: ");
    for (int id : kIds) {
      Serial.printf("ID%d=%s ", id, isOnline(id) ? "yes" : "no");
    }
    Serial.println();
  }
}
