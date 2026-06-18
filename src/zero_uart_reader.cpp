#include <Arduino.h>

#ifndef YYT_UART_RX_PIN
#define YYT_UART_RX_PIN 4
#endif

#ifndef YYT_UART_TX_PIN
#define YYT_UART_TX_PIN 5
#endif

static constexpr uint32_t YYT_BAUD = 115200;
static constexpr size_t FLOAT_COUNT = 11;
static constexpr size_t FRAME_SIZE = FLOAT_COUNT * sizeof(float) + 4;
static const uint8_t TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

static uint8_t frame[FRAME_SIZE];
static size_t frame_pos = 0;
static float values[FLOAT_COUNT] = {0};
static bool have_frame = false;
static uint32_t last_frame_ms = 0;
static uint32_t last_print_ms = 0;
static int current_motor_id = 0;
static String serial_line;

static bool tail_matches()
{
  return memcmp(frame + FLOAT_COUNT * sizeof(float), TAIL, sizeof(TAIL)) == 0;
}

static void decode_frame()
{
  memcpy(values, frame, sizeof(values));
  have_frame = true;
  last_frame_ms = millis();
}

static void push_byte(uint8_t b)
{
  if (frame_pos < FRAME_SIZE) {
    frame[frame_pos++] = b;
  }

  if (frame_pos == FRAME_SIZE) {
    if (tail_matches()) {
      decode_frame();
      frame_pos = 0;
      return;
    }

    memmove(frame, frame + 1, FRAME_SIZE - 1);
    frame_pos = FRAME_SIZE - 1;
  }
}

static void print_status()
{
  if (!have_frame) {
    Serial.println("No YYT UART frame yet. Check GND and YYT TX -> ESP32 RX.");
    return;
  }

  Serial.printf(
      "ia=%.3f ib=%.3f ic=%.3f vel_set=%.3f vel=%.3f angle=%.6f full_angle=%.6f Id=%.3f Iq=%.3f pid=%.3f zero_elec=%.6f age_ms=%lu\n",
      values[0],
      values[1],
      values[2],
      values[3],
      values[4],
      values[5],
      values[6],
      values[7],
      values[8],
      values[9],
      values[10],
      (unsigned long)(millis() - last_frame_ms));
}

static void capture_zero()
{
  if (!have_frame) {
    Serial.println("Cannot capture zero: no YYT UART frame yet.");
    return;
  }

  Serial.println("=== ZERO CAPTURE ===");
  if (current_motor_id > 0) {
    Serial.printf("motor_id: %d\n", current_motor_id);
  } else {
    Serial.println("motor_id: not set, use command: id 1");
  }
  Serial.printf("zero_offset_angle_rad: %.6f\n", values[5]);
  Serial.printf("zero_offset_full_angle_rad: %.6f\n", values[6]);
  Serial.printf("calibration_csv: id=%d,angle=%.6f,full_angle=%.6f\n",
                current_motor_id,
                values[5],
                values[6]);
  Serial.println("Use full_angle for leg calibration if it changes continuously across turns.");
  Serial.println("====================");
}

static void print_help()
{
  Serial.println("Commands:");
  Serial.println("  help       : show commands");
  Serial.println("  id <n>     : set the physical YYT board ID you are measuring, e.g. id 1");
  Serial.println("  s          : print latest frame");
  Serial.println("  z          : capture current angle as mechanical zero candidate");
  Serial.println();
}

static void handle_line(String line)
{
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("help")) {
    print_help();
    return;
  }

  int id = 0;
  if (sscanf(line.c_str(), "id %d", &id) == 1) {
    if (id < 1 || id > 8) {
      Serial.println("Bad ID. Use 1..8.");
      return;
    }
    current_motor_id = id;
    Serial.printf("Now measuring physical YYT ID%d. Connect only this board's UART TX to ESP32 RX.\n",
                  current_motor_id);
    return;
  }

  if (line.equalsIgnoreCase("s")) {
    print_status();
    return;
  }

  if (line.equalsIgnoreCase("z")) {
    capture_zero();
    return;
  }

  Serial.println("Unknown command. Type: help");
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial1.begin(YYT_BAUD, SERIAL_8N1, YYT_UART_RX_PIN, YYT_UART_TX_PIN);

  Serial.println();
  Serial.println("YYT UART zero reader");
  Serial.printf("ESP32 RX pin: GPIO%d  <- YYT USART1_TX / PB6\n", YYT_UART_RX_PIN);
  Serial.printf("ESP32 TX pin: GPIO%d  -> YYT USART1_RX / PB7 (optional, not used)\n", YYT_UART_TX_PIN);
  print_help();
}

void loop()
{
  while (Serial1.available() > 0) {
    push_byte((uint8_t)Serial1.read());
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handle_line(serial_line);
      serial_line = "";
    } else if (serial_line.length() < 64) {
      serial_line += c;
    }
  }

  if (millis() - last_print_ms > 1000) {
    last_print_ms = millis();
    if (have_frame) {
      Serial.printf("live angle=%.6f full_angle=%.6f age_ms=%lu\n",
                    values[5],
                    values[6],
                    (unsigned long)(millis() - last_frame_ms));
    } else {
      Serial.println("Waiting for YYT UART frame...");
    }
  }
}
