#include "drive_uart_debug.h"

#include "Data.h"
#include "FOC.h"
#include "Motor.h"
#include "mt6816.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef DRIVE_ID
#define DRIVE_ID 0
#endif

#ifndef DRIVE_ZERO_RAD
#if DRIVE_ID == 1
#define DRIVE_ZERO_RAD 4.860f
#elif DRIVE_ID == 2
#define DRIVE_ZERO_RAD 5.553f
#elif DRIVE_ID == 5
#define DRIVE_ZERO_RAD 0.161f
#elif DRIVE_ID == 6
#define DRIVE_ZERO_RAD 1.991f
#else
#define DRIVE_ZERO_RAD 0.0f
#endif
#endif

#define DEBUG_MODE_OFF 0
#define DEBUG_MODE_VOLTAGE 1
#define DEBUG_MODE_HOLD 2
#define DEBUG_MODE_PHASE 3

#define DEBUG_MAX_VOLTAGE_V 6.0f

#define SWD_CMD_NONE 0U
#define SWD_CMD_STOP 1U
#define SWD_CMD_ARM 2U
#define SWD_CMD_PHASE_PULSE 3U
#define SWD_CMD_Q_PULSE 4U
#define SWD_CMD_HOLD_CURRENT 5U

static uint8_t *debug_rx_buffer = NULL;
static uint16_t debug_rx_buffer_len = 0;
static char debug_line[96];
static char debug_pending_line[96];
static uint8_t debug_line_len = 0;
static volatile uint8_t debug_line_pending = 0;

static uint8_t debug_armed = 0;
static uint8_t debug_mode = DEBUG_MODE_OFF;
static uint8_t debug_telemetry = 0;
static uint8_t debug_print_help = 0;
static int debug_sign = -1;
static float debug_voltage_v = 0.0f;
static float debug_phase_angle_el = 0.0f;
static float debug_target_rad = DRIVE_ZERO_RAD;
static float debug_kp_v = 1.0f;
static float debug_ki_v = 0.0f;
static float debug_kd_v = 0.05f;
static float debug_limit_v = 0.8f;
static float debug_integral_v = 0.0f;
static float debug_last_output_v = 0.0f;
static uint32_t debug_last_us = 0;
static uint32_t debug_pulse_end_ms = 0;
static uint32_t debug_last_telemetry_ms = 0;

volatile uint32_t yyt_swd_magic = 0x59595435U;
volatile uint32_t yyt_swd_cmd = SWD_CMD_NONE;
volatile int32_t yyt_swd_arg0 = 0;
volatile int32_t yyt_swd_arg1 = 0;
volatile int32_t yyt_swd_arg2 = 0;
volatile uint32_t yyt_swd_ack = 0;
volatile int32_t yyt_swd_angle_mrad = 0;
volatile int32_t yyt_swd_full_mrad = 0;
volatile int32_t yyt_swd_vel_mrad_s = 0;
volatile int32_t yyt_swd_vbus_mV = 0;
volatile int32_t yyt_swd_iq_mA = 0;
volatile int32_t yyt_swd_out_mV = 0;
volatile int32_t yyt_swd_zero_el_mrad = 0;
volatile uint32_t yyt_swd_mode = DEBUG_MODE_OFF;
volatile uint32_t yyt_swd_armed = 0;

static void debug_write(const char *text)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 30);
}

static void debug_printf_i(const char *prefix, int32_t value, const char *suffix)
{
    char out[64];
    snprintf(out, sizeof(out), "%s%ld%s", prefix, (long)value, suffix);
    debug_write(out);
}

static float clamp_float(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int32_t to_milli(float value)
{
    return (int32_t)lroundf(value * 1000.0f);
}

static float from_milli(int32_t value)
{
    return (float)value * 0.001f;
}

static float normalize_signed_angle(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

static void debug_stop_output(void)
{
    debug_mode = DEBUG_MODE_OFF;
    debug_voltage_v = 0.0f;
    debug_phase_angle_el = 0.0f;
    debug_integral_v = 0.0f;
    debug_last_output_v = 0.0f;
    debug_pulse_end_ms = 0;
    mode = -1;
    setPhaseVoltage(0.0f, 0.0f, _electricalAngle());
}

static void debug_update_swd_status(void)
{
    yyt_swd_angle_mrad = to_milli(sensor_direction * MT6816_Get_AngleData());
    yyt_swd_full_mrad = to_milli(sensor_direction * MT6816_Get_FullAngleData());
    yyt_swd_vel_mrad_s = to_milli(sensor_direction * MT6816_Get_Velocity_L());
    yyt_swd_vbus_mV = to_milli(vbus);
    yyt_swd_iq_mA = to_milli(Iq);
    yyt_swd_out_mV = to_milli(debug_last_output_v);
    yyt_swd_zero_el_mrad = to_milli(zero_electric_angle);
    yyt_swd_mode = debug_mode;
    yyt_swd_armed = debug_armed;
}

static void debug_handle_swd_command(void)
{
    const uint32_t cmd = yyt_swd_cmd;
    if (cmd == SWD_CMD_NONE)
    {
        return;
    }

    yyt_swd_cmd = SWD_CMD_NONE;
    yyt_swd_ack = cmd;

    if (cmd == SWD_CMD_STOP)
    {
        debug_stop_output();
        debug_armed = 0;
        return;
    }
    if (cmd == SWD_CMD_ARM)
    {
        debug_stop_output();
        debug_armed = 1;
        return;
    }
    if (cmd == SWD_CMD_PHASE_PULSE)
    {
        debug_armed = 1;
        debug_phase_angle_el = from_milli(yyt_swd_arg0);
        debug_voltage_v = clamp_float(from_milli(yyt_swd_arg1), DEBUG_MAX_VOLTAGE_V);
        uint32_t duration_ms = (uint32_t)yyt_swd_arg2;
        if (duration_ms < 20U) duration_ms = 20U;
        if (duration_ms > 800U) duration_ms = 800U;
        debug_mode = DEBUG_MODE_PHASE;
        debug_pulse_end_ms = HAL_GetTick() + duration_ms;
        return;
    }
    if (cmd == SWD_CMD_Q_PULSE)
    {
        debug_armed = 1;
        debug_voltage_v = clamp_float(from_milli(yyt_swd_arg0), DEBUG_MAX_VOLTAGE_V);
        uint32_t duration_ms = (uint32_t)yyt_swd_arg1;
        if (duration_ms < 20U) duration_ms = 20U;
        if (duration_ms > 800U) duration_ms = 800U;
        debug_mode = DEBUG_MODE_VOLTAGE;
        debug_pulse_end_ms = HAL_GetTick() + duration_ms;
        return;
    }
    if (cmd == SWD_CMD_HOLD_CURRENT)
    {
        debug_armed = 1;
        debug_target_rad = sensor_direction * MT6816_Get_AngleData();
        debug_integral_v = 0.0f;
        debug_last_us = 0;
        debug_mode = DEBUG_MODE_HOLD;
        return;
    }
}

static void debug_print_status(void)
{
    const float angle = sensor_direction * MT6816_Get_AngleData();
    const float full = sensor_direction * MT6816_Get_FullAngleData();
    const float vel = sensor_direction * MT6816_Get_Velocity_L();
    const float err = normalize_signed_angle(debug_target_rad - angle);
    char out[240];

    snprintf(out, sizeof(out),
             "st id=%d armed=%u mode=%u sign=%d angle_mrad=%ld full_mrad=%ld "
             "vel_mrad_s=%ld target_mrad=%ld err_mrad=%ld out_mV=%ld "
             "kp=%ld ki=%ld kd=%ld limit_mV=%ld vbus_mV=%ld iq_mA=%ld\r\n",
             DRIVE_ID,
             (unsigned)debug_armed,
             (unsigned)debug_mode,
             debug_sign,
             (long)to_milli(angle),
             (long)to_milli(full),
             (long)to_milli(vel),
             (long)to_milli(debug_target_rad),
             (long)to_milli(err),
             (long)to_milli(debug_last_output_v),
             (long)to_milli(debug_kp_v),
             (long)to_milli(debug_ki_v),
             (long)to_milli(debug_kd_v),
             (long)to_milli(debug_limit_v),
             (long)to_milli(vbus),
             (long)to_milli(Iq));
    debug_write(out);
}

static void debug_print_help_text(void)
{
    debug_write("\r\nYYT drive UART debug, ASCII 115200 8N1\r\n");
    debug_write("Units: pulse/v/limit use mV, target uses raw single-turn mrad, pid uses mV/rad.\r\n");
    debug_write("Commands:\r\n");
    debug_write("  help\r\n");
    debug_write("  status\r\n");
    debug_write("  tele on|off\r\n");
    debug_write("  arm\r\n");
    debug_write("  disarm\r\n");
    debug_write("  stop\r\n");
    debug_write("  pulse <mV> <ms>        example: pulse 300 50\r\n");
    debug_write("  v <mV>                 manual q-axis voltage, example: v -300\r\n");
    debug_write("  target <mrad>          example: target 161\r\n");
    debug_write("  hold current           hold current raw single-turn angle\r\n");
    debug_write("  holdzero               target DRIVE_ZERO_RAD and hold\r\n");
    debug_write("  hold <mrad>            target raw single-turn mrad and hold\r\n");
    debug_write("  pid <kp> <ki> <kd> <limit> [sign]\r\n");
    debug_write("  sign <1|-1>\r\n");
    debug_write("  cal                    rerun motor electrical alignment\r\n\r\n");
}

static void debug_start_rx(void)
{
    if (debug_rx_buffer == NULL || debug_rx_buffer_len == 0U) return;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, debug_rx_buffer, debug_rx_buffer_len);
    if (huart1.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

static void debug_handle_command(char *line)
{
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    int e = 0;

    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    if (strcmp(line, "help") == 0)
    {
        debug_print_help_text();
        return;
    }
    if (strcmp(line, "status") == 0)
    {
        debug_print_status();
        return;
    }
    if (strcmp(line, "tele on") == 0)
    {
        debug_telemetry = 1;
        debug_write("telemetry on\r\n");
        return;
    }
    if (strcmp(line, "tele off") == 0)
    {
        debug_telemetry = 0;
        debug_write("telemetry off\r\n");
        return;
    }
    if (strcmp(line, "arm") == 0)
    {
        debug_stop_output();
        debug_armed = 1;
        debug_write("armed, output still 0 mV\r\n");
        return;
    }
    if (strcmp(line, "disarm") == 0)
    {
        debug_stop_output();
        debug_armed = 0;
        debug_write("disarmed, output forced to 0 mV\r\n");
        return;
    }
    if (strcmp(line, "stop") == 0)
    {
        debug_stop_output();
        debug_write("stopped, output forced to 0 mV\r\n");
        return;
    }
    if (strcmp(line, "hold current") == 0)
    {
        if (!debug_armed)
        {
            debug_write("refused: arm first\r\n");
            return;
        }
        debug_target_rad = sensor_direction * MT6816_Get_AngleData();
        debug_integral_v = 0.0f;
        debug_last_us = 0;
        debug_mode = DEBUG_MODE_HOLD;
        debug_printf_i("hold current target_mrad=", to_milli(debug_target_rad), "\r\n");
        return;
    }
    if (strcmp(line, "holdzero") == 0)
    {
        if (!debug_armed)
        {
            debug_write("refused: arm first\r\n");
            return;
        }
        debug_target_rad = DRIVE_ZERO_RAD;
        debug_integral_v = 0.0f;
        debug_last_us = 0;
        debug_mode = DEBUG_MODE_HOLD;
        debug_printf_i("hold zero target_mrad=", to_milli(debug_target_rad), "\r\n");
        return;
    }
    if (strcmp(line, "cal") == 0)
    {
        if (debug_armed)
        {
            debug_write("refused: disarm before cal\r\n");
            return;
        }
        debug_write("running Motor_init calibration\r\n");
        debug_stop_output();
        Motor_init();
        debug_write("cal done\r\n");
        return;
    }
    if (sscanf(line, "pulse %d %d", &a, &b) == 2)
    {
        if (!debug_armed)
        {
            debug_write("refused: arm first\r\n");
            return;
        }
        if (b < 10) b = 10;
        if (b > 500) b = 500;
        debug_voltage_v = clamp_float(from_milli(a), DEBUG_MAX_VOLTAGE_V);
        debug_mode = DEBUG_MODE_VOLTAGE;
        debug_pulse_end_ms = HAL_GetTick() + (uint32_t)b;
        debug_printf_i("pulse_mV=", to_milli(debug_voltage_v), " started\r\n");
        return;
    }
    if (sscanf(line, "v %d", &a) == 1)
    {
        if (!debug_armed)
        {
            debug_write("refused: arm first\r\n");
            return;
        }
        debug_voltage_v = clamp_float(from_milli(a), DEBUG_MAX_VOLTAGE_V);
        debug_mode = (debug_voltage_v == 0.0f) ? DEBUG_MODE_OFF : DEBUG_MODE_VOLTAGE;
        if (debug_mode == DEBUG_MODE_OFF)
        {
            debug_stop_output();
        }
        debug_printf_i("manual_mV=", to_milli(debug_voltage_v), "\r\n");
        return;
    }
    if (sscanf(line, "target %d", &a) == 1)
    {
        debug_target_rad = from_milli(a);
        debug_integral_v = 0.0f;
        debug_printf_i("target_mrad=", to_milli(debug_target_rad), "\r\n");
        return;
    }
    if (sscanf(line, "hold %d", &a) == 1)
    {
        if (!debug_armed)
        {
            debug_write("refused: arm first\r\n");
            return;
        }
        debug_target_rad = from_milli(a);
        debug_integral_v = 0.0f;
        debug_last_us = 0;
        debug_mode = DEBUG_MODE_HOLD;
        debug_printf_i("hold target_mrad=", to_milli(debug_target_rad), "\r\n");
        return;
    }
    if (sscanf(line, "pid %d %d %d %d %d", &a, &b, &c, &d, &e) >= 4)
    {
        debug_kp_v = from_milli(a);
        debug_ki_v = from_milli(b);
        debug_kd_v = from_milli(c);
        debug_limit_v = fabsf(from_milli(d));
        if (debug_limit_v > DEBUG_MAX_VOLTAGE_V) debug_limit_v = DEBUG_MAX_VOLTAGE_V;
        if (e == 1 || e == -1) debug_sign = e;
        debug_integral_v = 0.0f;
        debug_write("pid updated\r\n");
        debug_print_status();
        return;
    }
    if (sscanf(line, "sign %d", &a) == 1)
    {
        if (a != 1 && a != -1)
        {
            debug_write("bad sign, use 1 or -1\r\n");
            return;
        }
        debug_sign = a;
        debug_integral_v = 0.0f;
        debug_printf_i("sign=", debug_sign, "\r\n");
        return;
    }

    debug_write("unknown command, type help\r\n");
}

void DriveUartDebug_Init(uint8_t *rx_buffer, uint16_t rx_buffer_len)
{
    debug_rx_buffer = rx_buffer;
    debug_rx_buffer_len = rx_buffer_len;
    debug_stop_output();
    debug_start_rx();
}

void DriveUartDebug_OnRx(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i)
    {
        char ch = (char)data[i];
        if (ch == '\r' || ch == '\n')
        {
            if (debug_line_len > 0U && !debug_line_pending)
            {
                debug_line[debug_line_len] = '\0';
                memcpy(debug_pending_line, debug_line, debug_line_len + 1U);
                debug_line_pending = 1;
            }
            debug_line_len = 0;
        }
        else if (debug_line_len < sizeof(debug_line) - 1U)
        {
            debug_line[debug_line_len++] = ch;
        }
    }
    debug_start_rx();
}

void DriveUartDebug_RestartRx(void)
{
    debug_start_rx();
}

void DriveUartDebug_Run(void)
{
    debug_handle_swd_command();

    if (debug_print_help)
    {
        debug_print_help = 0;
        debug_print_help_text();
    }

    if (debug_line_pending)
    {
        char line[96];
        __disable_irq();
        memcpy(line, debug_pending_line, sizeof(line));
        debug_line_pending = 0;
        __enable_irq();
        line[sizeof(line) - 1U] = '\0';
        debug_handle_command(line);
    }

    if (!debug_armed)
    {
        debug_stop_output();
    }
    else if (debug_mode == DEBUG_MODE_VOLTAGE)
    {
        if (debug_pulse_end_ms != 0U &&
            (int32_t)(HAL_GetTick() - debug_pulse_end_ms) >= 0)
        {
            debug_stop_output();
            debug_write("pulse done, output 0 mV\r\n");
        }
        else
        {
            debug_last_output_v = debug_voltage_v;
            mode = -1;
            setPhaseVoltage(debug_voltage_v, 0.0f, _electricalAngle());
        }
    }
    else if (debug_mode == DEBUG_MODE_PHASE)
    {
        if (debug_pulse_end_ms != 0U &&
            (int32_t)(HAL_GetTick() - debug_pulse_end_ms) >= 0)
        {
            debug_stop_output();
        }
        else
        {
            debug_last_output_v = debug_voltage_v;
            mode = -1;
            setPhaseVoltage(debug_voltage_v, 0.0f, debug_phase_angle_el);
        }
    }
    else if (debug_mode == DEBUG_MODE_HOLD)
    {
        const uint32_t now_us = HAL_GetTick() * 1000U;
        float dt = 0.001f;
        if (debug_last_us != 0U)
        {
            dt = (float)((uint32_t)(now_us - debug_last_us)) * 1e-6f;
            if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
        }
        debug_last_us = now_us;

        const float angle = sensor_direction * MT6816_Get_AngleData();
        const float vel = sensor_direction * MT6816_Get_Velocity_L();
        const float err = normalize_signed_angle(debug_target_rad - angle);
        debug_integral_v += debug_ki_v * err * dt;
        debug_integral_v = clamp_float(debug_integral_v, debug_limit_v);
        debug_last_output_v = (debug_kp_v * err) - (debug_kd_v * vel) + debug_integral_v;
        debug_last_output_v *= (float)debug_sign;
        debug_last_output_v = clamp_float(debug_last_output_v, debug_limit_v);
        mode = -1;
        setPhaseVoltage(debug_last_output_v, 0.0f, _electricalAngle());
    }
    else
    {
        debug_last_output_v = 0.0f;
        setPhaseVoltage(0.0f, 0.0f, _electricalAngle());
    }

    if (debug_telemetry && HAL_GetTick() - debug_last_telemetry_ms >= 100U)
    {
        debug_last_telemetry_ms = HAL_GetTick();
        debug_print_status();
    }

    debug_update_swd_status();
}
