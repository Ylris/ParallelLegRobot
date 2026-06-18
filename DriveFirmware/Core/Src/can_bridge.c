#include "can_bridge.h"

#include "fdcan.h"
#include "Data.h"
#include "FOC.h"
#include "Motor.h"
#include "mt6816.h"
#include <math.h>
#include <string.h>

#ifndef DRIVE_ID
#define DRIVE_ID 1
#endif

#define CAN_CMD_LEFT_GROUP_ID   0x100U
#define CAN_CMD_RIGHT_GROUP_ID  0x200U
#define CAN_FEEDBACK_BASE_ID    0x100U
#define CAN_COMMAND_TIMEOUT_US  100000U
#define CAN_FEEDBACK_PERIOD_US  2000U
#define CAN_RECOVERY_PERIOD_US  100000U
#define CAN_MAX_COMMAND_VOLTAGE 12.0f
#define CAN_SPIN_TEST_MV        4321
#define CAN_ZERO_HOLD_MV        2345
#define CAN_SPIN_TEST_VOLTAGE   6.0f
#define CAN_SPIN_TEST_VELOCITY  0.12f
#define CAN_ZERO_HOLD_KP        12.0f
#define CAN_ZERO_HOLD_KI        3.0f
#define CAN_ZERO_HOLD_LIMIT     12.0f
#define CAN_ZERO_APPROACH_RAD   0.12f

#ifndef DRIVE_ZERO_RAD
#if DRIVE_ID == 1
#define DRIVE_ZERO_RAD          4.860f
#elif DRIVE_ID == 2
#define DRIVE_ZERO_RAD          5.553f
#elif DRIVE_ID == 5
#define DRIVE_ZERO_RAD          0.161f
#elif DRIVE_ID == 6
#define DRIVE_ZERO_RAD          1.991f
#else
#define DRIVE_ZERO_RAD          0.0f
#endif
#endif

extern uint32_t Get_Timestamp_us(void);

static float can_voltage_cmd = 0.0f;
static float can_spin_velocity = 0.0f;
static float can_zero_integral = 0.0f;
static uint32_t can_zero_pid_us = 0;
static uint32_t last_command_us = 0;
static uint32_t last_feedback_us = 0;
static uint32_t last_recovery_us = 0;
static uint8_t can_enabled = 0;

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void write_i16_le(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value & 0xFFU);
    data[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

static void write_i32_le(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)((uint32_t)value & 0xFFU);
    data[1] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
    data[2] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
    data[3] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}

static float clamp_float(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void process_command(uint32_t std_id, const uint8_t *data)
{
    uint8_t slot;
    int16_t mv;

    if (DRIVE_ID >= 1 && DRIVE_ID <= 4 && std_id == CAN_CMD_LEFT_GROUP_ID)
    {
        slot = (uint8_t)(DRIVE_ID - 1U);
    }
    else if (DRIVE_ID >= 5 && DRIVE_ID <= 8 && std_id == CAN_CMD_RIGHT_GROUP_ID)
    {
        slot = (uint8_t)(DRIVE_ID - 5U);
    }
    else
    {
        return;
    }

    mv = read_i16_le(&data[slot * 2U]);
    last_command_us = Get_Timestamp_us();
    if (mv == CAN_SPIN_TEST_MV || mv == -CAN_SPIN_TEST_MV)
    {
        can_spin_velocity = mv > 0 ? CAN_SPIN_TEST_VELOCITY : -CAN_SPIN_TEST_VELOCITY;
        mode = 8;
        return;
    }
    if (mv == CAN_ZERO_HOLD_MV || mv == -CAN_ZERO_HOLD_MV)
    {
        uint8_t entering_zero_hold = (mode != 9);
        if (entering_zero_hold)
        {
            can_zero_integral = 0.0f;
            can_zero_pid_us = Get_Timestamp_us();
        }
        Pos_set = DRIVE_ZERO_RAD;
        mode = 9;
        return;
    }

    can_voltage_cmd = clamp_float((float)mv * 0.001f, CAN_MAX_COMMAND_VOLTAGE);
    mode = 7;
}

static void poll_rx(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }
        if (rx_header.IdType == FDCAN_STANDARD_ID && rx_header.DataLength == FDCAN_DLC_BYTES_8)
        {
            process_command(rx_header.Identifier, rx_data);
        }
    }
}

static void send_feedback(void)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
#if CAN_FEEDBACK_SINGLE_TURN
    float angle = sensor_direction * MT6816_Get_AngleData();
    float speed = 0.0f;
#else
    float angle = sensor_direction * MT6816_Get_FullAngleData();
    float speed = sensor_direction * MT6816_Get_Velocity_L();
#endif
    int32_t angle_mrad = (int32_t)(angle * 1000.0f);
    int16_t speed_rpm_x10 = (int16_t)(speed * 60.0f / (2.0f * PI) * 10.0f);
    uint16_t adc4_raw = adc_value_in4 > 65535U ? 65535U : (uint16_t)adc_value_in4;

    write_i32_le(&tx_data[0], angle_mrad);
    write_i16_le(&tx_data[4], speed_rpm_x10);
    write_i16_le(&tx_data[6], (int16_t)adc4_raw);

    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.Identifier = CAN_FEEDBACK_BASE_ID + DRIVE_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        return;
    }

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}

static void recover_can_if_needed(uint32_t now_us)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;
    uint8_t needs_recovery = 0;

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) != HAL_OK)
    {
        needs_recovery = 1;
    }
    else if (protocol_status.BusOff != 0U)
    {
        needs_recovery = 1;
    }
    else if ((FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0U)
    {
        needs_recovery = 1;
    }

    if (!needs_recovery ||
        (uint32_t)(now_us - last_recovery_us) < CAN_RECOVERY_PERIOD_US)
    {
        return;
    }

    last_recovery_us = now_us;
    can_zero_integral = 0.0f;
    mode = -1;
    setPhaseVoltage(0.0f, 0.0f, _electricalAngle());

    (void)HAL_FDCAN_Stop(&hfdcan1);
    if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK)
    {
        last_command_us = now_us;
        last_feedback_us = now_us;
    }
}

void CAN_Bridge_Init(void)
{
    FDCAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) == HAL_OK &&
        HAL_FDCAN_Start(&hfdcan1) == HAL_OK)
    {
        can_enabled = 1;
        last_command_us = Get_Timestamp_us();
        last_feedback_us = last_command_us;
    }
}

void CAN_Bridge_Run(void)
{
    uint32_t now_us;

    if (!can_enabled)
    {
        return;
    }

    poll_rx();
    now_us = Get_Timestamp_us();
    recover_can_if_needed(now_us);

    if (mode == 7)
    {
        if ((uint32_t)(now_us - last_command_us) > CAN_COMMAND_TIMEOUT_US)
        {
            can_voltage_cmd = 0.0f;
            mode = -1;
        }
        else
        {
            setPhaseVoltage(can_voltage_cmd, 0.0f, _electricalAngle());
        }
    }
    else if (mode == 8)
    {
        if ((uint32_t)(now_us - last_command_us) > CAN_COMMAND_TIMEOUT_US)
        {
            mode = -1;
            setPhaseVoltage(0.0f, 0.0f, _electricalAngle());
        }
        else
        {
            velocityOpenloopVoltage(can_spin_velocity, CAN_SPIN_TEST_VOLTAGE);
        }
    }
    else if (mode == 9)
    {
        if ((uint32_t)(now_us - last_command_us) > CAN_COMMAND_TIMEOUT_US)
        {
            can_zero_integral = 0.0f;
            mode = -1;
            setPhaseVoltage(0.0f, 0.0f, _electricalAngle());
        }
        else
        {
            float angle = sensor_direction * MT6816_Get_FullAngleData();
            float error = angle - DRIVE_ZERO_RAD;
            if (fabsf(error) > CAN_ZERO_APPROACH_RAD)
            {
                can_zero_integral = 0.0f;
                can_zero_pid_us = now_us;
                can_spin_velocity = error > 0.0f ? -CAN_SPIN_TEST_VELOCITY : CAN_SPIN_TEST_VELOCITY;
                velocityOpenloopVoltage(can_spin_velocity, CAN_SPIN_TEST_VOLTAGE);
            }
            else
            {
                float pid_error;
                float output;
                float Ts = (float)((uint32_t)(now_us - can_zero_pid_us)) * 1e-6f;
                if (Ts <= 0.0f || Ts > 0.5f)
                {
                    Ts = 1e-3f;
                }
                can_zero_pid_us = now_us;

                pid_error = DRIVE_ZERO_RAD - angle;
                can_zero_integral += CAN_ZERO_HOLD_KI * pid_error * Ts;
                can_zero_integral = clamp_float(can_zero_integral, CAN_ZERO_HOLD_LIMIT);
                output = CAN_ZERO_HOLD_KP * pid_error + can_zero_integral;
                output = clamp_float(output, CAN_ZERO_HOLD_LIMIT);
                setPhaseVoltage(output, 0.0f, _electricalAngle());
            }
        }
    }

    if ((uint32_t)(now_us - last_feedback_us) >= CAN_FEEDBACK_PERIOD_US)
    {
        last_feedback_us = now_us;
        send_feedback();
    }
}
