#include "can_bridge.h"

#include "fdcan.h"
#include "Data.h"
#include "FOC.h"
#include "mt6816.h"
#include <string.h>

#ifndef DRIVE_ID
#define DRIVE_ID 1
#endif

#define CAN_CMD_LEFT_GROUP_ID   0x100U
#define CAN_CMD_RIGHT_GROUP_ID  0x200U
#define CAN_FEEDBACK_BASE_ID    0x100U
#define CAN_COMMAND_TIMEOUT_US  100000U
#define CAN_FEEDBACK_PERIOD_US  2000U
#define CAN_MAX_COMMAND_VOLTAGE 3.0f

extern uint32_t Get_Timestamp_us(void);

static float can_voltage_cmd = 0.0f;
static uint32_t last_command_us = 0;
static uint32_t last_feedback_us = 0;
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
    can_voltage_cmd = clamp_float((float)mv * 0.001f, CAN_MAX_COMMAND_VOLTAGE);
    last_command_us = Get_Timestamp_us();
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
    float angle = sensor_direction * MT6816_Get_FullAngleData();
    float speed = sensor_direction * MT6816_Get_Velocity_L();
    int32_t angle_mrad = (int32_t)(angle * 1000.0f);
    int16_t speed_rpm_x10 = (int16_t)(speed * 60.0f / (2.0f * PI) * 10.0f);

    write_i32_le(&tx_data[0], angle_mrad);
    write_i16_le(&tx_data[4], speed_rpm_x10);
    write_i16_le(&tx_data[6], 0);

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

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
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

    now_us = Get_Timestamp_us();
    poll_rx();

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

    if ((uint32_t)(now_us - last_feedback_us) >= CAN_FEEDBACK_PERIOD_US)
    {
        last_feedback_us = now_us;
        send_feedback();
    }
}
