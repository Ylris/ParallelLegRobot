#ifndef DRIVE_UART_DEBUG_H
#define DRIVE_UART_DEBUG_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

void DriveUartDebug_Init(uint8_t *rx_buffer, uint16_t rx_buffer_len);
void DriveUartDebug_Run(void);
void DriveUartDebug_OnRx(uint8_t *data, uint16_t size);
void DriveUartDebug_RestartRx(void);

#endif
