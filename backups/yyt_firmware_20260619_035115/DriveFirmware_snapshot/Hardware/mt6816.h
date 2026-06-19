#ifndef _MT6816_H
#define _MT6816_H

#include <stdbool.h>	
#include <string.h>		
#include <stdlib.h>		
#include <stdio.h>		
#include <spi.h>		
#include <main.h>		
#include "math.h"
#include "tim.h"



#define MT6816_CS1_PORT GPIOB
#define MT6816_CS1_PIN GPIO_PIN_12
#define MT6816_CS2_PORT GPIOA
#define MT6816_CS2_PIN GPIO_PIN_11

#define MT6816_CS1_L() HAL_GPIO_WritePin(MT6816_CS1_PORT, MT6816_CS1_PIN, GPIO_PIN_RESET)
#define MT6816_CS1_H() HAL_GPIO_WritePin(MT6816_CS1_PORT, MT6816_CS1_PIN, GPIO_PIN_SET)
#define MT6816_CS2_L() HAL_GPIO_WritePin(MT6816_CS2_PORT, MT6816_CS2_PIN, GPIO_PIN_RESET)
#define MT6816_CS2_H() HAL_GPIO_WritePin(MT6816_CS2_PORT, MT6816_CS2_PIN, GPIO_PIN_SET)

#define MT6816_SPI_Get_HSPI (hspi1)
#define MT6816_SPI_Mode (0x03)

#define MT6816_CPR 16384
#define _2PI 6.28318530718f
#define PI 3.1415926

typedef struct{
	float angle_full_rotation;
	uint32_t	timestamp;
}Angle_Time_Pack;

typedef struct{
	float velocity;
	uint32_t	timestamp;
}Velocity_Time_Pack;

typedef struct {
    uint16_t sample_data;
    uint16_t angle_data;
		uint16_t angle_data_prev;
	
		Angle_Time_Pack angle_time_pack_prev;
		Velocity_Time_Pack velocity_time_pack;
		Velocity_Time_Pack velocity_time_pack_prev;
		float angle;
		float full_rotation_offset;
		float angle_full_rotation;
	
		long  cpr; 
    bool no_mag_flag;
		bool pc_flag;
		float vel_filter;
		float time_diff;
		float angle_diff;
} MT6816_SPI_Signal_Typedef;

extern MT6816_SPI_Signal_Typedef mt6816_spi;

void MT6816_SPI_Signal_Init(void);
float MT6816_SPI_Get_AngleData(GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
float MT6816_Get_AngleData();
float MT6816_Get_FullAngleData();
float MT6816_Get_Velocity_RAW();
float MT6816_Get_Velocity_L();



#endif 