#include "mt6816.h"
#include "stm32g4xx_hal.h"
#include "stdbool.h"
#include "Lowpass.h"

extern SPI_HandleTypeDef hspi1; // 假设 SPI1 已经通过 HAL 库初始化

// 定义 CS 引脚
#define MT6816_CS1_PORT GPIOB
#define MT6816_CS1_PIN GPIO_PIN_12
#define MT6816_CS2_PORT GPIOA
#define MT6816_CS2_PIN GPIO_PIN_11

// CS 引脚控制宏
#define MT6816_CS1_L() HAL_GPIO_WritePin(MT6816_CS1_PORT, MT6816_CS1_PIN, GPIO_PIN_RESET)
#define MT6816_CS1_H() HAL_GPIO_WritePin(MT6816_CS1_PORT, MT6816_CS1_PIN, GPIO_PIN_SET)
#define MT6816_CS2_L() HAL_GPIO_WritePin(MT6816_CS2_PORT, MT6816_CS2_PIN, GPIO_PIN_RESET)
#define MT6816_CS2_H() HAL_GPIO_WritePin(MT6816_CS2_PORT, MT6816_CS2_PIN, GPIO_PIN_SET)


MT6816_SPI_Signal_Typedef mt6816_spi;

void MT6816_SPI_Signal_Init(void) {
    mt6816_spi.sample_data = 0;
		mt6816_spi.full_rotation_offset=0;
		mt6816_spi.cpr=MT6816_CPR;
		SPI1->CR1 |= SPI_CR1_SPE;
	
		mt6816_spi.angle_time_pack_prev.angle_full_rotation=MT6816_Get_FullAngleData();
		mt6816_spi.angle_time_pack_prev.timestamp=Get_Timestamp_us();
	
		mt6816_spi.velocity_time_pack.timestamp=Get_Timestamp_us();
	  mt6816_spi.velocity_time_pack.velocity=0;
}

float MT6816_SPI_Get_AngleData(GPIO_TypeDef *CS_PORT, uint16_t CS_PIN) {
    uint16_t data_t[2];
    uint16_t data_r[2];
    
		float d_angle = 0;
    // 发送的命令
    data_t[0] = (0x80 | 0x03) << 8;
    data_t[1] = (0x80 | 0x04) << 8;

	  for(uint8_t i=0;i<1;i++)
		{
			uint8_t h_count=0;
			// 读取第一组数据
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); // CS 拉低
			HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data_t[0], (uint8_t *)&data_r[0], 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);   // CS 拉高

			// 读取第二组数据
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); // CS 拉低
			HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data_t[1], (uint8_t *)&data_r[1], 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);   // CS 拉高
			
			// 处理接收到的数1
			mt6816_spi.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
			// 奇偶校验
			//h_count = __builtin_popcount(mt6816_spi.sample_data);
			for (uint8_t j = 0; j < 16; j++) 
			{
				if (mt6816_spi.sample_data & (0x0001 << j))
					h_count++;
			}
			mt6816_spi.pc_flag = (h_count & 0x01) == 0;

			if (mt6816_spi.pc_flag) {
					mt6816_spi.angle_data = mt6816_spi.sample_data >> 2;
					mt6816_spi.no_mag_flag = (bool)(mt6816_spi.sample_data & (0x0001 << 1));
			}
		}
		//检查角度跳变，增减圈数
		d_angle=mt6816_spi.angle_data-mt6816_spi.angle_data_prev;
		if (fabs(d_angle) > (0.8f * mt6816_spi.cpr))
				mt6816_spi.full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI;
		mt6816_spi.angle_data_prev = mt6816_spi.angle_data;
		
		return (float)mt6816_spi.angle_data/mt6816_spi.cpr* _2PI;
}

float MT6816_Get_AngleData() 
{
		mt6816_spi.angle=MT6816_SPI_Get_AngleData(MT6816_CS1_PORT,MT6816_CS1_PIN);
		return mt6816_spi.angle;
}

float MT6816_Get_FullAngleData()
{ 
		mt6816_spi.angle_full_rotation=mt6816_spi.full_rotation_offset+MT6816_Get_AngleData();
		return mt6816_spi.angle_full_rotation;
}

// 获取原始速度
float MT6816_Get_Velocity_RAW()
{
    uint32_t now_us;
    float Ts, angle_current,vel;
		// 当前角度
    angle_current = MT6816_Get_FullAngleData();
    // 计算采样时间
    now_us = Get_Timestamp_us(); 
    if (now_us > mt6816_spi.angle_time_pack_prev.timestamp)
        Ts = (float)(now_us - mt6816_spi.angle_time_pack_prev.timestamp) * 1e-6f;
    else//如果时间溢出
        Ts = (float)(0xFFFFFFFF - mt6816_spi.angle_time_pack_prev.timestamp + now_us) * 1e-6f;

    // 快速修复奇怪情况（微秒溢出）
    if (Ts < 1e-6f || Ts > 0.5) Ts = 1e-4f;
		float inv_Ts = 1.0f / Ts;

    // 速度计算
    mt6816_spi.angle_diff = (float)(angle_current*1e6f-mt6816_spi.angle_time_pack_prev.angle_full_rotation*1e6f)*1e-6f;
		if(mt6816_spi.angle_diff<8.28f&&mt6816_spi.angle_diff>5.28f) mt6816_spi.angle_diff-=_2PI;
		if(mt6816_spi.angle_diff>-8.28f&&mt6816_spi.angle_diff<-4.28f) mt6816_spi.angle_diff+=_2PI;
		

		mt6816_spi.time_diff=Ts;
		vel=mt6816_spi.angle_diff*inv_Ts;
		mt6816_spi.angle_time_pack_prev.angle_full_rotation=angle_current;
		mt6816_spi.angle_time_pack_prev.timestamp=now_us;
		

		mt6816_spi.velocity_time_pack.velocity=vel;
		mt6816_spi.velocity_time_pack.timestamp=now_us;
    return vel;
}

// 有滤波速度
extern LowPassFilter  LPF_velocity;
float MT6816_Get_Velocity_L(void)
{
    
		mt6816_spi.velocity_time_pack_prev.timestamp=mt6816_spi.velocity_time_pack.timestamp;
		mt6816_spi.velocity_time_pack_prev.velocity=mt6816_spi.velocity_time_pack.velocity;
		
		float vel_ori = MT6816_Get_Velocity_RAW();
    float vel_flit = LowPassFilter_operator(&LPF_velocity, mt6816_spi.velocity_time_pack_prev,mt6816_spi.velocity_time_pack);
		mt6816_spi.vel_filter=vel_flit;
    return vel_flit;   
}