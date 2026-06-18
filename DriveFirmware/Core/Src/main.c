/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Data.h"
#include "FOC.h"
#include "mt6816.h"
#include "Motor.h"
#include "Lowpass.h"
#include "can_bridge.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t dma_in_process=0;
static uint32_t overflow_count = 0;
static uint8_t buffer_receive[100];
static volatile uint8_t motor_calibration_requested=0;
/* USER CODE END PV */

#ifndef YYT_SAFE_START
#define YYT_SAFE_START 1
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t Get_Timestamp_us(void)
{
    uint32_t timestamp = 0;
    uint32_t current_count;
	
    __disable_irq();
    current_count = __HAL_TIM_GET_COUNTER(&htim4);
    timestamp = (overflow_count * 0xFFFF + current_count); // 1MHz
    __enable_irq();

    return timestamp;
}

float parseData(const char* input)
{
		const char* equalSign_ptr = strchr(input, '=');
    if (equalSign_ptr == NULL)
    {
        return -1.0f;
    }
		const char* end = strchr(input, '!');
    if (end == NULL)
    {
        return -2.0f;
    }
		const char* data_ptr=equalSign_ptr+1;
		
		char command[16]; 
    char data[16]; 
    size_t command_length = equalSign_ptr - input;
		size_t data_length=end-data_ptr;
   
    strncpy(command, input, command_length);
		strncpy(data, data_ptr, data_length);
    command[command_length] = '\0'; 
		data[data_length] = '\0'; 
		float data_flt=1.0f;
    if(strcmp(command,"P1")==0)
		{
			PID_Pos.P= atof(data);
			
		}
		else if(strcmp(command,"I1")==0)
		{
			PID_Pos.I= atof(data);
		}
		else if(strcmp(command,"D1")==0)
		{
			PID_Pos.D= atof(data);
		}
		else if(strcmp(command,"P0")==0)
		{
			Pos_set= atof(data);
		}
		else if(strcmp(command,"V0")==0)
		{
			Velo_set= atof(data);
		}
		else if(strcmp(command,"P2")==0)
		{
			PID_Vel.P= atof(data);
		}
		else if(strcmp(command,"I2")==0)
		{
			PID_Vel.I= atof(data);
		}
		else if(strcmp(command,"D2")==0)
		{
			PID_Vel.D= atof(data);
		}
		else if(strcmp(command,"Q0")==0)
		{
			IQ_set= atof(data);
		}
		else if(strcmp(command,"D0")==0)
		{
			ID_set= atof(data);
		}
		
		else if(strcmp(command,"P3")==0)
		{
			PID_Cur_Q.P= atof(data);
		}
		else if(strcmp(command,"I3")==0)
		{
			PID_Cur_Q.I= atof(data);
		}
		else if(strcmp(command,"D3")==0)
		{
			PID_Cur_Q.D= atof(data);
		}
		else if(strcmp(command,"P4")==0)
		{
			PID_Cur_D.P= atof(data);
		}
		else if(strcmp(command,"I4")==0)
		{
			PID_Cur_D.I= atof(data);
		}
		else if(strcmp(command,"D4")==0)
		{
			PID_Cur_D.D= atof(data);
		}
		else if(strcmp(command,"P5")==0)
		{
			PID_Vel_Cur.P= atof(data);
		}
		else if(strcmp(command,"I5")==0)
		{
			PID_Vel_Cur.I= atof(data);
		}
		else if(strcmp(command,"D5")==0)
		{
			PID_Vel_Cur.D= atof(data);
		}
		else if(strcmp(command,"P6")==0)
		{
			PID_Pos_Cur.P= atof(data);
		}
		else if(strcmp(command,"I6")==0)
		{
			PID_Pos_Cur.I= atof(data);
		}
		else if(strcmp(command,"D6")==0)
		{
			PID_Pos_Cur.D= atof(data);
		}
				else if(strcmp(command,"P7")==0)
		{
			PID_Pos_Vel_Cur.P= atof(data);
		}
		else if(strcmp(command,"I7")==0)
		{
			PID_Pos_Vel_Cur.I= atof(data);
		}
		else if(strcmp(command,"D7")==0)
		{
			PID_Pos_Vel_Cur.D= atof(data);
		}
		else if(strcmp(command,"M0")==0)
		{
			mode= atof(data);
		}
		else if(strcmp(command,"C0")==0)
		{
			if(atof(data)>0.5f)
			{
				motor_calibration_requested=1;
			}
		}
		else if(strcmp(command,"L0")==0)
		{
			uq_limit= atof(data);
		}
		return data_flt;
}

void VOFA_RUN()
{
		if(mode<0)
		{
			setPhaseVoltage(0,0,zero_electric_angle_norm);
		}
		else if(mode==0)
		{
			PositionCloseloop(Pos_set);
		}
		else if(mode==1)
		{
			VelocityCloseloop(Velo_set);
		}
		else if(mode==2)
		{
			//velocityOpenloop(0.1);
			CurrentCloseloop(IQ_set,ID_set);
		}
		else if(mode==3)
		{
			Velocity_CurrentCloseloop(Velo_set);
		}
		else if(mode==4)
		{
			Position_CurrentCloseloop(Pos_set);
		}
		else if(mode==5)
		{
			Position_Velocity_CurrentCloseloop(Pos_set);
		}
		else if(mode==6)
		{
			velocityOpenloop(0.1);
		}
		
		
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void System_Init()
{
		LOWPass_Init();
	  convert=vcc/shunt_resistor/adc_cpr/amp_gain;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim1) - 10);	
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	ADC_Count_Caloffset();
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); //开启三个PWM通道输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	MT6816_SPI_Signal_Init();
	CAN_Bridge_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	System_Init();
#if YYT_SAFE_START
	PID_Pos_Set(50,27,0,6);
  PID_Cur_D_Set(7.5,30,0,6);
	PID_Cur_Q_Set(7.5,40,0,12);
	PID_Vel_Set(5,6.2,0,12);
	PID_Vel_Cur_Set(0.7,5.7,0,12);
	PID_Pos_Cur_Set(20,2,0,3);
	pole_pairs = 11;
	Pos_set=0;
	Velo_set=0;
	IQ_set=0;
	ID_set=0;
	mode=-1;
	uq_limit=6;
	voltage_power_supply =24;
#else
	Motor_init();
	//4108 10 oomm parameter
	PID_Pos_Set(50,27,0,6);
  PID_Cur_D_Set(7.5,30,0,6);
	PID_Cur_Q_Set(7.5,40,0,12);
	PID_Vel_Set(5,6.2,0,12);
	PID_Vel_Cur_Set(0.7,5.7,0,12);
	PID_Pos_Cur_Set(20,2,0,3);
	pole_pairs = 11;
	Velo_set=6.28;
	mode=1;
	voltage_power_supply =24;

	//4108 3 oomm parameter
//	PID_Pos_Set(50,27,0,6);
//  PID_Cur_D_Set(2.2,3,0,6);
//	PID_Cur_Q_Set(2.2,3,0,6);
//	PID_Vel_Set(7.5,0.1,0,12);
//		pole_pairs = 11;
	//6015 parameter
//	PID_Pos_Set(50,27,0,6);
//	PID_Pos_Cur_Set(20,2,0,3);
//	PID_Pos_Vel_Cur_Set(20,2,0,3);
//  PID_Cur_D_Set(7.5,30,0,6);
//	PID_Cur_Q_Set(18,20,0,12);
//	PID_Vel_Set(7.5,0.1,0,12);
//	PID_Vel_Cur_Set(2.4,30,0,12);
//	pole_pairs = 14;
//	Velo_set=6.28;
//	mode=3;
//	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//	Pos_set=3.14;
//	IQ_set=3.0;
#endif
	
  while (1)
  {
		if(motor_calibration_requested)
		{
			motor_calibration_requested=0;
			mode=-1;
			IQ_set=0;
			ID_set=0;
			Velo_set=0;
			Motor_init();
		}
//		velocityOpenloop(0.1);
		//MT6816_Get_Velocity_L();
		//VelocityCloseloop(Velo_set);
		clarke_transform();
		park_transform(zero_electric_angle_norm);
		CAN_Bridge_Run();
		VOFA_RUN();

		//PositionCloseloop(Pos_set);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ADC_Count_Caloffset()
{
    for (int i = 0; i < 16; ++i)
    {
        HAL_Delay(1);
        adc_value_in1_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        adc_value_in2_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        adc_value_in3_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
			  adc_value_in4_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
        bias_ia+=adc_value_in1_raw;
				bias_ib+=adc_value_in2_raw;
				bias_ic+=adc_value_in3_raw;
				bias_vbus+=adc_value_in4_raw;
    }
		bias_ia = bias_ia / 16.0f;
    bias_ib = bias_ib / 16.0f;
    bias_ic = bias_ic / 16.0f;
    bias_vbus = bias_vbus / 16.0f;
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	{
			adc_value_in1 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
			
			ia=(adc_value_in1-bias_ia)*0.00161132f;
      adc_value_in2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
			ib=(adc_value_in2-bias_ib)*0.00161132f;
      adc_value_in3 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
			ic=(adc_value_in3-bias_ic)*0.00161132f;
		  adc_value_in4 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
			vbus=(adc_value_in4-bias_vbus)*0.0088623f;
	}
}


int count=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
		
		if (htim->Instance == TIM3)
	 {		 
			static float data[11];
		  data[0]=(float)ia;
			data[1]=(float)ib;
			data[2]=(float)ic;
		 
			data[3]=(float)Velo_set ;
			data[4]=(float)mt6816_spi.velocity_time_pack_prev.velocity;
			data[5]=(float)mt6816_spi.angle;
			data[6]=(float)mt6816_spi.angle_full_rotation;
			data[7]=(float)Id;
			data[8]=(float)Iq;
			data[9]=(float)PID_output;
			data[10]=(float)zero_electric_angle;
		 
			static uint8_t tail[4]={0x00,0x00,0x80,0x7f}; 
			static uint8_t buffer_transmit[sizeof(float)*11 + sizeof(tail)];

			memcpy(buffer_transmit, data, sizeof(float)*11);

			memcpy(buffer_transmit + sizeof(float)*11, tail, sizeof(tail));
			if(dma_in_process==0)
			{
				dma_in_process=1;
				HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer_receive, sizeof(buffer_receive));
				HAL_UART_Transmit_DMA(&huart1, buffer_transmit, sizeof(buffer_transmit));
			}
		}
	 
		if (htim->Instance == TIM4)
		{
			 overflow_count++;
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
			dma_in_process=0;
			if(count==0) 
			{
				count++;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
			}
			else if(count==1)
			{
				count--;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
			}
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	if(huart==&huart1)
	{
			static float data_[1];
		  data_[0]=(float)parseData((char*)buffer_receive);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer_receive, sizeof(buffer_receive));
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
