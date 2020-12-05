
#include "main.h"
#include "stm32f4xx_hal.h"
#include "MPU6050.h"


I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
int16_t Ax, Ay, Az, Gx, Gy, Gz;
double GYRO_X_RATE, GYRO_Y_RATE, GYRO_Z_RATE, GYRO_X_OFFSET,GYRO_Y_OFFSET, GYRO_Z_OFFSET, GYRO_X_ANGLE, GYRO_Y_ANGLE, GYRO_Z_ANGLE,ACCEL_X_ANGLE, ACCEL_Y_ANGLE, ANGLE_X_REAL, ANGLE_Y_REAL, dt=0.004;
uint8_t TX[2], RX;
long GYRO_X_OFFSET_SUM, GYRO_Y_OFFSET_SUM, GYRO_Z_OFFSET_SUM;
int16_t duty1, duty2, duty3, duty4,fade = 100;
int16_t throttle_y;
double PID_ErrorX, p_output_x, i_output_x, d_output_x, output_x, PID_Last_Error_x;
double PID_ErrorY, p_output_y, i_output_y, d_output_y, output_y, PID_Last_Error_y;
double PID_ErrorZ, p_output_z, i_output_z, d_output_z, output_z, PID_Last_Error_z;
uint8_t data;
uint8_t dataLG[4];
/* PID gain */
float setpoint_x = 0;
double Kp_x = 4;
double Ki_x = 0.03;
double Kd_x = 18;
int16_t output_x_max=400;

float setpoint_y = 0;
double Kp_y = 4;
double Ki_y = 0.03;
double Kd_y = 18;
int16_t output_y_max=400;

float setpoint_z = 0;
double Kp_z = 4;
double Ki_z = 0.045;
double Kd_z = 0;
int16_t output_z_max=400;
/* PID gain */

void SystemClock_Config(void);
void SysTick_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Set_PWM(int16_t PWM_time1,int16_t PWM_time2, int16_t PWM_time3, int16_t PWM_time4)
{

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // kh?i d?ng Timer 4 kênh 1
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // kh?i d?ng Timer 4 kênh 2
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // kh?i d?ng Timer 4 kênh 3
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // kh?i d?ng Timer 4 kênh 4
TIM4->CCR1 = PWM_time1; // thay d?i channel 1
TIM4->CCR2 = PWM_time2; // thay d?i channel 2
TIM4->CCR3 = PWM_time3; // thay d?i channel 3
TIM4->CCR4 = PWM_time4; // thay d?i channel 4
duty1 = PWM_time4;
duty2 = PWM_time3;
duty3 = PWM_time2;
duty4 = PWM_time1;
}

void PID_Calculate()
{
	/* PID X */
	PID_ErrorX = ANGLE_X_REAL - setpoint_x;
	
	p_output_x = Kp_x*PID_ErrorX;
	i_output_x += Ki_x*PID_ErrorX;
	d_output_x = Kd_x*(PID_ErrorX - PID_Last_Error_x);
	
	if(i_output_x > output_x_max) i_output_x = output_x_max;
	else if(i_output_x < -output_x_max) i_output_x = -output_x_max;
	
	output_x =  p_output_x + i_output_x + d_output_x;
	
	if(output_x > output_x_max) output_x = output_x_max;
	else if(output_x < -output_x_max) output_x = -output_x_max;

	PID_Last_Error_x = PID_ErrorX;
	
	/* PID Y */
	PID_ErrorY = ANGLE_Y_REAL - setpoint_y;
	
	p_output_y = Kp_y*PID_ErrorY;
	i_output_y += Ki_y*PID_ErrorY;
	d_output_y = Kd_y*(PID_ErrorY - PID_Last_Error_y);
	
	if(i_output_y > output_y_max) i_output_y = output_y_max;
	else if(i_output_y < -output_y_max) i_output_y = -output_y_max;
	
	output_y =  p_output_y + i_output_y + d_output_y;
			
	if(output_y > output_y_max) output_y = output_y_max;
	else if(output_y < -output_y_max) output_y = -output_y_max;
	
	PID_Last_Error_y = PID_ErrorY;

	/* PID Z */
	PID_ErrorZ = GYRO_Z_ANGLE - setpoint_z;
	
	p_output_z = Kp_z*PID_ErrorZ;
	i_output_z += Ki_z*PID_ErrorZ;
	d_output_z = Kd_z*(PID_ErrorZ - PID_Last_Error_z);
	
	if(i_output_z > output_z_max) i_output_z = output_z_max;
	else if(i_output_z < -output_z_max) i_output_z = -output_z_max;
	
	output_z =  p_output_z + i_output_z + d_output_z;
			
	if(output_z > output_z_max) output_z = output_z_max;
	else if(output_z < -output_z_max) output_z = -output_z_max;
	
	PID_Last_Error_z = PID_ErrorZ;

	/* Calculate the pulse for esc */
	duty1 = throttle_y - output_y - output_x;
	duty2 = throttle_y + output_y - output_x;
	duty3 = throttle_y + output_y + output_x;
	duty4 = throttle_y - output_y + output_x;
	
	if(duty1 > 1900) duty1 = 1900;
	else if(duty1 < 1200) duty1 = 1200;
	
	if(duty2 > 1900) duty2 = 1900;
	else if(duty2 < 1200) duty2 = 1200;

	if(duty3 > 1900) duty3 = 1900;
	else if(duty3 < 1200) duty3 = 1200;

	if(duty4 > 1900) duty4 = 1900;
	else if(duty4 < 1200) duty4 = 1200;
	
	Set_PWM(duty1, duty2, duty3, duty4);
	
}
void HAL_SYSTICK_Callback(void);

int main(void)
{
  
  HAL_Init();

  
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
//	Mpu6050_Init();
//	Calibrate_Gyros();
	Set_PWM(1000, 1000, 1000, 1000);
	HAL_Delay(3000);
  data = 0;
  while (1)
  {
//		Gyro_Angle();
//		Accel_Angle();
		HAL_UART_Receive(&huart2, &data,1, 0);
		if( data == 'A')
		{
			duty1 += fade;
			duty2 += fade;
			duty3 += fade;
			duty4 += fade;
		} else if (data == 'B')
		{
			duty1 -= fade;
			duty2 -= fade;
			duty3 -= fade;
			duty4 -= fade;
		}
//		else
//		{
//			duty1 = duty1;
//			duty2 = duty2;
//			duty3 = duty3;
//			duty4 = duty4;
//		}
			setpoint_x = dataLG[0] - 20;
			setpoint_y = dataLG[1] - 20;
		
//		PID_Calculate();
//		Set_PWM(duty1, duty2, duty3, duty4);
	}
			
		
	
}
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
