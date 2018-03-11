/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "vl53l0x.h"
#include "MPU9250.h"
#include "PMW3901.h"
#include "mavlink/standard/mavlink.h"
#include "mavlink/mavlink_helpers.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile float r_x = 0, r_y = 0, r_z = 0;
float pos_x = 0, pos_y = 0, pos_z = 0;
int16_t distance_front = 0, altitude = 0;

/*
 * RC Commands
 */
int16_t roll = 0, pitch = 0, yaw = 0, throttle = 0;
_Bool enabled = false;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	/*
	uint8_t bla[] = "started...\r\n";
	HAL_UART_Transmit(&huart1, bla, strlen(bla), 0xff);

	HAL_Delay(1000);
	char cmd1[] = "+++\r";
	char response1[2] = {0};
	HAL_UART_Transmit(&huart2, cmd1, strlen(cmd1), 0xff);
	HAL_UART_Receive(&huart2, response1, sizeof(response1), 0xff);
	HAL_UART_Transmit(&huart1, response1, strlen(response1), 0xff);
	HAL_Delay(1000);


	char cmd2[] = "ATS24=1\r";
	char response2[255] = {0};
	HAL_UART_Transmit(&huart2, cmd2, strlen(cmd2), 0xff);
	HAL_UART_Receive(&huart2, response2, sizeof(response2), 0xff);
	HAL_UART_Transmit(&huart1, response2, strlen(response2), 0xff);
	HAL_Delay(1000);

	char cmd5[] = "AT/C\r";
	char response5[255] = {0};
	HAL_UART_Transmit(&huart2, cmd5, strlen(cmd5), 0xff);
	HAL_UART_Receive(&huart2, response5, sizeof(response5), 0xff);
	HAL_UART_Transmit(&huart1, response5, strlen(response5), 0xff);
	HAL_Delay(1000);

	char cmd4[] = "AT/S\r";
	char response4[1024] = {0};
	HAL_UART_Transmit(&huart2, cmd4, strlen(cmd4), 0xff);
	HAL_UART_Receive(&huart2, response4, sizeof(response4), 0xff);
	HAL_UART_Transmit(&huart1, response4, strlen(response4), 0xff);
	HAL_Delay(1000);

	char cmd3[] = "ATO\r";
	char response3[255] = {0};
	HAL_UART_Transmit(&huart2, cmd3, strlen(cmd3), 0xff);
	HAL_UART_Receive(&huart2, response3, sizeof(response3), 0xff);
	HAL_UART_Transmit(&huart1, response3, strlen(response3), 0xff);

	uint8_t bla2[] = "finished.\r\n";
	HAL_UART_Transmit(&huart1, bla2, strlen(bla2), 0xff);*/
	/*
	HAL_Delay(1000);
	while(1) {
		char buffer[64] = {0};
		for(uint8_t i = 0; i < 64; i++) {
			memset(buffer, '.', i);
			uint8_t d = sprintf(buffer, "%d", i);
			buffer[i - 2] = '\r';
			buffer[i - 1] = '\n';
			HAL_UART_Transmit(&huart2, buffer, i, 0xff);
			HAL_Delay(20);
		}
	}*/
	HAL_Delay(5000);
	if (PMW3901_Init() == PMW_BROKEN_CONNECTION) {
		while (1) {
			HAL_GPIO_TogglePin(STATUS_GPIO_Port, STATUS_Pin);
			HAL_Delay(2000);
		}
	}
	distance_sensors_init();

	MPU9250_Init();
	MPU9250_ReadDataDMA();

	start_mavlink();

	/*
	 * Define PID banks
	 */
	rate_pitch_PID.KP = 1.0f;
	rate_roll_PID.KP = 1.0f;
	rate_yaw_PID.KP = 2.0f;

	//rate_pitch_PID.KI = 0.25f;
	//rate_roll_PID.KI = 0.25f;

	//rate_pitch_PID.KD = 0.25f;
	//rate_roll_PID.KD = 0.25f;

	angle_pitch_PID.KP = 3.0f;
	angle_roll_PID.KP = 3.0f;
	angle_pitch_PID.KI = 0.005f;
	angle_roll_PID.KI = 0.005f;

	uint32_t last_heartbeat = 0;
	uint32_t last_sensors_update = 0;
	uint32_t last_telemetry_update = 0;
	uint32_t last_distance_update = 0;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (HAL_GetTick() - last_heartbeat > 1000 && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
			uint8_t buffer[64] = {0};
			mavlink_message_t msg;
			mavlink_msg_heartbeat_pack(1, 100, &msg, MAV_TYPE_QUADROTOR,
					MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_SAFETY_ARMED, 0,
					MAV_STATE_STANDBY);
			uint8_t len = mavlink_msg_to_send_buffer(buffer, &msg);
			HAL_UART_Transmit(&huart2, buffer, len, 0xff);
			last_heartbeat = HAL_GetTick();
		}
		if (HAL_GetTick() - last_telemetry_update > 100) {
			uint8_t buffer[64] = {0};
			mavlink_message_t msg;
			mavlink_msg_vision_position_estimate_pack(1, 100, &msg,
				HAL_GetTick(), pos_x, pos_y, altitude, r_y, r_x, r_z);
			uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
			HAL_UART_Transmit(&huart2, buffer, len, 0xff);
			last_telemetry_update = HAL_GetTick();
		}
		if(HAL_GetTick() - last_distance_update > 100 && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
			uint8_t buffer[64] = {0};
			mavlink_message_t msg;
			mavlink_msg_distance_sensor_pack(1, 100, &msg, HAL_GetTick(), 30,
					1500, distance_front, MAV_DISTANCE_SENSOR_LASER, 0,
					MAV_SENSOR_ROTATION_NONE, 0);
			uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
			HAL_UART_Transmit(&huart2, buffer, len, 0xff);
			last_distance_update = HAL_GetTick();
		}
		if (HAL_GetTick() - last_sensors_update > 10) {
			int16_t deltaX, deltaY;
			//uint32_t dt = HAL_GetTick() - last_sensors_update;
			if (PMW_OK == PMW3901_Read_Motion(&deltaX, &deltaY)) {
				pos_x += deltaX;
				pos_y += deltaY;
			}
			distance_front = read_front();
			altitude = read_height();
			last_sensors_update = HAL_GetTick();
		}
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* USER CODE BEGIN 4 */
float update_pid(PID *m_pid, float setpoint, float measurement) {
	float error = setpoint - measurement;

	float P = error * m_pid->KP;

	m_pid->_I += error;
	float I = m_pid->_I * m_pid->KI;

	if (m_pid->_I > m_pid->I_MAX)
		m_pid->_I = m_pid->I_MAX;
	if (m_pid->_I < -(m_pid->I_MAX))
		m_pid->_I = -(m_pid->I_MAX);

	float D = (error - m_pid->_D) * m_pid->KD;
	m_pid->_D = error;

	return P + I + D;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		DBG_A_GPIO_Port->ODR |= DBG_A_Pin;

		// runs every ARR (5000) microseconds
		float ay = atan2(MPU9250.acc.x,
				sqrt(pow(MPU9250.acc.y, 2) + pow(MPU9250.acc.z, 2)))
				* 180/ M_PI;
		float ax = atan2(MPU9250.acc.y,
				sqrt(pow(MPU9250.acc.x, 2) + pow(MPU9250.acc.z, 2)))
				* 180/ M_PI;

		// angles based on gyro (deg/s)
		r_x = r_x + MPU9250.gyro.x / 200.0f;
		r_y = r_y - MPU9250.gyro.y / 200.0f;
		r_z = r_z + MPU9250.gyro.z / 200.0f;

		// complementary filter
		// tau = DT*(A)/(1-A)
		// = 0.48sec
		r_x = r_x * 0.9f + ax * 0.1f;
		r_y = r_y * 0.9f + ay * 0.1f;

		float a_error_pitch = update_pid(&angle_pitch_PID, pitch, r_x);
		float a_error_roll = update_pid(&angle_roll_PID, roll, -r_y);

		float error_pitch = update_pid(&rate_pitch_PID, a_error_pitch, MPU9250.gyro.x);
		float error_roll = update_pid(&rate_roll_PID, a_error_roll, MPU9250.gyro.y);
		float error_yaw = update_pid(&rate_yaw_PID, yaw, MPU9250.gyro.z);

		float m1_out = throttle - error_pitch - error_roll + error_yaw;
		float m2_out = throttle - error_pitch + error_roll - error_yaw;
		float m3_out = throttle + error_pitch + error_roll + error_yaw;
		float m4_out = throttle + error_pitch - error_roll - error_yaw;

		m1_out = constrain(m1_out, 0, htim1.Instance->ARR);
		m2_out = constrain(m2_out, 0, htim1.Instance->ARR);
		m3_out = constrain(m3_out, 0, htim1.Instance->ARR);
		m4_out = constrain(m4_out, 0, htim1.Instance->ARR);

		if(enabled) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m1_out);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m2_out);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m3_out);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m4_out);
		} else {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		}


		DBG_A_GPIO_Port->ODR &= ~DBG_A_Pin;
	}
}
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
	while (1) {
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
