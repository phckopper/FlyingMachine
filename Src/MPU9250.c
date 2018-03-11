/*
 * MPU9250.c
 *
 *  Created on: Feb 13, 2018
 *      Author: phckopper
 */
#include <stdbool.h>

#include "main.h"
#include "stm32f1xx_hal.h"

#include "MPU9250.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

uint8_t buffer[14];
uint8_t dummy[14] = {0};

static uint8_t _read_byte(uint8_t reg) {
	uint8_t tmp;
	reg |= 0x80;

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&reg, sizeof(reg), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t *)&tmp, sizeof(tmp), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);

	return tmp;
}

static void _write_byte(uint8_t reg, uint8_t value) {
	uint8_t tmp[] = {reg, value};

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&tmp, sizeof(tmp), HAL_MAX_DELAY);

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
}

uint8_t MPU9250_Init(void) {
	//memset(&MPU9250, 0, sizeof(MPU9250));
	if(MPU9250_CheckConnection() != 0x73) {
		return -1;
	}

	_write_byte(PWR_MGMNT_1, PWR_RESET);
	HAL_Delay(50);
	_write_byte(USER_CTRL, I2C_IF_DIS);
	HAL_Delay(1);
	_write_byte(PWR_MGMNT_1, CLOCK_SEL_PLL);
	HAL_Delay(1);
	_write_byte(PWR_MGMNT_2, SEN_ENABLE);
	HAL_Delay(1);
	_write_byte(ACCEL_CONFIG, ACCEL_FS_SEL_4G);
	HAL_Delay(1);
	_write_byte(GYRO_CONFIG, GYRO_FS_SEL_500DPS);
	HAL_Delay(100);

	MPU9250._calibrated = false;
	const size_t iterations = 5000;
	for(size_t i = 0; i < iterations; i++) {
		MPU9250_ReadDataSync();
		MPU9250.gyro_bias.x += MPU9250.gyro.x;
		MPU9250.gyro_bias.y += MPU9250.gyro.y;
		MPU9250.gyro_bias.z += MPU9250.gyro.z;
		HAL_Delay(1);
	}
	MPU9250.gyro_bias.x /= iterations;
	MPU9250.gyro_bias.y /= iterations;
	MPU9250.gyro_bias.z /= iterations;
	MPU9250._calibrated = true;

	//HAL_DMA_RegisterCallback(&hdma_spi1_rx, HAL_DMA_XFER_CPLT_CB_ID, MPU9250_DataReceived);

	return 0;
}

void MPU9250_ReadDataDMA(void) {
	//DBG_B_GPIO_Port->ODR ^= DBG_B_Pin;
	uint8_t reg = ACCEL_OUT | 0x80;

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&reg, sizeof(reg), HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive_DMA(&hspi1, dummy, buffer, sizeof(buffer));

}

void MPU9250_ReadDataSync(void) {
	uint8_t reg = ACCEL_OUT | 0x80;
	uint8_t buffer[14];

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, (uint8_t *)&reg, sizeof(reg), HAL_MAX_DELAY);

	HAL_SPI_Receive(&hspi1, (uint8_t *)&buffer, sizeof(buffer), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);

	int16_t ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	int16_t ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];

	int16_t gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	int16_t gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];

	MPU9250.acc.x = ax * (4.0f/32767.5f);
	MPU9250.acc.y = ay * (4.0f/32767.5f);
	MPU9250.acc.z = az * (4.0f/32767.5f);

	MPU9250.gyro.x = (gx * (500.0f/32767.5f));
	MPU9250.gyro.y = (gy * (500.0f/32767.5f));
	MPU9250.gyro.z = (gz * (500.0f/32767.5f));

	if(MPU9250._calibrated) {
		MPU9250.gyro.x -= MPU9250.gyro_bias.x;
		MPU9250.gyro.y -= MPU9250.gyro_bias.y;
		MPU9250.gyro.z -= MPU9250.gyro_bias.z;
	}
}

uint8_t MPU9250_CheckConnection(void) {
	return _read_byte(WHO_AM_I);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi) {
	UNUSED(hspi);
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);

	int16_t ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	int16_t ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];

	int16_t gx = (((int16_t)buffer[8]) << 8) | buffer[9];
	int16_t gy = (((int16_t)buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];

	MPU9250.acc.x = ax * (4.0f/32767.5f);
	MPU9250.acc.y = ay * (4.0f/32767.5f);
	MPU9250.acc.z = az * (4.0f/32767.5f);

	MPU9250.gyro.x = (gx * (500.0f/32767.5f));
	MPU9250.gyro.y = (gy * (500.0f/32767.5f));
	MPU9250.gyro.z = (gz * (500.0f/32767.5f));

	MPU9250.gyro.x -= MPU9250.gyro_bias.x;
	MPU9250.gyro.y -= MPU9250.gyro_bias.y;
	MPU9250.gyro.z -= MPU9250.gyro_bias.z;

	MPU9250_ReadDataDMA();
}

