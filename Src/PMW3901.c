/*
 * PMW3901.c
 *
 *  Created on: Feb 8, 2018
 *      Author: phckopper
 */
#include "PMW3901.h"

#include "main.h"
#include "spi.h"
#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi2;

uint8_t RAW_FRAME[FRAME_SIZE] = {0};

void HAL_Delay_Microseconds(volatile uint32_t micros) {
	micros *= (HAL_RCC_GetHCLKFreq() / 1000000)/5;

	while (micros--);
}

static uint8_t _read_byte(uint8_t reg) {
	uint8_t tmp;

	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay_Microseconds(50);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)&reg, sizeof(reg), HAL_MAX_DELAY);
	HAL_Delay_Microseconds(50);

	HAL_SPI_Receive(&hspi2, (uint8_t *)&tmp, sizeof(tmp), HAL_MAX_DELAY);
	HAL_Delay_Microseconds(200);
	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_SET);
	HAL_Delay_Microseconds(200);

	return tmp;
}

static void _write_byte(uint8_t reg, uint8_t value) {
	uint8_t tmp[] = {reg | 0x80u, value};

	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay_Microseconds(50);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)&tmp, sizeof(tmp), HAL_MAX_DELAY);
	HAL_Delay_Microseconds(50);

	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_SET);
	HAL_Delay_Microseconds(200);
}

PMW_Return_Code PMW3901_Get_Frame(void) {
	_write_byte(0x7f, 0x07);
	_write_byte(0x41, 0x1d);
	_write_byte(0x4c, 0x00);
	_write_byte(0x7f, 0x08);
	_write_byte(0x6a, 0x38);
	_write_byte(0x7f, 0x00);
	_write_byte(0x55, 0x04);
	_write_byte(0x40, 0x80);
	_write_byte(0x4d, 0x11);
	_write_byte(0x70, 0x00);
	_write_byte(0x58, 0xff);

	while((_read_byte(0x59) & 0xc0) != 0xc0);


	for(uint16_t i = 0; i < FRAME_SIZE; i ++) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		uint8_t data;
		while(1) {
			data = _read_byte(0x58);
			if(data & 0x40) {
				break;
			}
		}
		data = (data << 2) | ((_read_byte(0x58) & 0x0c) >> 2);
		RAW_FRAME[i] = data;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
	return PMW_OK;

}

uint8_t PMW3901_Debug(void) {
	_write_byte(0x15, 0x00);
	HAL_Delay(20);
	return _read_byte(0x15);
}

uint8_t PMW3901_RawData_Sum(void) {
	return _read_byte(0x08);
}

PMW_Return_Code PMW3901_Read_Motion(int16_t *deltaX, int16_t *deltaY) {
	if(!(_read_byte(0x02) & 0x80)) {
		return PMW_INVALID_DATA;
	}
	uint8_t deltaXL = _read_byte(0x03);
	uint8_t deltaXH = _read_byte(0x04);
	uint8_t deltaYL = _read_byte(0x05);
	uint8_t deltaYH = _read_byte(0x06);
	if(_read_byte(0x07) < 0x19 || _read_byte(0x0c) == 0x1F) {
		*deltaX = 0;
		*deltaY = 0;
		return PMW_OK;
	}
	*deltaX = ((int16_t) deltaXH << 8) | deltaXL;
	*deltaY = ((int16_t) deltaYH << 8) | deltaYL;

	return PMW_OK;
}

PMW_Return_Code PMW3901_Init(void) {
	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	_write_byte(0x3a, 0x5a); // reset
	HAL_Delay(5);

	if(_read_byte(0x00) != 0x49 || _read_byte(0x5f) != 0xb6) {
		return PMW_BROKEN_CONNECTION;
	}

	_read_byte(0x02);
	_read_byte(0x03);
	_read_byte(0x04);
	_read_byte(0x05);
	_read_byte(0x06);
	HAL_Delay(1);

	_write_byte(0x7f, 0x00);
	_write_byte(0x55, 0x01);
	_write_byte(0x50, 0x07);
	_write_byte(0x7f, 0x0e);
	_write_byte(0x43, 0x10);
	if(_read_byte(0x67) & 0x80) {
		_write_byte(0x48, 0x04);
	} else {
		_write_byte(0x48, 0x02);
	}
	_write_byte(0x7f, 0x00);
	_write_byte(0x51, 0x7b);
	_write_byte(0x50, 0x00);
	_write_byte(0x55, 0x00);
	_write_byte(0x7f, 0x0e);
	if(_read_byte(0x73) == 0x00) {
		uint8_t c1 = _read_byte(0x70);
		if(c1 <= 28) {
			c1 += 14;
		}
		else {
			c1 += 11;
		}
		if(c1 > 0x3f) {
			c1 = 0x3f;
		}
		uint8_t c2 = _read_byte(0x71);
		c2 = (c2 * 45)/100;
		_write_byte(0x7f, 0x00);
		_write_byte(0x61, 0xad);
		_write_byte(0x51, 0x70);
		_write_byte(0x7f, 0x0e);
		_write_byte(0x70, c1);
		_write_byte(0x71, c2);
	}
	_write_byte(0x7f, 0x00);
	_write_byte(0x61, 0xad);
	_write_byte(0x7f, 0x03);
	_write_byte(0x40, 0x00);
	_write_byte(0x7f, 0x05);
	_write_byte(0x41, 0xb3);
	_write_byte(0x43, 0xf1);
	_write_byte(0x45, 0x14);
	_write_byte(0x5b, 0x32);
	_write_byte(0x5f, 0x34);
	_write_byte(0x7b, 0x08);
	_write_byte(0x7f, 0x06);
	_write_byte(0x44, 0x1b);
	_write_byte(0x40, 0xbf);
	_write_byte(0x4e, 0x3f);
	_write_byte(0x7f, 0x08);
	_write_byte(0x65, 0x20);
	_write_byte(0x6a, 0x18);
	_write_byte(0x7f, 0x09);
	_write_byte(0x4f, 0xaf);
	_write_byte(0x5f, 0x40);
	_write_byte(0x48, 0x80);
	_write_byte(0x49, 0x80);
	_write_byte(0x57, 0x77);
	_write_byte(0x60, 0x78);
	_write_byte(0x61, 0x78);
	_write_byte(0x62, 0x08);
	_write_byte(0x63, 0x50);
	_write_byte(0x7f, 0x0a);
	_write_byte(0x45, 0x60);
	_write_byte(0x7f, 0x00);
	_write_byte(0x4d, 0x11);
	_write_byte(0x55, 0x80);
	_write_byte(0x74, 0x21);
	_write_byte(0x75, 0x1f);
	_write_byte(0x4a, 0x78);
	_write_byte(0x4b, 0x78);
	_write_byte(0x44, 0x08);
	_write_byte(0x45, 0x50);
	_write_byte(0x64, 0xff);
	_write_byte(0x65, 0x1f);
	_write_byte(0x7f, 0x14);
	_write_byte(0x65, 0x67);
	_write_byte(0x66, 0x08);
	_write_byte(0x63, 0x70);
	_write_byte(0x7f, 0x15);
	_write_byte(0x48, 0x48);
	_write_byte(0x7f, 0x07);
	_write_byte(0x41, 0x0d);
	_write_byte(0x43, 0x14);
	_write_byte(0x4b, 0x0e);
	_write_byte(0x45, 0x0f);
	_write_byte(0x44, 0x42);
	_write_byte(0x4c, 0x80);
	_write_byte(0x7f, 0x10);
	_write_byte(0x5b, 0x02);
	_write_byte(0x7f, 0x07);
	_write_byte(0x40, 0x41);
	_write_byte(0x70, 0x00);
	HAL_Delay(10);
	_write_byte(0x32, 0x44);
	_write_byte(0x7f, 0x07);
	_write_byte(0x40, 0x40);
	_write_byte(0x7f, 0x06);
	_write_byte(0x62, 0xf0);
	_write_byte(0x63, 0x00);
	_write_byte(0x7f, 0x0d);
	_write_byte(0x48, 0xc0);
	_write_byte(0x6f, 0xd5);
	_write_byte(0x7f, 0x00);
	_write_byte(0x5b, 0xa0);
	_write_byte(0x4e, 0xa8);
	_write_byte(0x5a, 0x50);
	_write_byte(0x40, 0x80);

	return PMW_OK;
}
