/*
 * OLED_SSD1306.c
 *
 *  Created on: June 19, 2024
 *      Author: Ganesh
 */

#include "OLED_SSD1306.h"
#include "MyFont.h"
#include <math.h>
#include <stdlib.h>

uint8_t OLED_Ram_Buffer[OLED_MAX_PAGES + 1][OLED_MAX_CLM + 1] = { 0 };

//used for setting up the display in page addressing mode
void OLED_Setup(I2C_HandleTypeDef *i2c) {

	// send all OLED commands for setup in one i2c transaction

#ifdef OLED_DISPLAY_128_64

	uint8_t Oled_Setup_Cmnds[] = { OLED_OFF_CMND, OLED_NORMAL_MODE,
	OLED_SETADD_MODE, OLED_PAGEADD_MODE,
	OLED_SET_MUX, OLED_128_64, OLED_SET_COMCONFIG,
	OLED_COMCONFIG_64, OLED_CHRG_PUMP1,
	OLED_CHRG_PUMP2, OLED_ON_CMND };

#elif  defined(OLED_DISPLAY_128_32)

	uint8_t Oled_Setup_Cmnds[] = { OLED_OFF_CMND, OLED_NORMAL_MODE,
	OLED_SETADD_MODE, OLED_PAGEADD_MODE,
	OLED_SET_MUX, OLED_128_32, OLED_SET_COMCONFIG,
	OLED_COMCONFIG_32, OLED_CHRG_PUMP1,
	OLED_CHRG_PUMP2, OLED_ON_CMND };

#else
	#error "Define the type of OLED display you are using"
#endif

	HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_CMND_REG, 1, Oled_Setup_Cmnds,
			sizeof(Oled_Setup_Cmnds), 1000);

	OLED_ClearDisplay(i2c);

}

//used for setting cursor to given page & clm location
void OLED_SetCursor(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t clm) {

	if (page >= 0 && page <= OLED_MAX_PAGES) {
		uint8_t data = ((0xB0) | (page & 0x0f));
		HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_CMND_REG, 1, &data, 1, 1000); // set page
		data = ((0x00) | (clm & 0x0f));
		HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_CMND_REG, 1, &data, 1, 1000); // set column lower nibble
		data = ((0x10) | ((clm >> 4) & 0x0f));
		HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_CMND_REG, 1, &data, 1, 1000); // set column higher nibble
	}

}

//used for clearing the display
void OLED_ClearDisplay(I2C_HandleTypeDef *i2c) {

	uint8_t data = 0x00;

	for (uint8_t page = 0; page <= OLED_MAX_PAGES; page++) {
		OLED_SetCursor(i2c, page, 0);
		for (uint8_t clm = 0; clm <= OLED_MAX_CLM; clm++) {
			HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_DATA_REG, 1, &data, 1, 1000);
		}
	}

}

//used for printing a single character
void OLED_PrintChar(I2C_HandleTypeDef *i2c, char data, uint8_t pg, uint8_t clm) {

	uint8_t ascii_data = data - 12; //doing this to access myfont array element by ascii value

	OLED_SetCursor(i2c, pg, clm);

	for (uint8_t indx = 0; indx <= 22; indx += 2) {
		HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_DATA_REG, 1,
				&MyFont[ascii_data][indx], 1, 1000);
	}

	OLED_SetCursor(i2c, pg + 1, clm);

	for (uint8_t indx = 1; indx <= 23; indx += 2) {
		HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_DATA_REG, 1,
				&MyFont[ascii_data][indx], 1, 1000);
	}

}

//used for printing a string
void OLED_Print(I2C_HandleTypeDef *i2c, char *str, uint8_t pg, uint8_t clm) {

	while (*str) {
		OLED_PrintChar(i2c, *str++, pg, clm);

		clm += 12; //to write the next character we move clm
		if (clm >= 120) {
			break; //to make sure we are not out of the display
		}

	}

}

//used for drawing a circle
void OLED_DrawCircle(I2C_HandleTypeDef *i2c, uint8_t centerX, uint8_t centerY,
		uint8_t radius, Oled_Color clr) {

	//	float x_cord = 0, y_cord = 0;

	for (float theta_value = 0; theta_value <= 360; theta_value++) {
		/*
		 * Calculation of circle coordinates, instead of creating separate variables I am doing all of this
		 * in the DrawPixel function call itself.
		 *
		 * theta_value *= (M_PI / 180.0);	//rad to deg
		 * x_cord = cos(theta_value*(M_PI / 180.0)) * radius;
		 * y_cord = sin(theta_value*(M_PI / 180.0)) * radius;
		 *
		 * */

		OLED_SinglePixel(i2c,
				(uint8_t) ((cos(theta_value * (PI_BY_180)) * radius) + centerX),
				(uint8_t) ((sin(theta_value * (PI_BY_180)) * radius) + centerY),
				clr);
	}

}

//used for lighting up a single pixel
void OLED_SinglePixel(I2C_HandleTypeDef *i2c, uint8_t clm, uint8_t row,
		Oled_Color clr) {

	uint8_t page = (uint8_t) (row / 8);
	uint8_t oled_data = OLED_Ram_Buffer[page][clm];

	if (clr==WHITE) {
		oled_data |= (1 << (row - (page * 8)));
	} else {
		oled_data &= ~(1 << (row - (page * 8)));
	}

	/*
	 * writing data to the software buffer so that next time
	 * when writing new data previous data won't get deleted
	 */

	OLED_Ram_Buffer[page][clm] = oled_data;
//	OLED_SetCursor(i2c, page, clm);
//	HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_DATA_REG, 1, &oled_data, 1, 1000);

}

//use this if you want to write something in oled RAM buffer in software
void OLED_Writeto_RAMBuffer(I2C_HandleTypeDef *i2c, uint8_t data, uint8_t page,
		uint8_t clm) {
	/*
	 * This function will be useful for a user in case user wants to write something
	 * in the OLED RAM buffer created in program(not a buffer in actual Oled display hardware)
	 * so that all of the data can be written first into the software buffer and when the
	 * user wants to finally display it then only it will transmitted via uC to Oled and
	 * this will save time since your are not writing the data every time in the actual hardware
	 * which requires some time to be transmitted to the Oled hardware.
	 *
	 * */
	OLED_Ram_Buffer[page][clm] |= data;
}

//use this to update data from software RAM buffer to OLED screen
void OLED_UpdateScreen(I2C_HandleTypeDef *i2c) {
	uint8_t *temp_ptr = NULL;
	for (uint8_t page = 0; page <= OLED_MAX_PAGES; page++) {

		OLED_SetCursor(i2c, page, 0);
		temp_ptr = &OLED_Ram_Buffer[page][0];

		HAL_I2C_Mem_Write(i2c, OLED_ADD, OLED_DATA_REG, 1, temp_ptr, 128, 1000);
	}
}

//create oled random function
void OLED_JhilmilScreen(I2C_HandleTypeDef *i2c) {
	for (uint8_t page = 0; page <= OLED_MAX_PAGES; page++) {
		for (uint8_t clm = 0; clm <= OLED_MAX_CLM; clm++) {
			OLED_Ram_Buffer[page][clm] = (uint8_t) (rand() % 256);
		}
	}
	OLED_UpdateScreen(i2c);
	OLED_ClearRAMBuffer();
}

//clear RAM buffer
void OLED_ClearRAMBuffer(void) {
	for (uint8_t page = 0; page <= OLED_MAX_PAGES; page++) {
		for (uint8_t clm = 0; clm <= OLED_MAX_CLM; clm++) {
			OLED_Ram_Buffer[page][clm] = 0;
		}
	}
}
