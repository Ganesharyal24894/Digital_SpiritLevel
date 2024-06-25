/*
 *OLED_SSD1306.h
 *
 *Created on: June 19, 2024
 *     Author: Ganesh
 */

#ifndef OLED_SSD1306_H_
#define OLED_SSD1306_H_
#include <stdint.h>
#include "main.h"
#define OLED_DISPLAY_128_64	//uncomment this if using 128 * 64 OLED display

//#define OLED_DISPLAY_128_32	//uncomment this if using 128 * 32 OLED display

#if defined(OLED_DISPLAY_128_64) && defined(OLED_DISPLAY_128_32)
#error "Please define only one type of display!!OLED display type ambiguous"

# elif defined(OLED_DISPLAY_128_64)
#define OLED_MAX_PAGES 7

# elif defined(OLED_DISPLAY_128_32)
#define OLED_MAX_PAGES 3

#else
#error "OLED Display type not defined"

#endif

typedef enum
{
	BLACK, WHITE
} Oled_Color;

#define OLED_ADD (0x3C << 1)
#define OLED_MAX_CLM 127
#define OLED_DATA_REG 0x40
#define OLED_CMND_REG 0x00
#define OLED_ON_CMND 0xAF
#define OLED_OFF_CMND 0xAE
#define OLED_DISPLAYFILL_ON 0xA5
#define OLED_DISPLAYFILL_OFF 0xA4
#define OLED_NORMAL_MODE 0xA6
#define OLED_INVERSE_MODE 0xA7
#define OLED_SETADD_MODE 0x20
#define OLED_PAGEADD_MODE 0x02
//
#define OLED_MIRROR_VERT 0xC0
#define OLED_SET_MUX 0xA8
#define OLED_128_32 0x1F
#define OLED_128_64 0x3F
#define OLED_SET_COMCONFIG 0xDA
#define OLED_COMCONFIG_32 0x02
#define OLED_COMCONFIG_64 0x12
#define OLED_CHRG_PUMP1 0x8D
#define OLED_CHRG_PUMP2 0x14
#define PI_BY_180 0.017453292519943295

void OLED_Setup(I2C_HandleTypeDef *i2c);
void OLED_SetCursor(I2C_HandleTypeDef *i2c, uint8_t page, uint8_t clm);
void OLED_ClearDisplay(I2C_HandleTypeDef *i2c);
void OLED_PrintChar(I2C_HandleTypeDef *i2c, char data, uint8_t pg, uint8_t clm);
void OLED_Print(I2C_HandleTypeDef *i2c, char *str, uint8_t pg, uint8_t clm);
void OLED_DrawCircle(I2C_HandleTypeDef *i2c, uint8_t centerX, uint8_t centerY,
		uint8_t radius, Oled_Color clr);
void OLED_SinglePixel(I2C_HandleTypeDef *i2c, uint8_t clm, uint8_t row,
		Oled_Color clr);
void OLED_Writeto_RAMBuffer(I2C_HandleTypeDef *i2c, uint8_t data, uint8_t page,
		uint8_t clm);
void OLED_UpdateScreen(I2C_HandleTypeDef *i2c);
void OLED_JhilmilScreen(I2C_HandleTypeDef *i2c);
void OLED_ClearRAMBuffer(void);

#endif /*OLED_SSD1306_H_ */
