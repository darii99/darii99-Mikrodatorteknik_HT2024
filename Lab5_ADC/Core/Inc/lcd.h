
#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "main.h"
#include <stdbool.h>


// D7 D6 D5 D4 BT  E RW RS




typedef struct
{
	I2C_HandleTypeDef	*	hi2c;
	uint8_t					device_address;
} TextLCDType;


/*
 * Initializes the device. It's put in 4 bit mode.
 * Convenience functions for sending a full byte is provided.
 *
 * Should probably be called (from main()) like this:
 *		TextLCD_Init(&hlcd, &hi2c1, 0x4E);
 */
void TextLCD_Init(TextLCDType *lcd, I2C_HandleTypeDef *hi2c, uint8_t DevAddress);

/*
 * Because there's no direct access to the backlight pin, this works
 * a bit weird. Whenever a message of any kind is sent to the device
 * the backlight bit is always sent as well.
 *
 * Using this function will make sure that the value of bt is used
 * the next time any data is sent to the device.
 *
 * Active high.
 */
void TextLCD_SetBacklightFlag(GPIO_PinState bt);





void TextLCD_Home		(TextLCDType * hlcd);
void TextLCD_Clear		(TextLCDType * hlcd);
void TextLCD_SetDDRAMAdr(TextLCDType * hlcd, uint8_t adr);
void TextLCD_Position	(TextLCDType * hlcd, int col, int row);
void TextLCD_PutChar	(TextLCDType * hlcd, char c);
void TextLCD_PutStr		(TextLCDType * hlcd, char * str);

//void TextLCD_Printf(TextLCDType *lcd, char *message, ...);

#endif /* INC_LCD_H_ */
