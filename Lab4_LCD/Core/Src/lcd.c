
#include "lcd.h"

// first:   D7 D6 D5 D4 BT E  RW RS
// second:  D3 D2 D1 D0 BT E  RW RS


/*
 * my = mu = micro
 * Holds for an amount of microseconds.
 */
void My_Delay(uint32_t mysec)
{
	//HAL_Delay( 1 + (mysec / 1000) );
	uint16_t cr1 = TIM2->CR1;
	cr1 		 = cr1 | (0x0001 << 0);
	TIM2->CR1	 = cr1;
	while (TIM2->CNT < mysec) {
			//do nothing
		}
}

#define BIT_BT   0x08
#define BIT_E    0x04    // 0000 0100 == 0x08
#define BIT_RW   0x02
#define BIT_RS   0x01

#define INV_BT  ~BIT_BT
#define INV_E   ~BIT_E   // 1111 1011 == 0xFB
#define INV_RW  ~BIT_RW
#define INV_RS  ~BIT_RS

// Back light flag, is sent every command
GPIO_PinState BT = GPIO_PIN_SET;

// Always be writing! (can't really read from the device)
GPIO_PinState RW = GPIO_PIN_RESET;

/***************************************************************************
 *  Nibble should be in the MSB part as the lower four will be control.
 *
 * So the bits are
 *
 *  MSB b7   b6   b5   b4   b3   b2   b1   b0  LSB
 *
 *      D3   D2   D1   D0   BT    E   RW   RS
 *
 *  When sending a byte, use this twice, sending D7-D4 the first time
 *  and D3-D0 the second time.
 ****************************************************************************/
void TextLCD_SendNibbleWithPulseOnE(TextLCDType * hlcd, uint8_t data)
{
	/***** Put nibble when E is low *****/
	data = data & INV_E;
	HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->device_address, &data, 1, 1000);
	My_Delay(2000);

	/***** Now set E to high *****/
	data = data | BIT_E;
	HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->device_address, &data, 1, 1000);
	My_Delay(2000);

	/***** Then go low again *****/
	data = data & INV_E;
	HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->device_address, &data, 1, 1000);
}


void TextLCD_SendByte(
		TextLCDType   * hlcd,
		uint8_t         data,
		GPIO_PinState   RS)
{
	// Place the data bits in the top four bits. The lowest four will
	// be for control.
	uint8_t d_lo = (data & 0x0F) << 4;
	uint8_t d_hi = (data & 0xF0);

	// Set the control bits for the message.
	uint8_t ctrl = 0x00;
	ctrl = (BT == GPIO_PIN_SET) ? (ctrl | BIT_BT) : (ctrl & INV_BT);
	ctrl = (RS == GPIO_PIN_SET) ? (ctrl | BIT_RS) : (ctrl & INV_RS);
	ctrl = (RW == GPIO_PIN_SET) ? (ctrl | BIT_RW) : (ctrl & INV_RW);

	TextLCD_SendNibbleWithPulseOnE( hlcd, (d_hi | ctrl) );
	TextLCD_SendNibbleWithPulseOnE( hlcd, (d_lo | ctrl) );
}



void TextLCD_Init(
		TextLCDType         *   hlcd,
		I2C_HandleTypeDef   *   hi2c,
		uint8_t                 device_address)
{
	//uint32_t dly = 5 * 1000 * 1000; // 5 seconds
	//My_Delay(dly);
	hlcd->hi2c           = hi2c;
	hlcd->device_address = device_address;

	uint8_t data = 0x30; // b# 0011 1000
	uint8_t ctrl = 0x08;

	//My_Delay(70000);

	TextLCD_SendNibbleWithPulseOnE(hlcd, (data|ctrl) );
	TextLCD_SendNibbleWithPulseOnE(hlcd, (data|ctrl) );
	TextLCD_SendNibbleWithPulseOnE(hlcd, (data|ctrl) );

	data = 0x20;
	TextLCD_SendNibbleWithPulseOnE(hlcd, (data|ctrl) );

	// Finished setting up 4-bit mode. Let's configure display

	// hlcd, data, rs, rw
	TextLCD_SendByte(hlcd, 0x28, 0); //N=1 (2 line), F=0 (5x8)

	TextLCD_SendByte(hlcd, 0x0F, 0); //Display off, Cursor Off, Blink off
	TextLCD_SendByte(hlcd, 0x01, 0);
	My_Delay(5000);

	TextLCD_SendByte(hlcd, 0x06, 0);
	TextLCD_SendByte(hlcd, 0x0C, 0);

}


void TextLCD_SetBacklightFlag(GPIO_PinState bt)
{
	BT = bt;
}



void TextLCD_Home		(TextLCDType * hlcd)
{
	TextLCD_SendByte(hlcd, 0x02, 0);
	My_Delay(15.2);
}


void TextLCD_Clear		(TextLCDType * hlcd)
{
	TextLCD_SendByte(hlcd, 0x01, 0);
	My_Delay(2000);
}

void TextLCD_SetDDRAMAdr(TextLCDType * hlcd, uint8_t adr)
{
	TextLCD_SendByte(hlcd, 0x80 + adr, 0);
	My_Delay(37);
}


void TextLCD_Position	(TextLCDType * hlcd, int col, int row)
{
	if(row == 0)
			TextLCD_SetDDRAMAdr(hlcd, 0x00 + col);
	else
			TextLCD_SetDDRAMAdr(hlcd, 0x40 + col);
	My_Delay(37);
}

void TextLCD_PutChar	(TextLCDType * hlcd, char c)
{
	TextLCD_SendByte(hlcd, c, 1);
	My_Delay(37);
}


void TextLCD_PutStr		(TextLCDType * hlcd, char * str)
{
	int i = 0;
		while(str[i] != '\0')
		{
			TextLCD_PutChar(hlcd, str[i]);
			i++;
		}
}





#if 0
void TextLCD_Printf(TextLCDType *lcd, char *message, ...)
{

}
#endif

