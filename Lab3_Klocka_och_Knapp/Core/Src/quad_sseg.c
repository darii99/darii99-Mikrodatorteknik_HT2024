

#include "main.h"
#include "quad_sseg.h"



// from datasheet, p.8
uint8_t seg_array[] =
{
		0x3F, // 0
		0x06, // 1
		0x5B, // 2
		0x4F, // 3
		0x66, // 4
		0x6D, // 5
		0x7D, // 6
		0x07, // 7
		0x7F, // 8
		0x6F, // 9
		0x77, // A
		0x7C, // b
		0x58, // c
		0x5E, // d
		0x79, // E
		0x71, // f
		0x00, // ' '
		0x40  // '-'
};


/*
 * Depending on CPU speed this value might need changing.
 *
 * F401RE	- 84 MHz (100 seems to work)
 * WB55G	- 32 MHz (100 seems to work)
 *
 */
#define DELAY_NOP_COUNTER 250

void TM1637_Delay()
{
	for (int i = 0; i < DELAY_NOP_COUNTER; i++)
	{
		__NOP();
	}
}


void TM1637_Strt()
{
	HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_SET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_RESET);	TM1637_Delay();
}

void TM1637_Stop()
{
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_RESET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_SET);	TM1637_Delay();
}

void TM1637_WriteByte(uint8_t byte)
{
	for (uint8_t c = 0; c < 8; c++) {
		HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);
		if ( (byte & 0x01) == 0 )
		{
			HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_SET);
		}
		byte = byte >> 1;
		TM1637_Delay();
		HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
		TM1637_Delay();
	}

	HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);
	TM1637_Delay();
	while( HAL_GPIO_ReadPin(SEG_DIO_GPIO_Port, SEG_DIO_Pin) == GPIO_PIN_SET )
	{
		// Wait here until it goes low
	}
	HAL_GPIO_WritePin(SEG_DIO_GPIO_Port, SEG_DIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
	TM1637_Delay();
}


void TM1637_WriteCmnd(uint8_t cmd)
{
	TM1637_Strt();
	TM1637_WriteByte(cmd);
	TM1637_Stop();
}

void TM1637_WriteData(uint8_t addr, uint8_t data)
{
	TM1637_Strt();
	TM1637_WriteByte(addr);
	TM1637_WriteByte(data);
	TM1637_Stop();
}




void qs_put_bitmasks(uint8_t bm0, uint8_t bm1, uint8_t bm2, uint8_t bm3)
{
	TM1637_WriteCmnd(0x44);
	TM1637_WriteData(0xC0, bm0);
	TM1637_WriteData(0xC1, bm1);
	TM1637_WriteData(0xC2, bm2);
	TM1637_WriteData(0xC3, bm3);
	TM1637_WriteCmnd(0x8A);
}



void qs_put_bitmask_at(uint8_t bm, uint8_t loc)
{
	if ( 0 <= loc && loc <= 3)
	{
		loc += 0xC0;
		TM1637_WriteCmnd(0x44);
		TM1637_WriteData(loc, bm);
		TM1637_WriteCmnd(0x8A);
	}
}


void qs_put_digits(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t colon)
{
	qs_put_bitmasks(seg_array[d0],
					seg_array[d1] | (colon ? 0x80 : 0x00),
					seg_array[d2],
					seg_array[d3]);
}



void qs_put_big_num(uint16_t big_num)
{
	uint8_t ones = (big_num /    1) % 10;
	uint8_t tens = (big_num /   10) % 10;
	uint8_t huns = (big_num /  100) % 10;
	uint8_t thus = (big_num / 1000) % 10;

	if (thus == 0)
	{
		thus = QS_BLANK;
		if (huns == 0)
		{
			huns = QS_BLANK;
			if (tens == 0)
			{
				tens = QS_BLANK;
			}
		}
	}
	qs_put_digits(thus, huns, tens, ones, 0);
}

