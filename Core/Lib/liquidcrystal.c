/*
 * liquidcrystal.c
 *
 *  Created on: Mar 28, 2024
 *      Author: User
 */

#include "liquidcrystal_i2c.h"

extern I2C_HandleTypeDef hi2c3;

uint8_t dpFunction;
uint8_t dpControl;
uint8_t dpMode;
uint8_t dpRows;
uint8_t dpBacklight;
uint32_t delay_start = 0xffffffff;
uint8_t init_fsm_case = 0;

static void
SendCommand (uint8_t);
static void
SendChar (uint8_t);
static void
Send (uint8_t, uint8_t);
static void
Write4Bits (uint8_t);
static void
ExpanderWrite (uint8_t);
static void
PulseEnable (uint8_t);
static void
DelayInit (void);
static void
DelayUS (uint32_t);
static uint8_t
DelayUS_nb (uint32_t);

static char snum[4];
static char snum_time[4];
uint16_t display_fsm_case = 0;
uint32_t display_delay = 0xFFFFFFFF;
char *tactic_side = "blue  #";
char *tactic_side_short = "b #";
static char tactic_number[2];
uint8_t prev_time = 0;
extern tactic_num tactic;

void
display_fsm ()
{
	/* Display FSM */
	switch (display_fsm_case)
		{

		/* Inicijalizacija displeja */
		case 0:
			if (HD44780_Init (2))
				display_fsm_case = 1;
			break;

			/* Ispis pre cinca */
		case 1:
			HD44780_NoBacklight ();
			HD44780_Clear ();
			HD44780_SetCursor (2, 0);
			HD44780_PrintStr ("+381Robotics");
			HD44780_Backlight ();
			display_fsm_case = 2;
			break;

			/* Biranje taktike i cekanje pocetka */
		case 2:
			HD44780_SetCursor (3, 1);
			if (tactic.side == 1)
				{
					tactic_side = " blue   #";
					tactic_side_short = "B";
				}
			else
				{
					tactic_side = "yellow  #";
					tactic_side_short = "Y";
				}
			HD44780_PrintStr (tactic_side);
			itoa (tactic.num, tactic_number, 10);
			HD44780_PrintStr (tactic_number);
			break;

			/* Ispis celog displeja */
		case 3:
			display_write_all (get_points (), get_time_s (), tactic_side_short, tactic_number);
			display_fsm_case = 4;
			break;

			/* Ispisivanje samo brojeva svake sekunde */
		case 4:
			if (prev_time != get_time_s ())
				{
					prev_time = get_time_s ();
					display_write_numbers (get_points (), get_time_s ());
				}
			break;

			/* Isteklo vreme, nista vise ne ispisuj */
		case 5:
			break;
		}
}

void
display_write_all (uint8_t points, uint8_t time, char *tactic_side, char *tactic_num)
{
	itoa (points, snum, 10);
	itoa (time, snum_time, 10);

	HD44780_Clear ();
	HD44780_SetCursor (0, 0);
	HD44780_PrintStr ("+381");
	HD44780_SetCursor (6, 1);
	HD44780_PrintStr ("points:");
	HD44780_SetCursor (0, 1);
	HD44780_PrintStr (tactic_side);
	HD44780_PrintStr (tactic_num);
	HD44780_SetCursor (8, 0);
	HD44780_PrintStr ("time:");
	display_write_numbers (points, time);
}

void
display_write_numbers (uint8_t points, uint8_t time)
{
	itoa (points, snum, 10);
	itoa (time, snum_time, 10);

	if (points < 10)
		HD44780_SetCursor (15, 1);
	else if (points < 100)
		HD44780_SetCursor (14, 1);
	else
		HD44780_SetCursor (13, 1);
	HD44780_PrintStr (snum);
	if (time < 10)
		HD44780_SetCursor (15, 0);
	else if (time < 100)
		HD44780_SetCursor (14, 0);
	else
		HD44780_SetCursor (13, 0);
	HD44780_PrintStr (snum_time);
}

uint8_t
HD44780_Init (uint8_t rows)
{
	switch (init_fsm_case)
		{
		case 0:
			dpRows = rows;
			dpBacklight = LCD_BACKLIGHT;
			dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
			if (dpRows > 1)
				{
					dpFunction |= LCD_2LINE;
				}
			else
				{
					dpFunction |= LCD_5x10DOTS;
				}
			DelayInit ();
			init_fsm_case = 1;
			break;

		case 1:
			/* Wait for initialization */
			if (DelayUS_nb (50000))
				init_fsm_case = 2;
			break;

		case 2:
			ExpanderWrite (dpBacklight);
			init_fsm_case = 3;
			break;

		case 3:
			if (DelayUS_nb (1000000))
				init_fsm_case = 4;
			break;

		case 4:
			/* 4bit Mode */
			Write4Bits (0x03 << 4);
			init_fsm_case = 5;
			break;

		case 5:
			if (DelayUS_nb (4500))
				init_fsm_case = 6;
			break;

		case 6:
			Write4Bits (0x03 << 4);
			init_fsm_case = 7;
			break;

		case 7:
			if (DelayUS_nb (4500))
				init_fsm_case = 8;
			break;

		case 8:
			Write4Bits (0x03 << 4);
			init_fsm_case = 9;
			break;

		case 9:
			if (DelayUS_nb (4500))
				init_fsm_case = 10;
			break;

		case 10:
			Write4Bits (0x02 << 4);
			init_fsm_case = 11;
			break;

		case 11:
			if (DelayUS_nb (100))
				init_fsm_case = 12;
			break;

		case 12:
			/* Display Control */
			SendCommand (LCD_FUNCTIONSET | dpFunction);
			dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
			HD44780_Display ();
			HD44780_Clear ();
			/* Display Mode */
			dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
			SendCommand (LCD_ENTRYMODESET | dpMode);
			init_fsm_case = 13;
			break;

		case 13:
			if (DelayUS_nb (4500))
				init_fsm_case = 14;
			break;

		case 14:
			HD44780_Home ();
			return 1;
			break;

		}
	return 0;
}

void
HD44780_Clear ()
{
	SendCommand (LCD_CLEARDISPLAY);
	DelayUS (2000);
}

void
HD44780_Home ()
{
	SendCommand (LCD_RETURNHOME);
	DelayUS (2000);
}

void
HD44780_SetCursor (uint8_t col, uint8_t row)
{
	int row_offsets[] =
		{ 0x00, 0x40, 0x14, 0x54 };
	if (row >= dpRows)
		{
			row = dpRows - 1;
		}
	SendCommand (LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void
HD44780_NoDisplay ()
{
	dpControl &= ~LCD_DISPLAYON;
	SendCommand (LCD_DISPLAYCONTROL | dpControl);
}

void
HD44780_Display ()
{
	dpControl |= LCD_DISPLAYON;
	SendCommand (LCD_DISPLAYCONTROL | dpControl);
}

void
HD44780_NoCursor ()
{
	dpControl &= ~LCD_CURSORON;
	SendCommand (LCD_DISPLAYCONTROL | dpControl);
}

void
HD44780_Cursor ()
{
	dpControl |= LCD_CURSORON;
	SendCommand (LCD_DISPLAYCONTROL | dpControl);
}

void
HD44780_NoBlink ()
{
	dpControl &= ~LCD_BLINKON;
	SendCommand (LCD_DISPLAYCONTROL | dpControl);
}

void
HD44780_Blink ()
{
	dpControl |= LCD_BLINKON;
	SendCommand (LCD_DISPLAYCONTROL | dpControl);
}

void
HD44780_ScrollDisplayLeft (void)
{
	SendCommand (LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void
HD44780_ScrollDisplayRight (void)
{
	SendCommand (LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void
HD44780_LeftToRight (void)
{
	dpMode |= LCD_ENTRYLEFT;
	SendCommand (LCD_ENTRYMODESET | dpMode);
}

void
HD44780_RightToLeft (void)
{
	dpMode &= ~LCD_ENTRYLEFT;
	SendCommand (LCD_ENTRYMODESET | dpMode);
}

void
HD44780_AutoScroll (void)
{
	dpMode |= LCD_ENTRYSHIFTINCREMENT;
	SendCommand (LCD_ENTRYMODESET | dpMode);
}

void
HD44780_NoAutoScroll (void)
{
	dpMode &= ~LCD_ENTRYSHIFTINCREMENT;
	SendCommand (LCD_ENTRYMODESET | dpMode);
}

void
HD44780_CreateSpecialChar (uint8_t location, uint8_t charmap[])
{
	location &= 0x7;
	SendCommand (LCD_SETCGRAMADDR | (location << 3));
	for (int i = 0; i < 8; i++)
		{
			SendChar (charmap[i]);
		}
}

void
HD44780_PrintSpecialChar (uint8_t index)
{
	SendChar (index);
}

void
HD44780_LoadCustomCharacter (uint8_t char_num, uint8_t *rows)
{
	HD44780_CreateSpecialChar (char_num, rows);
}

void
HD44780_PrintStr (const char c[])
{
	while (*c)
		SendChar (*c++);
}

void
HD44780_SetBacklight (uint8_t new_val)
{
	if (new_val)
		HD44780_Backlight ();
	else
		HD44780_NoBacklight ();
}

void
HD44780_NoBacklight (void)
{
	dpBacklight = LCD_NOBACKLIGHT;
	ExpanderWrite (0);
}

void
HD44780_Backlight (void)
{
	dpBacklight = LCD_BACKLIGHT;
	ExpanderWrite (0);
}

static void
SendCommand (uint8_t cmd)
{
	Send (cmd, 0);
}

static void
SendChar (uint8_t ch)
{
	Send (ch, RS);
}

static void
Send (uint8_t value, uint8_t mode)
{
	uint8_t highnib = value & 0xF0;
	uint8_t lownib = (value << 4) & 0xF0;
	Write4Bits ((highnib) | mode);
	Write4Bits ((lownib) | mode);
}

static void
Write4Bits (uint8_t value)
{
	ExpanderWrite (value);
	PulseEnable (value);
}

static void
ExpanderWrite (uint8_t _data)
{
	uint8_t data = _data | dpBacklight;
	HAL_I2C_Master_Transmit (&hi2c3, DEVICE_ADDR, (uint8_t*) &data, 1, 10);
}

static void
PulseEnable (uint8_t _data)
{
	ExpanderWrite (_data | ENABLE);
	DelayUS (20);

	ExpanderWrite (_data & ~ENABLE);
	DelayUS (20);
}

static void
DelayInit (void)
{
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

	DWT->CYCCNT = 0;

	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
}

static void
DelayUS (uint32_t us)
{
	uint32_t cycles = (SystemCoreClock / 1000000L) * us;
	uint32_t start = DWT->CYCCNT;
	volatile uint32_t cnt;

	do
		{
			cnt = DWT->CYCCNT - start;
		}
	while (cnt < cycles);
}

static uint8_t
DelayUS_nb (uint32_t us)
{
	uint32_t cycles = (SystemCoreClock / 1000000L) * us;
	delay_start = uint_min (delay_start, DWT->CYCCNT);

	if (DWT->CYCCNT <= delay_start + cycles)
		return 0;

	delay_start = 0xffffffff;
	return 1;
}
