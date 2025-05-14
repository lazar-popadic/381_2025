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
int8_t ready = 0;
extern int16_t tact_fsm_case;

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
static char x[6];
static char y[6];
static char phi[5];
static char snum_time[4];
static char snum_fsm[7];
uint16_t display_fsm_case = 0;
uint32_t display_delay = 0xFFFFFFFF;
char *tactic_side = "blue  #";
char *tactic_side_short = "b #";
static char tactic_number[2];
static uint8_t prev_time = 0;
static uint8_t prev_fsm = 0;
static uint8_t prev_pts = 0;
static tactic_num *tactic_ptr;
extern int16_t tact_fsm_case;

int8_t dbg = 0;

void
display_fsm ()
{
	/* Display FSM */
	switch (display_fsm_case)
		{

		/* Inicijalizacija displeja */
		case 0:
			tactic_ptr = get_tact_num_ptr ();
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
			if (tactic_ptr->side == BLUE)
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
			itoa (tactic_ptr->num, tactic_number, 10);
			HD44780_PrintStr (tactic_number);
			if (ready)
				{
					if (dbg)
						display_fsm_case = 13;
					else
						display_fsm_case = 3;
				}
			break;

			/* Ispis celog displeja */
		case 3:
			display_write_all (tactic_side_short, tactic_number);
			display_fsm_case = 4;
			break;

		case 13:
			display_write_all_dbg ();
			display_fsm_case = 14;
			break;

			/* Ispisivanje samo brojeva svake sekunde */
		case 4:
			if (prev_time != get_time_s ())
				{
					prev_time = get_time_s ();
					display_write_time (get_time_s ());
				}
			if (prev_fsm != tact_fsm_case)
				{
					prev_fsm = tact_fsm_case;
					display_write_case ();
				}
			if (prev_pts != get_points ())
				{
					prev_pts = get_points ();
					display_write_pts (get_points ());
				}
			break;

		case 14:
			if (prev_time != get_time_s ())
				{
					prev_time = get_time_s ();
					display_write_time (get_time_s ());
					display_write_pos ();
				}
			if (prev_fsm != tact_fsm_case)
				{
					prev_fsm = tact_fsm_case;
					display_write_case ();
				}
			break;

			/* Isteklo vreme, nista vise ne ispisuj */
		case 5:
			break;
		}
}

void
display_ready ()
{
	ready = 1;
}

void
display_write_all (char *tactic_side, char *tactic_num)
{
	HD44780_Clear ();
	HD44780_SetCursor (0, 0);
	HD44780_PrintStr ("+381");
	HD44780_SetCursor (9, 0);
	HD44780_PrintStr ("pts:");
	HD44780_SetCursor (4, 0);
	HD44780_PrintStr (":");
	HD44780_PrintStr (tactic_side);
	HD44780_PrintStr (tactic_num);
	HD44780_SetCursor (8, 1);
	HD44780_PrintStr ("time:");
	display_write_pts (get_points ());
	display_write_time (get_time_s ());
}

void
display_write_all_dbg ()
{
	HD44780_Clear ();
	HD44780_SetCursor (8, 1);
	HD44780_PrintStr ("time:");
}

void
display_write_time (uint8_t time)
{
	pad_num_string_uint8 (snum_time, 3, time);
	HD44780_SetCursor (13, 1);
	HD44780_PrintStr (snum_time);
}

void
display_write_pts (uint8_t points)
{
	pad_num_string_uint8 (snum, 3, points);
	HD44780_SetCursor (13, 0);
	HD44780_PrintStr (snum);
}

void
display_write_case ()
{
	pad_num_string (snum_fsm, 6, tact_fsm_case);
	HD44780_SetCursor (0, 1);
	HD44780_PrintStr (snum_fsm);
}

void
display_write_pos ()
{
	pad_num_string (x, 5, (int16_t) get_robot_base ()->x);
	pad_num_string (y, 5, (int16_t) get_robot_base ()->y);
	pad_num_string (phi, 4, (int16_t) get_robot_base ()->phi);

	HD44780_SetCursor (0, 0);
	HD44780_PrintStr (x);
	HD44780_SetCursor (6, 0);
	HD44780_PrintStr (y);
	HD44780_SetCursor (12, 0);
	HD44780_PrintStr (phi);
}

void
pad_num_string (char *str, uint8_t len, int16_t num)
{
	uint16_t abs_num = (uint16_t) (abs (num));
	uint8_t num_digits = 0;
	uint16_t temp = abs_num;

	do
		{
			temp /= 10;
			num_digits++;
		}
	while (temp > 0);

	for (int i = 0; i < len - 1; i++)
		str[i] = ' ';

	uint8_t sign_pos = len - num_digits - 1;
	str[sign_pos] = (num < 0) ? '-' : '+';

	for (int i = len - 1; i > sign_pos; i--)
		{
			str[i] = (abs_num % 10) + '0';
			abs_num /= 10;
		}

	str[len] = '\0';
}

void
pad_num_string_uint8 (char *str, uint8_t len, uint8_t num)
{
	uint8_t num_digits = 0;
	uint16_t temp = num;

	do
		{
			temp /= 10;
			num_digits++;
		}
	while (temp > 0);

	for (int i = 0; i < len - 1; i++)
		str[i] = ' ';

	for (int i = len - 1; i > len - num_digits - 1; i--)
		{
			str[i] = (num % 10) + '0';
			num /= 10;
		}

	str[len] = '\0';
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
	HAL_I2C_Master_Transmit (&hi2c3, DEVICE_ADDR, (uint8_t*) &data, 1, 1);
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
