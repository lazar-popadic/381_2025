/*
 * ax12a.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "../Inc/main.h"
#include "../Inc/usart.h"

/*
 * ugao: (OD 0 DO 1023 => 0D 0 stepeni DO 300 stepeni)
 * 0 = 0
 * 0x1ff = 511 = 150 stepeni
 * 0x3ff = 1023 = 300 stepeni
 *
 * brzina: (OD 1 DO 1023 => OD minimalne DO maksimalne)
 * 1 = minimum
 * 528 = maximum za ax12a
 * 0x3ff = 1023 = maximum, oko 59 rpm
 */

void
ax_move (uint8_t id, uint16_t angle, uint16_t speed, UART_HandleTypeDef huart)
{
	uint8_t angle_low, angle_high, speed_low, speed_high;
	angle_low = angle & 0xff;
	angle_high = angle >> 8;
	speed_low = speed & 0xff;
	speed_high = speed >> 8;

	uint16_t checksum_local = id + 7 + 3 + 0x1e + angle_low + angle_high + speed_low + speed_high;
	uint8_t checksum = (uint8_t) (~checksum_local);
	uint8_t ax_move[] =
		{ 0xff, 0xff, id, 0x07, 0x03, 0x1E, angle_low, angle_high, speed_low, speed_high, checksum };

	HAL_UART_Transmit (&huart, ax_move, 11, 200);
	HAL_Delay (50);
}

