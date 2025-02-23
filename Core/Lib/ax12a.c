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
ax_move (uint8_t id, uint16_t angle, uint16_t speed)
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

  HAL_UART_Transmit (&huart6, ax_move, 11, 100);
  HAL_UART_Transmit (&huart6, ax_move, 11, 100);
  HAL_UART_Transmit (&huart6, ax_move, 11, 100);
  HAL_UART_Transmit (&huart6, ax_move, 11, 100);
//  HAL_UART_Transmit_IT(&huart6, ax_move, 11);	// ima i ova opcija, ali mozda je bolje da mi zablokira dok ne posalje celu poruku
}

void
ax_move_2 (uint8_t id, uint16_t angle, uint16_t speed)
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

  HAL_UART_Transmit (&huart6, ax_move, 11, 100);
  HAL_Delay (10);
}

uint8_t flag_stop = 0;

void
ax_move_381 (uint8_t id, uint16_t angle, uint16_t speed)
{

  if (!flag_stop)
	{
	  uint8_t checksum_local = 0;
	  uint16_t sum_local = 0;

	  uint8_t servo_command[19];
	  uint8_t angle_lower, angle_upper;
	  uint8_t speed_lower, speed_upper;
	  //    uint8_t sum, checksum;

	  angle = angle * 3 + angle * 4 / 10;
	  angle_lower = (uint8_t) (angle & 0xff);
	  angle_upper = (uint8_t) (angle >> 8);
	  speed_lower = (uint8_t) (speed & 0xff);
	  speed_upper = (uint8_t) (speed >> 8);

	  sum_local = id + 0x28 + angle_lower + angle_upper + speed_lower + speed_upper;
	  checksum_local = (uint8_t) (~sum_local);

	  servo_command[0] = 0xFF;
	  servo_command[1] = 0xFF;
	  servo_command[2] = 0xFF;
	  servo_command[3] = 0xFF;
	  servo_command[4] = 0xFF;
	  servo_command[5] = 0xFF;
	  servo_command[6] = 0xFF;
	  servo_command[7] = 0xFF;
	  servo_command[8] = (uint8_t) id;
	  servo_command[9] = 0x07;
	  servo_command[10] = 0x03;
	  servo_command[11] = 0x1E;
	  servo_command[12] = angle_lower;
	  servo_command[13] = angle_upper;
	  servo_command[14] = speed_lower;
	  servo_command[15] = speed_upper;
	  servo_command[16] = checksum_local;

	  HAL_UART_Transmit (&huart6, servo_command, 17, 100);
	  HAL_Delay (2);
	}
}
