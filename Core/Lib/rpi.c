/*
 * rpi.c
 *
 *  Created on: Nov 23, 2024
 *      Author: lazar
 */

/*
 * x = (-15000, +15000)		->	samo *10
 * y = (-10000, +10000)		->	samo *10
 * phi = (0, +3600)			->	wrap360 umesto wrap180 i *10
 *
 * oblik poruke (ukupno 21 bajt): "sx+1234.5y+1234.5p123.0e"
 */

#include "main.h"
#include <stdio.h>
#include "usart.h"

static volatile struct_robot_base *base_ptr;
uint8_t recieved_from_rpi[19] =
  { 0 };
uint8_t transmit_to_rpi[19] =
	  { 0 };int16_t rpi_x = 0, rpi_y = 0, rpi_phi = 0;
float recieved_x = 0, recieved_y = 0, recieved_phi = 0;

void
rpi_init ()
{
  HAL_UART_Receive_DMA (&huart1, recieved_from_rpi, 19);
  HAL_UART_Transmit_DMA (&huart1, transmit_to_rpi, 19);
  base_ptr = get_robot_base ();
  transmit_to_rpi[0] = 's';
  transmit_to_rpi[18] = 'e';

}

void
update_transmit_buffer ()
{
  rpi_x = (int16_t) (base_ptr->x * 10);
  rpi_y = (int16_t) (base_ptr->y * 10);
  rpi_phi = (int16_t) (wrap360 (base_ptr->phi) * 10);

  transmit_to_rpi[2] = get_sign_char (rpi_x);
  transmit_to_rpi[3] = (abs (rpi_x) / 10000) % 10;
  transmit_to_rpi[4] = (abs (rpi_x) / 1000) % 10;
  transmit_to_rpi[5] = (abs (rpi_x) / 100) % 10;
  transmit_to_rpi[6] = (abs (rpi_x) / 10) % 10;
  transmit_to_rpi[7] = abs (rpi_x) % 10;

  transmit_to_rpi[8] = get_sign_char (rpi_y);
  transmit_to_rpi[9] = (abs (rpi_y) / 10000) % 10;
  transmit_to_rpi[10] = (abs (rpi_y) / 1000) % 10;
  transmit_to_rpi[11] = (abs (rpi_y) / 100) % 10;
  transmit_to_rpi[12] = (abs (rpi_y) / 10) % 10;
  transmit_to_rpi[13] = abs (rpi_y) % 10;

  transmit_to_rpi[14] = (abs (rpi_phi) / 1000) % 10;
  transmit_to_rpi[15] = (abs (rpi_phi) / 100) % 10;
  transmit_to_rpi[16] = (abs (rpi_phi) / 10) % 10;
  transmit_to_rpi[17] = abs (rpi_phi) % 10;
}

void
edit_recieved_odom ()
{
  if (recieved_from_rpi[0] == 's' && recieved_from_rpi[18] == 'e')
	{
		  recieved_x = interpret_sign (recieved_from_rpi[2]) * (recieved_from_rpi[3] * 1000 + recieved_from_rpi[4] * 100 + recieved_from_rpi[5] * 10 + recieved_from_rpi[6] + recieved_from_rpi[7] * 0.1);
		  recieved_y = interpret_sign (recieved_from_rpi[8]) * (recieved_from_rpi[9] * 1000 + recieved_from_rpi[10] * 100 + recieved_from_rpi[11] * 10 + recieved_from_rpi[12] + recieved_from_rpi[13] * 0.1);
		  recieved_phi = recieved_from_rpi[14] * 100 + recieved_from_rpi[15] * 10 + recieved_from_rpi[16] + recieved_from_rpi[17] * 0.1;
		  recieved_phi = wrap180 (recieved_phi);
	}
}
