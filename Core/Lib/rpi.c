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
 * ovde mi treba obrada odometrije, prebacivanje u string
 *
 * oblik poruke (ukupno 21 bajt): "sx+1234.5y+1234.5p123.0e"
 */

#include "main.h"
#include <stdio.h>
#include "usart.h"

static volatile struct_robot_base *base_ptr;
uint8_t recieve_from_rpi[21] =
  { 0 };
uint8_t transmit_to_rpi[21] =
  { 's', 'x', '+', '_', '_', '_', '_', '_', 'y', '+', '_', '_', '_', '_', '_', 'p', '_', '_', '_', '_', 'e' };
int16_t rpi_x = 0;
int16_t rpi_y = 0;
int16_t rpi_phi = 0;

void
rpi_init ()
{
  HAL_UART_Receive_DMA (&huart1, recieve_from_rpi, 21);
  HAL_UART_Transmit_DMA (&huart1, transmit_to_rpi, 21);
  base_ptr = get_robot_base ();
}

void
update_transmit_buffer ()
{
  rpi_x = (int16_t) (base_ptr->x * 10);
  rpi_y = (int16_t) (base_ptr->y * 10);
  rpi_phi = (int16_t) (wrap360 (base_ptr->phi) * 10);

  transmit_to_rpi[2] = get_sign_char (rpi_x);
  transmit_to_rpi[3] = (abs (rpi_x) / 10000) % 10 + '0';		// + '0' da bi prebacio u ASCII
  transmit_to_rpi[4] = (abs (rpi_x) / 1000) % 10 + '0';
  transmit_to_rpi[5] = (abs (rpi_x) / 100) % 10 + '0';
  transmit_to_rpi[6] = (abs (rpi_x) / 10) % 10 + '0';
  transmit_to_rpi[7] = abs (rpi_x) % 10 + '0';

  transmit_to_rpi[9] = get_sign_char (rpi_y);
  transmit_to_rpi[10] = (abs (rpi_y) / 10000) % 10 + '0';
  transmit_to_rpi[11] = (abs (rpi_y) / 1000) % 10 + '0';
  transmit_to_rpi[12] = (abs (rpi_y) / 100) % 10 + '0';
  transmit_to_rpi[13] = (abs (rpi_y) / 10) % 10 + '0';
  transmit_to_rpi[14] = abs (rpi_y) % 10 + '0';

  transmit_to_rpi[16] = (abs (rpi_phi) / 1000) % 10 + '0';
  transmit_to_rpi[17] = (abs (rpi_phi) / 100) % 10 + '0';
  transmit_to_rpi[18] = (abs (rpi_phi) / 10) % 10 + '0';
  transmit_to_rpi[19] = abs (rpi_phi) % 10 + '0';
}
