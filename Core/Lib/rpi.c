/*
 * rpi.c
 *
 *  Created on: Nov 23, 2024
 *      Author: lazar
 */

/* message: 10 bytes
 * uint8_t 's'
 * int16_t x = (-15000, +15000)			->	samo *10
 * int16_t y = (-10000, +10000)			->	samo *10
 * int16_t phi = (0, +3600)					->	wrap360 umesto wrap180 i *10
 * uint32_t mech_pos						->	lift_front, lift_back, grtl_fo, grtl_fi, grtl_bo, grtl_bi, ruc_front, ruc_back, gurl
 * uint8_t points = (0, 255)
 * uint8_t camera_signal
 * uint8_t 'e'
 */

#define TX_LEN				10
#define RX_LEN			8
#define PTS_OFFSET		0

#include "main.h"
#include <stdio.h>
#include "usart.h"

static volatile robot_base *base_ptr;
uint8_t recieved_from_rpi[RX_LEN] =
  { 0 };
uint8_t transmit_to_rpi[TX_LEN] =
  { 0 };
int16_t rpi_x = 0, rpi_y = 0, rpi_phi = 0;
uint8_t rpi_x_high, rpi_x_low, rpi_y_high, rpi_y_low, rpi_phi_high, rpi_phi_low;
uint32_t mech_states_u = 0;
float obstacle_x = 0, obstacle_y = 0, obstacle_phi = 0;
char camera_signal = '0';

void
rpi_init ()
{
  HAL_UART_Receive_DMA (&huart1, recieved_from_rpi, 8);
  HAL_UART_Transmit_DMA (&huart1, transmit_to_rpi, 8);
  base_ptr = get_robot_base ();
  transmit_to_rpi[0] = 's';
  transmit_to_rpi[TX_LEN - 1] = 'e';

}

void
update_transmit_buffer ()
{
  rpi_x = (int16_t) (base_ptr->x * 10);
  rpi_y = (int16_t) (base_ptr->y * 10);
  rpi_phi = (int16_t) (base_ptr->phi * 10);

  rpi_x_high = (rpi_x >> 8) & 0xFF;
  rpi_x_low = rpi_x & 0xFF;
  rpi_y_high = (rpi_y >> 8) & 0xFF;
  rpi_y_low = rpi_y & 0xFF;
  rpi_phi_high = (rpi_phi >> 8) & 0xFF;
  rpi_phi_low = rpi_phi & 0xFF;

  transmit_to_rpi[1] = rpi_x_high;
  transmit_to_rpi[2] = rpi_x_low;
  transmit_to_rpi[3] = rpi_y_high;
  transmit_to_rpi[4] = rpi_y_low;
  transmit_to_rpi[5] = rpi_phi_high;
  transmit_to_rpi[6] = rpi_phi_low;


  transmit_to_rpi[7] = (uint8_t) (get_pts () - PTS_OFFSET);

  transmit_to_rpi[8] = camera_signal;
}

void
edit_recieved_odom ()
{
  if (recieved_from_rpi[0] == 's' && recieved_from_rpi[RX_LEN] == 'e')
	{
	  obstacle_x = ((int16_t) ((int16_t) recieved_from_rpi[1] << 8) | (uint8_t) recieved_from_rpi[2]) * 0.1;
	  obstacle_y = ((int16_t) ((int16_t) recieved_from_rpi[3] << 8) | (uint8_t) recieved_from_rpi[4]) * 0.1;
	  obstacle_phi = ((int16_t) ((int16_t) recieved_from_rpi[5] << 8) | (uint8_t) recieved_from_rpi[6]) * 0.1;
	}
}

void
capture_image ()
{
  camera_signal = 'c';
}
