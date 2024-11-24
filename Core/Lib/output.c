/*
 * output.c
 *
 *  Created on: Nov 23, 2024
 *      Author: lazar
 */

#include "main.h"
#include "gpio.h"

void
motor_l_forw ()
{
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, 0);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_7, 1);
}

void
motor_l_back ()
{
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_7, 0);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, 1);
}

void
motor_r_forw ()
{
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, 0);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, 1);
}

void
motor_r_back ()
{
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, 0);
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, 1);
}

void
vacuum_0 (uint8_t on)
{
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_0, on);
}

void
vacuum_1 (uint8_t on)
{
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_1, on);
}

void
vacuum_2 (uint8_t on)
{
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_2, on);
}

void
vacuum_3 (uint8_t on)
{
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_3, on);
}
