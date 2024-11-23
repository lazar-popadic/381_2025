/*
 * input.c
 *
 *  Created on: Nov 23, 2024
 *      Author: lazar
 */

#include "main.h"
#include "gpio.h"

uint16_t
read_cinc ()
{
  return GPIOA->IDR >> 6 & 1;
}

uint16_t
read_switch_S ()
{
  return GPIOC->IDR >> 10 & 1;
}

uint16_t
read_switch_2 ()
{
  return GPIOC->IDR >> 11 & 1;
}

uint16_t
read_switch_1 ()
{
  return GPIOC->IDR >> 12 & 1;
}

void
choose_tactic (struct_tactic_num *tactic)
{
  tactic->side = read_switch_S ();
  tactic->num = (read_switch_2 () << 1) + read_switch_1 ();
}

uint16_t
read_sensors_front ()
{
  return GPIOC->IDR >> 4 & 1;
}

uint16_t
read_sensors_back ()
{
  return GPIOC->IDR >> 5 & 1;
}
