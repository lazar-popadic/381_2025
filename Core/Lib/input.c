/*
 * input.c
 *
 *  Created on: Nov 23, 2024
 *      Author: lazar
 */

#define CINC_THRESHOLD 10

#include "main.h"
#include "gpio.h"

static uint16_t cinc_cnt = 0;

uint8_t
read_cinc ()
{
	return GPIOA->IDR >> 6 & 1;
}

uint8_t
cinc_db ()
{
	if (!read_cinc ())
		cinc_cnt++;
	else
		cinc_cnt = 0;
	if (cinc_cnt > CINC_THRESHOLD)
		return 1;
	return 0;
}

uint8_t
read_switch_S ()
{
	return GPIOC->IDR >> 10 & 1;
}

uint8_t
read_switch_2 ()
{
	return GPIOC->IDR >> 11 & 1;
}

uint8_t
read_switch_1 ()
{
	return GPIOC->IDR >> 12 & 1;
}

void
choose_tactic (tactic_num *tactic)
{
	tactic->side = read_switch_S ();
	tactic->num = (read_switch_2 () << 1) + read_switch_1 ();
}

uint8_t
read_sensors_front ()
{
	return GPIOC->IDR >> 4 & 1;
}

uint8_t
read_sensors_back ()
{
	return GPIOC->IDR >> 5 & 1;
}
