/*
 * tact_global.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

tactic_num tactic;
uint16_t tact_fsm_case;
uint32_t tact_delay_1 = 0xFFFF;
uint8_t points = 0;
int8_t tact_state = TASK_RUNNING;

uint8_t
get_points ()
{
	return points;
}

void
set_points (uint8_t pts)
{
	points = pts;
}

void
add_points (uint8_t pts)
{
	points += pts;
}

float
x_side (float x)
{
	if (get_tact_num_ptr ()->side)
		return -x;
	return x;
}

tactic_num*
get_tact_num_ptr ()
{
	return &tactic;
}

