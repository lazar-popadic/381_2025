/*
 * tact_global.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

tactic_num tactic;
int32_t tact_fsm_case;
int32_t prev_fsm_case = -1;
uint32_t tact_delay_1 = 0xFFFF;
uint32_t timeout_var = 0xFFFF;
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
	if (get_tact_num_ptr ()->side)	// plava
		return -x;
	// zuta
	return x;
}

float
phi_side (float phi)
{
	if (get_tact_num_ptr ()->side)	// plava
		return 180 - phi;
	// zuta
	return phi;
}

int8_t
bnr_side ()
{
	if (get_tact_num_ptr ()->side)	// plava
		return FORWARD;
	return BACKWARD;
}

tactic_num*
get_tact_num_ptr ()
{
	return &tactic;
}

uint8_t
timeout (uint32_t time)
{
	if (tact_fsm_case != prev_fsm_case)
		{
			timeout_var = 0xFFFF;
			prev_fsm_case = tact_fsm_case;
		}
	return delay_nb_2 (&timeout_var, time);
}
