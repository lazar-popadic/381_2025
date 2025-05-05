/*
 * tact_0.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 *
 *	Description:
 *			...
 */

#include "main.h"

extern int32_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

void
tact_3 ()
{
	lift_back_down ();
	vacuum_back (1);
	HAL_Delay (10);
	vacuum_front (1);
	HAL_Delay (10);
	prepare_back ();
	HAL_Delay (1000);
	prepare_front ();
}
