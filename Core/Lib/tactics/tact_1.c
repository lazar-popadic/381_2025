/*
 * tact_dev.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 *
 *	Description:
 *			...
 */

#include "main.h"

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;

int8_t
tact_1 ()
{
	switch (tact_fsm_case)
		{
		case 0:
			prepare_front ();
			if (delay_nb_2 (&tact_delay_1, 500))
				{
					tact_fsm_case = 1;
				}
			break;

		case 1:
			if (move_to_xy (x_side (500), 0, FORWARD, V_MAX_DEF, W_MAX_DEF, NO_SENS) == TASK_SUCCESS)
				{
					tact_fsm_case = -1;
					tact_state = TASK_SUCCESS;
				}
			break;
		}
	return tact_state;
}
