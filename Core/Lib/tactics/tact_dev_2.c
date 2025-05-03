/*
 * tact_dev_2.c
 *
 *		Taktika za razvoj i testiranje taskova.
 *		Trenutno:
 *			-	kvadrat
 *
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t
tact_dev_2 ()
{
	switch (tact_fsm_case)
		{
		case 0:
			start_match ();
			tact_fsm_case = 1;
			break;

		case 1:
			if (detected_timeout (1000))
				tact_fsm_case = 10;
			if (timeout (5000))
				tact_fsm_case = 20;
			break;

		default:
			if (timeout (2000))
				tact_fsm_case = 1;
			break;

		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
