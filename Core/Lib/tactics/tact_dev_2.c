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
//			prepare_front ();
			if (delay_nb_2 (&tact_delay_1, 1000))
				tact_fsm_case = 10;
			bnr_close ();
			break;

		case 10:
			if (delay_nb_2 (&tact_delay_1, 600))
				tact_fsm_case = 20;
			bnr_1 ();
			break;

		case 20:
			if (delay_nb_2 (&tact_delay_1, 2000))
				tact_fsm_case = 30;
			bnr_2 ();
			break;

		case 30:
			if (delay_nb_2 (&tact_delay_1, 600))
				tact_fsm_case = 40;
			bnr_3 ();
			break;

		case 40:
			if (delay_nb_2 (&tact_delay_1, 2000))
				tact_fsm_case = 50;
			bnr_4 ();
			break;

		case 50:
			if (delay_nb_2 (&tact_delay_1, 10000))
				tact_fsm_case = -1;
			bnr_close ();
			break;

		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
