/*
 * tact_dev.c
 *
 *		Taktika za razvoj i testiranje taskova.
 *		Trenutno:
 *			-	task za gradjenje 2 sprata
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
tact_dev ()
{
	switch (tact_fsm_case)
		{
		case 0:
			prepare_front ();
			tact_fsm_case = 10;
			break;

		case 10:
			cur_task = move_on_dir (200, FORWARD, 1.0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;

		case 20:
			cur_task = move_on_dir (100, FORWARD, 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 30;
			break;

		case 30:
			grtl_front_grip_all ();
			ruc_front_down ();
			lift_front_carry ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 40;
			break;

		case 40:
			cur_task = move_on_path (300, -600, 180, FORWARD, 0, 0.5, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;

		case 50:
			cur_task = task_sprat_12 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 60;

		case 60:
			cur_task = rot_to_phi (0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = -1;
			break;

		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
