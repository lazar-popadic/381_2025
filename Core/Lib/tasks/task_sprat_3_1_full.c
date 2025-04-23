/*
 * task_sprat_3_1_full.c
 *
 * 		Task celokupnog gradjenja 3 sprata i ostavljanja preostalog:
 *			- odvoji 1 sprat
 *			- okrene se
 *			-	2 sprata napravi
 *			- digne 2 sprata na 3.
 *			- okrene se
 *			- ostavi 1 sprat
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
static uint32_t task_delay = 0xFFFF;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_3_1_full (int8_t side)
{
	switch (task_fsm_case)
		{
		/*
		 *	Odvoji 1 sprat
		 */
		case 0:
			task_state = TASK_RUNNING;
			cur_task = task_sprat_3_full (side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 40;
			break;

		case 40:
			cur_task = rot_relative (180, W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 50;
			break;

		case 50:
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = 60;
			if (side == FORWARD)
				grtl_back_open ();
			else
				grtl_front_open ();
			break;

		case 60:
//			cur_task = move_on_dir (75, side, V_MAX_DEF, NO_SENS);
			vacuum_front (0);
			vacuum_back (0);
//			if (cur_task == TASK_SUCCESS)
			task_fsm_case = 70;
			break;

		case 70:
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = 80;
			if (side == BACKWARD)
				{
					grtl_back_open ();
					ruc_back_up ();
				}
			else
				{
					grtl_front_open ();
					ruc_front_up ();
				}
			break;

		case 80:
			cur_task = move_on_dir (130, -1 * side, V_MAX_DEF, -1 * side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = -1;
			break;

		case -1:
			task_fsm_case = 0;
			task_delay = 0xFFFF;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
