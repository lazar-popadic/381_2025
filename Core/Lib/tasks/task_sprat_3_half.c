/*
 * task_sprat_3_half.c
 *
 * 		Task celokupnog gradjenja 3 sprata, sa 1 celim i jednim polovicnim buntom:
 *			- ostavi 1 sprat od polovicnog bunta
 *			- okrene se
 *			-	2 sprata napravi
 *			- digne 2 sprata na 3.
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
extern uint32_t task_delay_s3h;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_3_half (int8_t side)
{
	switch (task_fsm_case)
		{
		/*
		 *	Ostavi 1 sprat
		 */
		case 0:
			task_state = TASK_RUNNING;
			if (side == FORWARD)
				{
					lift_front_down ();
					grtl_front_open ();
					ruc_front_down ();
					vacuum_front (0);
				}
			else
				{
					lift_back_down ();
					grtl_back_open_outside ();
					ruc_back_down ();
					vacuum_back (0);
				}
			if (delay_nb_2 (&task_delay_s3h, 500))
				task_fsm_case = 5;
			break;

		case 5:
			cur_task = move_on_dir (220, -1 * side, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 7;
			break;

		case 7:
			if (side == FORWARD)
				{
					grtl_front_close ();
					ruc_front_up ();
				}
			else
				{
					grtl_back_close ();
					ruc_back_up ();
				}
			task_fsm_case = 10;
			break;

		case 10:
			cur_task = rot_relative (180, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 20;
			break;

			/*
			 *	Napravi 2 sprata
			 */
		case 20:
			cur_task = task_sprat_12 (-1 * side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

			/*
			 *	Podigne ih na 3.
			 */
		case 30:
			cur_task = task_sprat_3 (-1 * side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = -1;
			break;

		case -1:
			task_fsm_case = 0;
			task_delay_s3h = 0xFFFF;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
