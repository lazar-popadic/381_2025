/*
 * task_sprat_3_2_full.c
 *
 * 		Task celokupnog gradjenja 3 sprata, na vec prethodno postavljen 1, i ostavljanja preostala 2:
 *			-	2 sprata napravi
 *			- digne 2 sprata na 3.
 *			- okrene se
 *			- 2 sprata napravi
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_3_2_full (int8_t side)
{
	switch (task_fsm_case)
		{
		/*
		 *	Odvoji 1 sprat
		 */
		case 0:
			task_state = TASK_RUNNING;
			cur_task = task_sprat_12 (side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 10;
			break;

		case 10:
			cur_task = task_sprat_3 (side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 20;
			break;

		case 20:
			cur_task = rot_relative (180, W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			cur_task = task_sprat_12 (-1 * side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 40;
			break;

		case 40:
			cur_task = move_on_dir (200, side, V_MAX_DEF, side);
			vacuum_front (0);
			vacuum_back (0);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = -1;
			break;

		case -1:
			task_fsm_case = 0;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
