/*
 * task_sprat_3_full.c
 *
 * 		Task celokupnog gradjenja 3 sprata, sa 2 cela bunta:
 *			- odvoji 1 sprat
 *			- okrene se
 *			-	2 sprata napravi
 *			- digne 2 sprata na 3.
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_3_full (int8_t side)
{
	switch (task_fsm_case)
		{
		/*
		 *	Odvoji 1 sprat
		 */
		case 0:
			task_state = TASK_RUNNING;
			cur_task = task_sprat_1 (side, OUT_GRTL);
			if (cur_task == TASK_SUCCESS)
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
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
