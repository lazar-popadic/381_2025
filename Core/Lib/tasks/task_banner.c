/*
 * task_banner.c
 *
 * 		Task za ostavljanje banera:
 * 		Objasnjenje za argumente:
 * 				- bnr_dir:		da li da ide ka sredini stola ili ka spolja
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
uint32_t banner_delay = 0xFFFF;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_banner (int8_t bnr_dir)
{
	switch (task_fsm_case) {
	case 0:
		task_state = TASK_RUNNING;
		cur_task = rot_to_phi(180, W_MAX_DEF, NO_SENS);

		if (cur_task == TASK_SUCCESS)
			task_fsm_case = 180;
		break;
	case 180:
		if (get_tact_num_ptr()->side)	// plava
			bnr_3();
		else
			bnr_1();

			if (delay_nb_2 (&banner_delay, 250))
				task_fsm_case = 185;
			break;
	case 185:
		if (get_tact_num_ptr()->side)	// plava
			bnr_4();
		else
			bnr_2();

			if (delay_nb_2 (&banner_delay, 500))
				task_fsm_case = 190;
			break;
		case 190:
			cur_task = move_on_dir (500, bnr_dir, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 195;
			break;
	case 195:
		if (get_tact_num_ptr()->side)	// plava
			bnr_1();
		else
			bnr_3();

			if (delay_nb_2 (&banner_delay, 750))
				task_fsm_case = 200;
			break;
		case 200:
		if (get_tact_num_ptr()->side)	// plava
			bnr_2();
		else
			bnr_4();

		if (delay_nb_2 (&banner_delay, 500))
				task_fsm_case = -1;
			break;

		case -1:
			task_fsm_case = 0;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
