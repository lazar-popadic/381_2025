/*
 * task_sprat_12.c
 *
 * 		Task odvajanja i sastavljanja 2 sprata:
 * 			- na pocetku drzi ceo MS
 * 			- na kraju ne drzi nista u toj strani i izgradjena su 2 sprata
 *			- udaljen je za 200mm od centra ostavljenog sprata
 *			- otvorene su mu grtalice na zadatoj strani i lift je dole
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
task_sprat_12 (int8_t side)
{
	switch (task_fsm_case)
		{
		case 0:
			task_state = TASK_RUNNING;
			cur_task = task_sprat_1 (side);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 3;
			break;

		case 3:
			if (side == FORWARD)
				vacuum_front (0);
			else
				vacuum_back (0);
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = 5;
			break;

		case 5:
			if (delay_nb_2 (&task_delay, 500))
				task_fsm_case = 10;
			if (side == FORWARD)
				ruc_front_up ();
			else
				ruc_back_up ();
			break;

		case 10:
			if (delay_nb_2 (&task_delay, 1000))
				task_fsm_case = 20;
			if (side == FORWARD)
				lift_front_up ();
			else
				lift_back_up ();
			break;

		case 20:
			cur_task = move_on_dir (210, side, 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			if (side == FORWARD)
				lift_front_drop ();
			else
				lift_back_drop ();
			task_fsm_case = 40;
			break;

		case 40:
			if (side == FORWARD)
				{
					grtl_front_open ();
					lift_front_leave ();
				}
			else
				{
					grtl_back_open ();
					lift_back_leave ();
				}
			task_fsm_case = 50;
			break;

		case 50:
			cur_task = move_on_dir (210, -1 * side, V_MAX_DEF, NO_SENS);
			if (delay_nb_2 (&task_delay, 500))
				{
					if (side == FORWARD)
						lift_front_down ();
					else
						lift_back_down ();
				}
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
