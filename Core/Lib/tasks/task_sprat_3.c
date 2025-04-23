/*
 * task_sprat_3.c
 *
 * 		Task dizanja 2 sprata na 3.:
 * 			- na pocetku ne drzi nista na strani na kojoj radi i otvorene su mu grtalice
 * 			- vec su spremna 2 sprata i odvojen 1 sprat ispred njih
 * 			- na kraju ne drzi nista u toj strani i izgradjena su 3 sprata
 *			- udaljen je za 200mm od centra ostavljenog sprata
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
task_sprat_3 (int8_t side)
{
	switch (task_fsm_case)
		{
		case 0:
			task_state = TASK_RUNNING;
			cur_task = move_on_dir (280, side, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			if (side == FORWARD)
				grtl_front_grip_all ();
			else
				grtl_back_grip_all ();
			task_fsm_case = 40;
			break;

		case 40:
			if (delay_nb_2 (&task_delay, 1000))
				task_fsm_case = 50;
			if (side == FORWARD)
				lift_front_up ();
			else if (side == BACKWARD)
				{
					lift_back_up ();
				}
			break;

		case 50:
			cur_task = move_on_dir (255, side, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 60;
			break;

		case 60:
			if (side == FORWARD)
				lift_front_drop ();
			else
				lift_back_drop ();
			task_fsm_case = 70;
			break;

		case 70:
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = 80;
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
			break;

		case 80:
			cur_task = move_on_dir (260, -1 * side, 1.0, NO_SENS);
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
