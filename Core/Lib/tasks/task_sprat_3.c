/*
 * task_sprat_3.c
 *
 * 		Task dizanja 2 sprata na 3.:
 * 			- na pocetku ne drzi nista na strani na kojoj radi i otvorene su mu grtalice
 * 			- vec su spremna 2 sprata i odvojen 1 sprat ispred njih
 * 			- na kraju ne drzi nista u toj strani i izgradjena su 3 sprata
 *			- udaljen je za oko 200mm od centra ostavljenog sprata
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
extern uint32_t task_delay_3;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_3 (int8_t side)
{
	switch (task_fsm_case)
		{
		case 0:
			task_state = TASK_RUNNING;
			task_delay_3 = 0xFFFF;
			cur_task = move_on_dir (295, side, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			if (side == FORWARD)
				grtl_front_grip_all ();
			else
				grtl_back_grip_all ();
			task_delay_3 = 0xFFFF;
			HAL_Delay (200);
			task_fsm_case = 40;
			break;

		case 40:
//			if (delay_nb_2 (&task_delay_3, 1500))
			task_fsm_case = 50;
			if (side == FORWARD)
				lift_front_up ();
			else
				lift_back_up ();
			HAL_Delay (1500);
			break;

		case 50:
			cur_task = move_on_dir (115, side, 0.2, NO_SENS);
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
			if (delay_nb_2 (&task_delay_3, 100))
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
			cur_task = move_on_dir (200, -1 * side, 1.0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 90;
			break;

		case 90:
			if (side == FORWARD)
				lift_front_down ();
			else
				lift_back_down ();
			if (delay_nb_2 (&task_delay_3, 500))
				{
					task_fsm_case = -1;
				}
			break;

		case -1:
			task_fsm_case = 0;
			task_delay_3 = 0xFFFF;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
