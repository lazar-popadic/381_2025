/*
 * task_dev.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
static uint32_t task_delay_1;
static int8_t task_state;
static int8_t cur_task;

int8_t
task_2_sprata (int8_t side)
{
	switch (task_fsm_case)
		{
		case 0:
			if (side == FORWARD)
				{
					lift_front_down ();
					grtl_front_open_outside ();
				}
			else
				{
					lift_back_down ();
					grtl_back_open_outside ();
				}
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 10;
			break;

		case 10:
			if (side == FORWARD)
				{
					vacuum_front (1);
					ruc_front_mid ();
				}
			else
				{
					vacuum_back (1);
					ruc_back_mid ();
				}
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 20;
			break;

		case 20:
			if (side == FORWARD)
				gurl_front ();
			else
				gurl_back ();
			cur_task = move_on_dir (70, -1 * side, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			gurl_mid ();
			cur_task = move_on_dir (70, -1 * side, 1.0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 40;
			break;

		case 40:
			if (side == FORWARD)
				ruc_front_down ();
			else
				ruc_back_down ();
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 50;
			break;

		case 50:
			vacuum_front (0);
			vacuum_back (0);
			task_fsm_case = 60;
			break;

		case 60:
			if (side == FORWARD)
				ruc_front_up ();
			else
				ruc_back_up ();
			if (delay_nb_2 (&task_delay_1, 200))
				task_fsm_case = 70;
			break;

		case 70:
			if (side == FORWARD)
				lift_front_up ();
			else
				lift_back_up ();
			if (delay_nb_2 (&task_delay_1, 500))
				task_fsm_case = 80;
			break;

		case 80:
			cur_task = move_on_dir (140, side, 0.15, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 90;
			break;

		case 90:
			if (side == FORWARD)
				lift_front_drop ();
			else
				lift_back_drop ();
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 100;
			break;

		case 100:
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
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 110;
			break;

		case 110:
			cur_task = move_on_dir (200, -1 * side, 1.0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = -1;
			break;

		case -1:
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
