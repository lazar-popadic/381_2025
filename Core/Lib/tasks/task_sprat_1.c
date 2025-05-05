/*
 * task_sprat_1.c
 *
 * 		Task odvajanja 1 sprata:
 * 			- na pocetku drzi ceo MS
 * 			- na kraju drzi pola MS (unutrasnje konzerve i 1 dasku)
 *			- udaljen je za 200mm od centra ostavljenog sprata
 *
 *  Created on: Mar 31, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case = 0;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_1 (int8_t side, int8_t in_out)
{
	switch (task_fsm_case)
		{
		case 0:
			task_state = TASK_RUNNING;
			if (side == FORWARD)
				{
					//grtl_front_open_outside ();
					lift_front_down ();
					ruc_front_down ();
					if (in_out == OUT_GRTL)
						grtl_front_open_outside ();
					else
						grtl_front_open_inside ();

				}
			else
				{
					//grtl_back_open_outside ();
					lift_back_down ();
					ruc_back_down ();
					if (in_out == OUT_GRTL)
						grtl_back_open_outside ();
					else
						grtl_back_open_inside ();
				}
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
			task_fsm_case = 20;
			break;

		case 20:
			cur_task = move_on_dir (75, -1 * side, 0.25, NO_SENS);
			if (side == FORWARD)
				gurl_front ();
			else
				gurl_back ();
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			cur_task = move_on_dir (125, -1 * side, 1.0, NO_SENS);
			gurl_mid ();
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 35;
			break;

		case 35:
			if (side == FORWARD)
				ruc_front_down ();
			else
				ruc_back_down ();
			task_fsm_case = 40;
			break;

		case 40:
			if (side == FORWARD)
				ruc_front_full_down ();
			else
				ruc_back_full_down ();
			task_fsm_case = -1;
			add_points (4);
			break;

		case -1:
			task_fsm_case = 0;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
