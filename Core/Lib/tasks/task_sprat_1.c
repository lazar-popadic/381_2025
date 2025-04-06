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
static uint32_t task_delay = 0xFFFF;
static int8_t task_state = TASK_RUNNING;
static int8_t cur_task;

int8_t
task_sprat_1 (int8_t side)
{
	switch (task_fsm_case)
		{
		case 0:
			task_state = TASK_RUNNING;
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = 10;
			if (side == FORWARD)
				{
					lift_front_down ();
					grtl_front_open_outside ();
					ruc_front_down ();
				}
			else
				{
					lift_back_down ();
					grtl_back_open_outside ();
					ruc_back_down ();
				}
			break;

		case 10:
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = 20;
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

			break;

		case 20:
			if (side == FORWARD)
				gurl_front ();
			else
				gurl_back ();
			cur_task = move_on_dir (75, -1 * side, 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 30;
			break;

		case 30:
			gurl_mid ();
			cur_task = move_on_dir (125, -1 * side, 1.0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 40;
			break;

		case 40:
			if (delay_nb_2 (&task_delay, 100))
				task_fsm_case = -1;
			if (side == FORWARD)
				ruc_front_full_down ();
			else
				ruc_back_full_down ();
			break;

		case -1:
			task_fsm_case = 0;
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
