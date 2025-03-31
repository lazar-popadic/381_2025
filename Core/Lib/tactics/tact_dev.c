/*
 * tact_dev.c
 *
 *		Taktika za razvoj i testiranje taskova.
 *		Trenutno:
 *			-	task za gradjenje 2 sprata
 *
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t
tact_dev ()
{
	switch (tact_fsm_case)
		{
		case 0:
			prepare_front ();
			tact_fsm_case = 10;
			break;

		case 10:
			cur_task = move_on_dir (200, FORWARD, 1.0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;

		case 20:
			cur_task = move_on_dir (100, FORWARD, 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 30;
			break;

		case 30:
			grtl_front_grip_all ();
			ruc_front_down ();
			lift_front_carry ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 60;
			break;

		case 60:
			cur_task = move_to_xy (0, 0, FORWARD, 1.0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 70;
			break;

		case 70:
			lift_front_down ();
			gurl_mid ();
			ruc_front_down ();
			grtl_front_open_outside ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 80;
			break;

		case 80:
			vacuum_front (1);
			grtl_front_open_outside ();
			ruc_front_mid ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 100;
			break;

		case 100:
			gurl_front ();
			cur_task = move_on_dir (70, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 110;
			break;

		case 110:
			gurl_mid ();
			cur_task = move_on_dir (130, BACKWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 120;
			break;

		case 120:
			ruc_front_down ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 130;
			break;

		case 130:
			vacuum_front (0);
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 140;
			break;

		case 140:
			ruc_front_up ();
			if (delay_nb_2 (&tact_delay_1, 200))
				tact_fsm_case = 150;
			break;

		case 150:
			lift_front_up ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 160;
			break;

		case 160:
			cur_task = move_on_dir (200, FORWARD, 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 170;
			break;

		case 170:
			lift_front_drop ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 180;
			break;

		case 180:
			grtl_front_open ();
			lift_front_leave ();
			if (delay_nb_2 (&tact_delay_1, 100))
				tact_fsm_case = 190;
			break;

		case 190:
			cur_task = move_on_dir (200, BACKWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 200;
			break;

		case 200:
			cur_task = rot_to_phi (0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = -1;
			break;
		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
