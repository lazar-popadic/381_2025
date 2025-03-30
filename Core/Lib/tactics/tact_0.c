/*
 * tact_dev.c
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
tact_0 ()
{
	switch (tact_fsm_case)
		{
		case 0:
			prepare_back ();
			tact_fsm_case = 10;
			break;

		case 10:
			cur_task = move_on_dir (200, BACKWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 20;
				}
			break;

		case 20:
			cur_task = move_on_dir (100, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 30;
				}
			break;

		case 30:
			grtl_back_grip_all ();
			ruc_back_down ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 40;
				}
			break;

		case 40:
			lift_back_carry ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 60;
				}
			break;

		case 60:
			cur_task = move_on_dir (300, FORWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 70;
				}
			break;

		case 70:
			lift_back_down ();
			gurl_mid ();
			ruc_back_down ();
			grtl_back_open_outside ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 80;
				}
			break;

		case 80:
			vacuum_back (1);
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 85;
				}
			break;

		case 85:
			grtl_back_open_outside ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 90;
				}
			break;

		case 90:
			ruc_back_mid ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 100;
				}
			break;

		case 100:
			gurl_back ();
			cur_task = move_on_dir (70, FORWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 110;
				}
			break;

		case 110:
			gurl_mid ();
			if (move_on_dir (130, FORWARD, 0.5, NO_SENS) == TASK_SUCCESS)
				{
					tact_fsm_case = 120;
				}
			break;

		case 120:
			ruc_back_down ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 130;
				}
			break;

		case 130:
			vacuum_back (0);
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 140;
				}
			break;

		case 140:
			ruc_back_up ();
			if (delay_nb_2 (&tact_delay_1, 500))
				{
					tact_fsm_case = 150;
				}
			break;

		case 150:
			lift_back_up ();
			if (delay_nb_2 (&tact_delay_1, 1000))
				{
					tact_fsm_case = 160;
				}
			break;

		case 160:
			if (move_on_dir (200, BACKWARD, 0.25, NO_SENS) == TASK_SUCCESS)
				{
					tact_fsm_case = 170;
				}
			break;

		case 170:
			lift_back_drop ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 180;
				}
			break;

		case 180:
			grtl_back_open ();
			if (delay_nb_2 (&tact_delay_1, 100))
				{
					tact_fsm_case = 190;
				}
			break;

		case 190:
			if (move_on_dir (200, FORWARD, 0.5, NO_SENS) == TASK_SUCCESS)
				{
					tact_fsm_case = -1;
					tact_state = TASK_SUCCESS;
				}
			break;
		}
	return tact_state;
}
