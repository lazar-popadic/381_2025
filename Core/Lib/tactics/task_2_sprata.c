/*
 * task_dev.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

static int16_t task_fsm_case;
static uint32_t task_delay_1;
static int8_t task_state;
static int8_t cur_task;


int8_t
task_2_sprata (int8_t side)
{
	switch (task_fsm_case)
		{
		case 60:
			cur_task = move_to_xy (0, 0, side, 1.0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 70;
			break;

		case 70:
			lift_front_down ();
			gurl_mid ();
			ruc_front_down ();
			grtl_front_open_outside ();
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 80;
			break;

		case 80:
			if(side == FORWARD){
				vacuum_front (1);
			grtl_front_open_outside ();
			ruc_front_mid ();
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 100;
			}
			else{
				vacuum_back (1);
			grtl_back_open_outside ();
			ruc_back_mid ();
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 100;
			}
			break;

		case 100:
			if(side == FORWARD){
				gurl_front ();
			}
			else{
				gurl_back ();
			}
			cur_task = move_on_dir (70, -1*side, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 110;
			break;

		case 110:
			gurl_mid ();
			cur_task = move_on_dir (130, -1*side, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 120;
			break;

		case 120:
			if(side == FORWARD){
			ruc_front_down ();
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 130;
			}
			else{
				ruc_back_down ();
				if (delay_nb_2 (&task_delay_1, 100))
					task_fsm_case = 130;
			}
			break;

		case 130:
			if(side == FORWARD){
				vacuum_front (0);
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 140;
			}
			else{
				vacuum_back (0);
			if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 140;
			}
			break;

		case 140:
			if(side == FORWARD){
				ruc_front_up ();
				if (delay_nb_2 (&task_delay_1, 200))
					task_fsm_case = 150;
			}
			else {
			ruc_back_up ();
			if (delay_nb_2 (&task_delay_1, 200))
				task_fsm_case = 150;
			}
			break;

		case 150:
			if(side == FORWARD){
			lift_front_up ();
			if (delay_nb_2 (&task_delay_1, 500))
				task_fsm_case = 160;
			}
			else{
				lift_back_up ();
				if (delay_nb_2 (&task_delay_1, 500))
					task_fsm_case = 160;
			}
			break;

		case 160:
			cur_task = move_on_dir (200, side, 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = 170;
			break;

		case 170:
			if(side == FORWARD){
				lift_front_drop ();
				if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 180;
			}
			else{
				lift_back_drop ();
				if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 180;
			}
			break;

		case 180:
			if(side == FORWARD){
				grtl_front_open ();
				lift_front_leave ();
				if (delay_nb_2 (&task_delay_1, 100))
				task_fsm_case = 190;
			}
			else {
				grtl_back_open ();
				lift_back_leave ();
				if (delay_nb_2 (&task_delay_1, 100))
					task_fsm_case = 190;
			}
			break;
		case 190:
			cur_task = move_on_dir (200, -1*side, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				task_fsm_case = -1;
			break;

		case -1:
			task_state = TASK_SUCCESS;
			break;
		}
	return task_state;
}
