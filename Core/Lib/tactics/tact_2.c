/*
 * tact_2.c
 *
 *  Created on: Mar 21, 2025
 *      Author: Lea mhm :-)
 *
 *	Description:
 *			...
 */

#include "main.h"

extern int32_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t
tact_2 ()
{
	switch (tact_fsm_case)
		{
		case -1:
			break;

		case 0: //krece ka ms24
			get_robot_base ()->x = x_side (1228);
			get_robot_base ()->phi = phi_side (-90);
			get_robot_base ()->y = -60;
			tact_fsm_case = 10;
			break;
		case 10:
			cur_task = move_to_xy (x_side (1150), -570, FORWARD, V_MAX_DEF, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;
		case 20:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 25;
			break;
		case 25:
			prepare_front ();
			tact_fsm_case = 30;
			break;
		case 30:
			cur_task = move_on_dir (160, FORWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (1500))
				{
					tact_fsm_case = 40;
					reset_all_delays ();
				}
			break;
		case 40:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 45;
			break;
		case 45:
			cur_task = move_on_dir (40, BACKWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF * 0.3, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 60;
			break;
		case 60:
			cur_task = move_on_dir (290, BACKWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (3000))
				{
					tact_fsm_case = 70;
					reset_all_delays ();
				}
			break;
		case 70:
			lift_back_down_bnr ();
			grtl_back_open_outside ();
			if (delay_nb_2 (&tact_delay_1, 500))
				{
					tact_fsm_case = 72;
					add_points (20);
				}
			break;
		case 72:
			cur_task = move_on_dir (70, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 75;
			break;
		case 75:
			grtl_back_close ();
			lift_back_down ();
			tact_fsm_case = 80;
			break;
		case 80:
			cur_task = move_on_path (x_side (1120), 270, phi_side (90), FORWARD, 1, 0.51, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 90;
			break;
		case 90:
			prepare_back ();
			tact_fsm_case = 100;
			break;
		case 100:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 110;
			break;
		case 110:
			cur_task = move_on_dir (200, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				{
					tact_fsm_case = 120;
					reset_all_delays ();
				}
			break;

		case 120:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 130;
			break;
		case 130:
			cur_task = move_on_path (x_side (1120), 100, phi_side (-60), FORWARD, 0, 0.51, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;
		case 140:
			cur_task = task_sprat_3_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 145;
			add_points (28);
			break;
		case 145:
			cur_task = move_on_dir(100, FORWARD, V_MAX_DEF*0.5, FORWARD);
			if(cur_task == TASK_SUCCESS)
				tact_fsm_case = 150;
			break;
		case 150:
			grtl_back_close ();
			tact_fsm_case = 160;
			break;
		case 160:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 170;
			break;
		case 170: //krece ka ms22
			cur_task = move_on_path (x_side (490), 250, phi_side (180), FORWARD, 0, 0.5, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 180;
			break;
		case 180:
			prepare_back ();
			tact_fsm_case = 190;
			break;
		case 190:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 200;
			break;
		case 200:
			cur_task = move_on_dir (320, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 210;
			break;
		case 210:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 220;
			break;
		case 220:
			cur_task = move_on_path (x_side (-228), -710, phi_side (90), BACKWARD, 0, 0.5, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 230;
			break;
		case 230:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF * 0.5, NO_SENS);
			tact_fsm_case = 240;
			break;
		case 240:
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				add_points (28);
				tact_fsm_case = -1;
			break;

		}
	return tact_state;
}
