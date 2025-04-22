/*
 * tact_1.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 *
 *	Description:
 *			...
 */

#include "main.h"

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t
tact_1 ()
{
	switch (tact_fsm_case)
		{
	case 0: //krece ka ms24
			get_robot_base ()->x = x_side (-228);
			// TODO: strane, ovo je za zutu, ne radi x_side
			get_robot_base ()->phi = phi_side (90);
			get_robot_base ()->y = -710;
			prepare_front ();
			tact_fsm_case = 10;
			break;
		case 10: //srednji bunt blizi njemu
			cur_task = move_on_path (x_side (-400), -250, phi_side (90), FORWARD, 0, 0.5, 0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11;
			break;
		case 11:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;
		case 20: //pred uzimanje bunta
			cur_task = move_to_xy (x_side (-400), 150, FORWARD, 0.3, W_MAX_DEF,
					NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 30;
			break;
		case 30:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			prepare_back ();
			tact_fsm_case = 40;
			break;
		case 40: //bunt pored startnog polja
		cur_task = move_on_path(x_side(-690), -400, phi_side(0), BACKWARD, 0,
				0.5, 0, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 45;
		break;
	case 45:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;
		break;
		case 50: //pred uzimanje bunta
			cur_task = move_on_dir (270, BACKWARD, 0.3, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 60;
			break;
		case 60:
			grtl_back_grip_all ();
			ruc_back_carry ();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 70;
		break;
	case 70:
		cur_task = move_to_xy(x_side(-300), -730, BACKWARD, 0.5, W_MAX_DEF,
		NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 75;
		break;
	case 75:
		cur_task = rot_to_phi(phi_side(180), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		break;
	case 80:
		cur_task = task_sprat_3_1_full(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 9999;
		break;
	case 90:
			cur_task = move_on_dir (100, FORWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 100;
			break;
		case 81:
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 85;
			break;
		case 85:
			grtl_back_close ();
			HAL_Delay (10);
			grtl_front_close ();
			tact_fsm_case = 87;
			break;
		case 87:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 90;
			break;
		case 91: //krece ka ms22
			cur_task = move_on_path (x_side (470), 250, phi_side (0), BACKWARD, 0, 0.5, 0,
					NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 91;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 10000;
			break;
		case 92:
			prepare_back ();
			tact_fsm_case = 95;
			break;
		case 95:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 100;
			break;
		case 100:
			cur_task = move_on_dir (270, BACKWARD, 0.3, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 110;
			break;
		case 110:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 120;
			break;
		case 120: //krece ka ms14
			cur_task = move_on_path (x_side (-650), -450, phi_side (0), BACKWARD, 0, 0.5, 0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 125;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 15000;
			break;
		case 125:
				prepare_front ();
				tact_fsm_case = 130;
				break;
		case 130:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;
		case 140:
			cur_task = move_on_dir (230, FORWARD, 0.3, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 150;
			break;
		}
	return tact_state;
}
