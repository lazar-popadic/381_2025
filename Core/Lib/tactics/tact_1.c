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
			cur_task = move_to_xy (x_side (-400), 80, FORWARD, 0.3, W_MAX_DEF,
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
			cur_task = move_on_path (x_side (-680), -400, phi_side (0), BACKWARD, 0, 0.5, 0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 45;
			break;
		case 45:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50: //pred uzimanje bunta
			cur_task = move_on_dir (310, BACKWARD, 0.3, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 60;
			break;
		case 60:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 70;
			break;
		case 70:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = task_banner (TO_CENTER);
		else
			cur_task = task_banner (TO_OUT);

			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 80;
			break;

		case 80:
			if (get_tact_num_ptr ()->side)	// plava
				cur_task = task_sprat_3_1_full (FORWARD);
			else
				cur_task = task_sprat_3_1_full (BACKWARD);
			//cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 90;
			break;
		case 90: //bunt kod protivnickog polja
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy (x_side (-1182), -597, BACKWARD, V_MAX_DEF,
						W_MAX_DEF,NO_SENS);
		else
			cur_task = move_to_xy (x_side (-1182), -597, FORWARD, V_MAX_DEF,
						W_MAX_DEF,NO_SENS);

			if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
	case 100:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy(x_side(-1300), -595, BACKWARD, 0.2,
			W_MAX_DEF, NO_SENS);
		else
			cur_task = move_to_xy(x_side(-1300), -595, FORWARD, 0.2,
			W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 110;
		break;
		case 110:
		if (get_tact_num_ptr()->side) {
			grtl_back_grip_all();
			ruc_back_carry();
			lift_back_carry();
			vacuum_back(1);
			prepare_front();
		} else {
			grtl_front_grip_all();
			ruc_front_carry();
			lift_front_carry();
			vacuum_front(1);
			prepare_back(); }
		tact_fsm_case = 120;
			break;

	case 120: //rezervisan bunt
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_path(x_side(-735), 400, phi_side(90), FORWARD, 0,
					0.5, 0, NO_SENS);
		else
			cur_task = move_on_path(x_side(-735), 400, phi_side(90), BACKWARD,
					0, 0.5, 0, NO_SENS);

			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 130;
			break;

		case 130:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		else
			cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);

		if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;

		case 140:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy(x_side(-735), 620, FORWARD, 0.3, W_MAX_DEF,
					NO_SENS);
		else
			cur_task = move_to_xy(x_side(-735), 620, BACKWARD, 0.3, W_MAX_DEF,
			NO_SENS);
		if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 150;
			break;

		case 150:
		if (get_tact_num_ptr()->side) {
			grtl_front_grip_all();
			ruc_front_carry();
			lift_front_carry();
			vacuum_front(1);
		} else {
			grtl_back_grip_all();
			ruc_back_carry();
			lift_back_carry();
			vacuum_back(1);
		}
		tact_fsm_case = 160;
			break;
		case 160: //krece ka polju za gradnju
		if (get_tact_num_ptr()->side)
			cur_task = move_to_xy(x_side(-735), -700, BACKWARD, 0.5, W_MAX_DEF,
			NO_SENS);
		else
			cur_task = move_to_xy(x_side(-735), -700, FORWARD, 0.5, W_MAX_DEF,
			NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 170;
			break;
		case 170:
		if (get_tact_num_ptr()->side)
			cur_task = rot_to_phi(phi_side(180), W_MAX_DEF, NO_SENS);
		else
			cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 175;
			break;
		case 175:
			if (get_tact_num_ptr()->side)
			cur_task = move_on_dir(60, BACKWARD, 0.3, NO_SENS);
		else
			cur_task = move_on_dir(60, FORWARD, 0.3, NO_SENS);
		if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 180;
			break;
		case 180:
			if (get_tact_num_ptr ()->side)	// plava
				cur_task = task_sprat_12 (BACKWARD);
			else
				cur_task = task_sprat_12 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 190;
			break;
		case 190:
			if (get_tact_num_ptr ()->side)	// plava
				cur_task = task_sprat_3 (BACKWARD);
			else
				cur_task = task_sprat_3 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 195;
			break;
		case 195:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		else
			cur_task = rot_to_phi(phi_side(180), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 200;
			break;

		case 200:
			if (get_tact_num_ptr ()->side)	// plava
				cur_task = task_sprat_12 (FORWARD);
			else
				cur_task = task_sprat_12 (BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 210;
		break;
	case 210: //krece ka ms14
		if (get_tact_num_ptr()->side)
			cur_task = move_to_xy(x_side(-1000), 500, BACKWARD, 0.5, W_MAX_DEF,
			NO_SENS);
		else
			cur_task = move_to_xy(x_side(-1000), 500, FORWARD, 0.5, W_MAX_DEF,
			NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 9999;
		break;

		}
	return tact_state;
}
