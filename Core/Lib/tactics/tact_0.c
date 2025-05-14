/*
 * tact_0.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
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
tact_0 ()
{
	switch (tact_fsm_case)
	{
	case -1:
		break;
	case 0: //krece ka ms24
		get_robot_base ()->x = x_side (-302);
		get_robot_base ()->phi = phi_side (90);
		get_robot_base ()->y = -710;
		tact_fsm_case = 2;
		break;
	case 2:
		cur_task = move_on_dir (170, BACKWARD, V_MAX_DEF * 0.5, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout (1000))
			tact_fsm_case = 5;
		break;
	case 5:
		lift_back_down_bnr ();
		grtl_back_open_outside ();
		if (delay_nb_2 (&tact_delay_1, 500))
		{
			tact_fsm_case = 7;
			add_points (20);
		}
		break;
	case 7:
		cur_task = move_on_dir (100, FORWARD, 0.5, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout (1000))
			tact_fsm_case = 11;
		break;
	case 11:
		lift_back_down ();
		grtl_back_close ();
		tact_fsm_case = 12;
		break;
	case 12: //srednji bunt blizi njemu
		if (get_tact_num_ptr ()->side) //plavu
			cur_task = move_on_path (x_side (-400), -430, phi_side (120), FORWARD, 0, V_MAX_DEF_PATH, 0, NO_SENS);
		else//zuta
			cur_task = move_on_path (x_side (-400), -430, phi_side (120), FORWARD, 0, V_MAX_DEF_PATH, 0, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 13;
		break;
	case 13:
		prepare_front ();
		tact_fsm_case = 15;
		break;
	case 15:
		cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 20;
		break;
	case 20: //pred uzimanje bunta
		cur_task = move_on_dir (350, FORWARD, 0.4, FORWARD); //300
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 30;
		break;
	case 30:
		grtl_front_grip_all ();
		ruc_front_carry ();
		lift_front_carry ();
		vacuum_front (1);
		tact_fsm_case = 40;
		break;
	case 40:
		cur_task = move_to_xy(x_side(-1200), -600, BACKWARD, V_MAX_DEF,W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;
		break;
	case 50:
		cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 55;
		break;
	case 55:
		prepare_back();
		tact_fsm_case = 56;
		break;
	case 56:
		cur_task = move_on_dir (100, BACKWARD, V_MAX_DEF*0.4, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout(2000))
			tact_fsm_case = 58;
		break;
	case 58:
		grtl_back_grip_all ();
		ruc_back_carry ();
		lift_back_carry ();
		vacuum_back (1);
		tact_fsm_case = 60;
		break;
	case 60:
		cur_task = move_to_xy(x_side(-725), -400, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		break;
	case 80:
		cur_task = rot_to_phi(-90, W_MAX_DEF, NO_SENS);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 85;
		break;
	case 85:
		cur_task = move_on_dir(350, FORWARD, V_MAX_DEF*0.4, NO_SENS);
		if(cur_task == TASK_SUCCESS || timeout(4000))
			tact_fsm_case = 90;
		break;
	case 90:
		cur_task = move_on_dir(250, BACKWARD, V_MAX_DEF*0.4, BACKWARD);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
	case 100:
		cur_task = task_sprat_12(FORWARD);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 105;
		break;
	case 105:
			cur_task = move_on_dir(60, FORWARD, V_MAX_DEF*0.4, NO_SENS);
			if(cur_task == TASK_SUCCESS)
				tact_fsm_case = 110;
			break;
	case 110:
		cur_task = task_sprat_3(FORWARD);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 115;
		break;
	case 115:
				cur_task = move_on_dir(100, BACKWARD, V_MAX_DEF*0.4, BACKWARD);
				if(cur_task == TASK_SUCCESS)
					tact_fsm_case = 120;
				break;
	case 120:
		cur_task = move_to_xy(x_side(-250), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 130;
		break;
	case 130:
		cur_task = rot_to_phi(90, W_MAX_DEF, NO_SENS);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 140;
		break;
	case 140:
		cur_task = move_on_dir(150, BACKWARD, V_MAX_DEF*0.4, NO_SENS);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 145;
		break;
	case 150:
		cur_task = task_sprat_12(BACKWARD);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 160;
		break;
	case 160:
		cur_task = move_on_dir(150, FORWARD, V_MAX_DEF*0.4, FORWARD);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 170;
		break;



	}
	return tact_state;
}
