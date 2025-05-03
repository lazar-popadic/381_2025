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

extern int32_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t tact_1() {
	switch (tact_fsm_case) {
	case 0: //krece ka ms24
		get_robot_base()->x = x_side(-228);
		get_robot_base()->phi = phi_side(90);
		get_robot_base()->y = -710;
		tact_fsm_case = 10;
		break;
	case 10: //srednji bunt blizi njemu
		cur_task = move_on_path(x_side(-360), -380, phi_side(135), FORWARD, 0,
				0.5, 0, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 12;
		break;
	case 12:
		prepare_front();
		tact_fsm_case = 15;
		break;
	case 15:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 20;
		break;
	case 20: //pred uzimanje bunta
		cur_task = move_on_dir(300, FORWARD, 0.2, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 30;
		break;
	case 30:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 40;
		break;
	case 40: //bunt pored startnog polja
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_path(x_side(-700), -400, phi_side(45), BACKWARD, //-680
					0, 0.5, 0, NO_SENS);
		else
			cur_task = move_on_path(x_side(-700), -400, phi_side(45), BACKWARD,
					0, 0.5, 0, NO_SENS);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 42;
		break;
	case 42:
		prepare_back();
		tact_fsm_case = 45;
		break;
	case 45:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;
		break;
	case 50: //pred uzimanje bunta
		cur_task = move_on_dir(270, BACKWARD, 0.2, NO_SENS); //280
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 60;
		break;
	case 60:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 65;
		break;
	case 65:
		cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 70;
		break;
	case 70:
		if (get_tact_num_ptr()->side)	// plava
			cur_task =  move_on_dir(450, FORWARD, 0.5, NO_SENS); //280
		else
			cur_task = move_on_dir(450, BACKWARD, 0.5, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		break;
		/*case 70:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = task_banner(TO_CENTER);
		else
			cur_task = task_banner(TO_OUT);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		break;*/

	case 80:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = task_sprat_3_1_full(FORWARD);
		else
			cur_task = task_sprat_3_1_full(BACKWARD);
		//cur_task = task_sprat_3_full (BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 90;
		break;
	case 90: //bunt kod protivnickog polja
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy(x_side(-1192), -607, BACKWARD, 0.75, //-1182
					W_MAX_DEF,
					BACKWARD);
		else
			cur_task = move_to_xy(x_side(-1182), -582, FORWARD, V_MAX_DEF,
					W_MAX_DEF,
					FORWARD);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 95;
		if (timeout (2000))
			tact_fsm_case = 500;
		break;
	case 95:
		cur_task = rot_to_phi(180, W_MAX_DEF, NO_SENS); //180
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
	case 100:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_dir(115, BACKWARD, 0.2, NO_SENS);//118
		else
			cur_task = move_on_dir(118, FORWARD, 0.2, NO_SENS);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 110;
		break;
	case 110:
		if (get_tact_num_ptr()->side) {
			grtl_back_grip_all();
			ruc_back_carry();
			lift_back_carry();
			vacuum_back(1);
		} else {
			grtl_front_grip_all();
			ruc_front_carry();
			lift_front_carry();
			vacuum_front(1);
		}
		tact_fsm_case = 120;
		break;

	case 120: //rezervisan bunt
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_path(x_side(-705), 400, phi_side(90), FORWARD, 0, //-735
					0.5, 0, NO_SENS);
		else
			cur_task = move_on_path(x_side(-665), 400, phi_side(-90), BACKWARD,
					0, 0.5, 0, NO_SENS);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 130;
		break;

	case 130:
		if (get_tact_num_ptr()->side)	// plava
		{
			cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
			prepare_front();
		} else {
			cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
			prepare_back();
		}
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 140;
		break;

	case 140:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_dir(130, FORWARD, 0.2, NO_SENS);//140
		else
			cur_task = move_on_dir(140, BACKWARD, 0.2, NO_SENS);

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
			cur_task = move_to_xy(x_side(-685), -720, BACKWARD, V_MAX_DEF*0.5, //x je bio na -735 y ma -700
					W_MAX_DEF*0.5,
					NO_SENS);
		else
			cur_task = move_to_xy(x_side(-685), -700, FORWARD, V_MAX_DEF*0.5,//PROVERITI
					W_MAX_DEF*0.5,
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
		if (get_tact_num_ptr()->side)	// plava
			cur_task = task_sprat_12(BACKWARD);
		else
			cur_task = task_sprat_12(FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 190;
		break;
	case 190:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = task_sprat_3(BACKWARD);
		else
			cur_task = task_sprat_3(FORWARD);

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
		if (get_tact_num_ptr()->side)	// plava
			cur_task = task_sprat_12(FORWARD);
		else
			cur_task = task_sprat_12(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 205;
		break;

	case 205:
		grtl_back_close();
		grtl_front_close();
		tact_fsm_case = 210;
		break;

	case 210: //zavrsetak
		if (get_tact_num_ptr()->side)
			cur_task = move_to_xy(x_side(-900), 150, BACKWARD, 0.5, W_MAX_DEF,
					NO_SENS);
		else
			cur_task = move_to_xy(x_side(-1000), 500, FORWARD, 0.5, W_MAX_DEF,
					NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 9999;
		break;

		//	case 220: //zavrsetak
		//		if (get_tact_num_ptr()->side)
		//			cur_task = move_to_xy(x_side(-1200), 300, BACKWARD, 0.5, W_MAX_DEF,
		//			NO_SENS);
		//		else
		//			cur_task = move_to_xy(x_side(-1200), 300, FORWARD, 0.5, W_MAX_DEF,
		//			NO_SENS);
		//		if (cur_task == TASK_SUCCESS)
		//			tact_fsm_case = 9999;
		//		break;
	case 500:
		grtl_back_close ();
		HAL_Delay (10);
		grtl_front_close ();
		tact_fsm_case = 510;
		break;
	case 510: //rezervisan bunt
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy(x_side(-705), 400, FORWARD, V_MAX_DEF*0.5, W_MAX_DEF*0.5, FORWARD);
		else
			cur_task = move_to_xy(x_side(-665), 400, BACKWARD, V_MAX_DEF*0.5, W_MAX_DEF*0.5, BACKWARD);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 520;
		break;
	case 520:
		if (get_tact_num_ptr()->side)	// plava
		{
			cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
			prepare_front();
		} else {
			cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
			prepare_back();
		}
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 530;
		break;

	case 530:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_dir(185, FORWARD, 0.2, NO_SENS);//140
		else
			cur_task = move_on_dir(140, BACKWARD, 0.2, NO_SENS);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 540;
		break;
	case 540:
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
		tact_fsm_case = 550;
	case 550:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_dir(420, BACKWARD, 0.5, BACKWARD);//140
		else
			cur_task = move_on_dir(140, FORWARD, 0.5, FORWARD);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 560;
		break;
	case 560: //bunt kod protivnickog sa druge strane  gore
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy(x_side(1100), 350, BACKWARD, 0.75, //-1182
					W_MAX_DEF,
					BACKWARD);
		else
			cur_task = move_to_xy(x_side(1182), 582, FORWARD, V_MAX_DEF,
					W_MAX_DEF,
					FORWARD);

		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 570;
		break;
	case 570:
		prepare_back();
		tact_fsm_case = 575;
		break;
	case 575:
		cur_task = rot_to_phi(phi_side(180), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 580;
		break;
	case 580: //pred uzimanje bunta
		cur_task = move_on_dir(150, BACKWARD, 0.2, NO_SENS); //280
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 590;
		break;
	case 590:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 595;
		break;
	case 595:
		cur_task = move_on_dir(50, FORWARD, 0.2, NO_SENS); //280
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 600;
		break;
	case 600:
			cur_task = move_to_xy(x_side(950), -100, FORWARD, V_MAX_DEF*0.75, W_MAX_DEF*0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 610;
			break;
	case 610:
		cur_task = rot_to_phi(phi_side(180), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 615;
		break;
	case 615:
		cur_task = move_on_dir(250, BACKWARD, 0.5, NO_SENS); //280
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 620;
		break;
	case 620:
			if (get_tact_num_ptr()->side)	// plava
				cur_task = task_sprat_3_1_full(BACKWARD);
			else
				cur_task = task_sprat_3_1_full(FORWARD);
			//cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 630;
			break;





	}
	return tact_state;
}
