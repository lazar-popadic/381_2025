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

int8_t tact_3() {
	switch (tact_fsm_case) {
	case -1:
		break;
	case 0: //krece ka ms24
		get_robot_base()->x = x_side(-302);
		get_robot_base()->phi = phi_side(90);
		get_robot_base()->y = -710;
		tact_fsm_case = 2;
		break;
	case 2:
		cur_task = move_on_dir(170, BACKWARD, 0.5, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout(1000))
			tact_fsm_case = 5;
		break;
	case 5:
		lift_back_down_bnr();
		grtl_back_open_outside();
		if (delay_nb_2(&tact_delay_1, 500)) {
			tact_fsm_case = 7;
			add_points(20);
		}
		break;
	case 7:
		cur_task = move_on_dir(100, FORWARD, V_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 11;
		break;
	case 11:
		lift_back_down();
		grtl_back_close();
		tact_fsm_case = 12;
		break;
	case 12: //srednji bunt blizi njemu
		if (get_tact_num_ptr()->side) //plavu
			cur_task = move_on_path(x_side(-390), -400, phi_side(120), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		else
			//zuta
			cur_task = move_on_path(x_side(-370), -400, phi_side(120), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 13;
		break;
	case 13:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 15;
		break;
	case 15:
		prepare_front();
		tact_fsm_case = 20;
		break;
	case 20: //pred uzimanje bunta
		cur_task = move_on_dir_ortho(275, FORWARD, 0.2, FORWARD); //300
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
	case 40: //srednji bunt dalji njemu
		cur_task = move_on_path(x_side(390), 280, phi_side(0), FORWARD, 0,
		V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;
		break;
	case 50:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 60;
		break;
	case 60:
		prepare_back();
		tact_fsm_case = 70;
		break;
	case 70: //pred uzimanje bunta
		cur_task = move_on_dir_ortho(275, BACKWARD, 0.2, BACKWARD); //300
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		break;
	case 80:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 90;
		break;
	case 90:
		cur_task = move_to_xy(x_side(1200), -125, FORWARD, V_MAX_DEF,
		W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 95;
		break;
	case 95:
		cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
	case 100:
		cur_task = task_sprat_3_1_full(FORWARD);
		if (cur_task == TASK_SUCCESS) {
			tact_fsm_case = 110;
			add_points(32);
			grtl_front_close();
		}
		break;
	case 110:
		cur_task = move_to_xy(x_side(745), -450, BACKWARD, V_MAX_DEF, W_MAX_DEF,
		BACKWARD);
		//cur_task = move_on_path(x_side(700), -500, phi_side(90), BACKWARD, 0, V_MAX_DEF_PATH, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 115;
		break;
	case 115:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 120;
		break;
	case 120:
		prepare_back();
		tact_fsm_case = 130;
		break;
	case 130:
		cur_task = move_on_dir(300, BACKWARD, 0.2, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout(5000))
			tact_fsm_case = 140;
		break;
	case 140:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 150;
		break;
	case 150:
		cur_task = move_to_xy(x_side(1100), -600, FORWARD,
		V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 155;
		break;
	case 155:
		cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 160;
		break;
	case 160:
		prepare_front();
		tact_fsm_case = 170;
		break;
	case 170:
		cur_task = move_on_dir(500, FORWARD, 0.2, FORWARD);
		if (cur_task == TASK_SUCCESS || timeout(1500))
			tact_fsm_case = 180;
		break;
	case 180:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 190;
		break;
	case 190:
		cur_task = move_on_dir(400, FORWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 195;
		break;
	case 195:
		cur_task = move_to_xy(x_side(800), -125, BACKWARD, V_MAX_DEF, W_MAX_DEF,
		BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 200;
		break;
	case 200:
		cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 210;
		break;
	case 210:
		cur_task = task_sprat_12(FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 220;
		break;
	case 220:
		cur_task = move_on_dir(50, FORWARD, V_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 230;
		break;
	case 230:
		cur_task = task_sprat_3(FORWARD);
		if (cur_task == TASK_SUCCESS) {
			tact_fsm_case = 240;
			add_points(24);
		}
		break;
	case 240:
		cur_task = move_to_xy(x_side(-655), 370, BACKWARD, V_MAX_DEF, W_MAX_DEF,
		BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 250;
		break;

	case 250:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 260;
		break;

	case 260:
		prepare_front();
		tact_fsm_case = 270;
		break;

	case 270:
		cur_task = move_on_dir(240, FORWARD, 0.2, NO_SENS);

		if (cur_task == TASK_SUCCESS || timeout(1200))
			tact_fsm_case = 280;
		break;
	case 280:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 290;
		break;
	case 290:
		cur_task = move_to_xy(x_side(-150), -710, BACKWARD, V_MAX_DEF,
		W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 300;
		break;
	case 300:
		prepare_back();
		add_points(4);
		tact_fsm_case = 310;
		break;
	case 310:
		cur_task = move_on_dir(150, FORWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 320;
		break;
	case 320:
		cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 330;
		break;
	case 330:
		cur_task = task_sprat_12(FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 340;
		break;
	case 340:
		cur_task = move_on_dir(100, FORWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 350;
		break;
	case 350:
		cur_task = task_sprat_3(FORWARD);
		if (cur_task == TASK_SUCCESS) {
			tact_fsm_case = 360;
			add_points(24);
		}
		break;
	}
	return tact_state;
}
