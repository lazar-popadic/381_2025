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

int8_t tact_0() {
	switch (tact_fsm_case) {
	case -1:
		break;
	case 0: //krece ka ms24
		get_robot_base()->x = x_side(-322);
		get_robot_base()->phi = phi_side(90);
		get_robot_base()->y = -710;
		tact_fsm_case = 2;
		break;
	case 2:
		cur_task = move_on_dir(165, BACKWARD, 0.5, NO_SENS);
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
			cur_task = move_on_path(x_side(-410), -400, phi_side(120), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		else
			//zuta
			cur_task = move_on_path(x_side(-400), -400, phi_side(120), FORWARD,
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
		cur_task = move_on_dir_ortho(350, FORWARD, 0.2, FORWARD); //300
		if (delay_nb_2(&tact_delay_1, 1500))
			grtl_front_grip_all();
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
		// prilazi buntu pored startnog polja
	case 40:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy(x_side(-740), -350, BACKWARD, V_MAX_DEF,
			W_MAX_DEF, BACKWARD);
		else
			//zuta
			cur_task = move_to_xy(x_side(-740), -350, BACKWARD, V_MAX_DEF,
			W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;

		break;
	case 50:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 55;
		break;
	case 55:
		prepare_back();
		tact_fsm_case = 60;
		break;
		// gura bunt u manje polje
	case 60:
		cur_task = move_on_dir_ortho(440, BACKWARD, 0.2, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout(2500))
			tact_fsm_case = 61;
		break;
	case 61:
		cur_task = move_on_dir_ortho(255, FORWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 62;
		break;
	case 62:
		cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 63;
		break;
	case 63:
		cur_task = task_sprat_12(FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 64;
		break;
	case 64:
		HAL_Delay(200);
		tact_fsm_case = 66;
		break;
	case 66:
		cur_task = task_sprat_3(FORWARD);
		if (cur_task == TASK_SUCCESS || timeout(10000)) {
			tact_fsm_case = 80;
			add_points(28);
		}
		break;
	case 80:
		if (get_tact_num_ptr()->side)
		cur_task = move_to_xy(x_side(-1170), -640, FORWARD, V_MAX_DEF,
		W_MAX_DEF, FORWARD);
		else
			cur_task = move_to_xy(x_side(-1170), -630, FORWARD, V_MAX_DEF,
					W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 85;
		break;
	case 85:
		cur_task = rot_to_phi(phi_side(180), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 90;
		break;
	case 90:
		prepare_front();
		tact_fsm_case = 92;
		break;
	case 92:
		cur_task = move_on_dir_ortho(180, FORWARD, 0.2, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout(1250))
			tact_fsm_case = 94;
		break;

	case 94:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 96;
		break;
	case 96:
		cur_task = move_on_dir_ortho(450, BACKWARD, V_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
		// bunt ms11
	case 100:
		cur_task = move_to_xy(x_side(-700), 305, FORWARD, V_MAX_DEF,
		W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 102;
		break;

	case 102:
		cur_task = rot_to_phi(phi_side(0), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 104;
		break;
	case 104:
		prepare_back();
		tact_fsm_case = 105;
		break;
	case 105:
		cur_task = move_on_dir_ortho(400, BACKWARD, V_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 106;
		break;
	case 106:
		cur_task = move_on_dir_ortho(260, BACKWARD, 0.2, NO_SENS);
		if (cur_task == TASK_SUCCESS || timeout(2000))
			tact_fsm_case = 108;
		break;
	case 108:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		if (!check_back())
			tact_fsm_case = 500;
		else
			tact_fsm_case = 110;
		break;
	case 110:
		cur_task = move_on_dir_ortho(200, FORWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 120;
		break;
		// odlazak u polje
	case 120:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_path(x_side(-300), -650, phi_side(-90), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		else
			cur_task = move_on_path(x_side(-300), -650, phi_side(-90), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 130;
		break;
	case 130:
		cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 150;
		break;
	case 150:
		cur_task = task_sprat_3_1_full(FORWARD);
		if (cur_task == TASK_SUCCESS) {
			tact_fsm_case = 165;
			add_points(32);
		}
		break;
		//	case 160:
		//		cur_task = move_on_dir(150, BACKWARD, V_MAX_DEF, BACKWARD);
		//		if (cur_task == TASK_SUCCESS)
		//			tact_fsm_case = 165;
		//		break;
	case 165:
		grtl_back_close();
		grtl_front_close();
		tact_fsm_case = 170;
		break;

	case 170: //rezervisan bunt
		if (get_tact_num_ptr()->side) //plavu
			cur_task = move_to_xy(x_side(-720), 370, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
			BACKWARD);
		else
			cur_task = move_to_xy(x_side(-720), 370, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
			BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 180;
		break;

	case 180:
		cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 190;
		break;

	case 190:
		prepare_back();
		tact_fsm_case = 200;
		break;

	case 200:
		cur_task = move_on_dir_ortho(240, BACKWARD, 0.2, NO_SENS);

		if (cur_task == TASK_SUCCESS || timeout(2000))
			tact_fsm_case = 210;
		break;
	case 210:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 280;
		break;

	case 280:
		cur_task = move_on_path(x_side(-250), -200, phi_side(-30), FORWARD, 0,
		V_MAX_DEF_PATH, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 290;
		break;
	case 290:
		cur_task = rot_to_phi(phi_side(90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 295;
		break;
	case 295:
		cur_task = move_on_dir(70, BACKWARD, V_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 300;
		break;
	case 300:
		cur_task = task_sprat_12(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 305;
		break;
	case 305:
		HAL_Delay(200);
		tact_fsm_case = 310;
		break;
	case 310:
		cur_task = task_sprat_3(BACKWARD);
		if (cur_task == TASK_SUCCESS) {
			tact_fsm_case = 320;
			add_points(24);
			grtl_back_close();

		}
		break;
	case 320:
		cur_task = move_to_xy(x_side(-720), -100, FORWARD, V_MAX_DEF, W_MAX_DEF,
		FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 330;
		break;
	case 330:
		cur_task = rot_to_xy(x_side(-1000), 500, FORWARD, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 9999;
		break;

		// ALT 1
	case 500:
		vacuum_back(0);
		ruc_back_up();
		tact_fsm_case = 505;
		break;
	case 505:
		if (get_tact_num_ptr()->side) //plavu
			cur_task = move_to_xy(x_side(-690), 370, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
			FORWARD);
		else
			cur_task = move_to_xy(x_side(-690), 370, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
			FORWARD);
		if (cur_task == TASK_SUCCESS) {
			grtl_back_close();
			lift_back_down();
			tact_fsm_case = 510;
		}
		break;

	case 510:
		cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 520;
		break;

	case 520:
		prepare_back();
		tact_fsm_case = 530;
		break;

	case 530:
		cur_task = move_on_dir_ortho(240, BACKWARD, 0.2, NO_SENS);

		if (cur_task == TASK_SUCCESS || timeout(2000))
			tact_fsm_case = 540;
		break;
	case 540:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 550;
		break;
	case 550:
		if (get_tact_num_ptr()->side)	// plava
			cur_task = move_on_path(x_side(-250), -650, phi_side(-90), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		else
			cur_task = move_on_path(x_side(-250), -650, phi_side(-90), FORWARD,
					0, V_MAX_DEF_PATH, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 560;
		break;
	case 560:
		cur_task = rot_to_phi(phi_side(-90), W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 570;
		break;
	case 570:
		cur_task = task_sprat_3_1_full(FORWARD);
		if (cur_task == TASK_SUCCESS) {
			tact_fsm_case = 580;
			add_points(32);
		}
		break;
	case 580:
		grtl_back_close();
		grtl_front_close();
		tact_fsm_case = 320;
		break;
	}
	return tact_state;
}
