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

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t tact_0() {
	switch (tact_fsm_case) {
	case 0: //krece prednjom stranom ka ms24
		prepare_front();
		tact_fsm_case = 10;
		break;
	case 10:
		cur_task = move_on_path(x_side(700), -550, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 20;
		break;
	case 20:
		cur_task = move_to_xy(x_side(700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 30;
		break;
	case 30:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		prepare_back();
		tact_fsm_case = 40;
		break;
	case 40:
		cur_task = move_to_xy(x_side(700), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;
		break;
	case 50: //krece ka ms23
		cur_task = move_to_xy(x_side(1400), -575, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 60;
		break;
	case 60:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		tact_fsm_case = 70;
		break;
	case 70: //krece ka ca6
		cur_task = move_on_path(x_side(1350), -100, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		if (cur_task == TASK_FAIL)
			tact_fsm_case = 500;
		break;
	case 80: //gradi na ca6
		cur_task = task_sprat_3_1_full(FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 85;
		break;
	case 85:
		prepare_back();
		tact_fsm_case = 90;
		break;
	case 90: //krece prednjom stranom ka ms22
		cur_task = move_on_path(x_side(400), 200, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
	case 100:
		cur_task = move_to_xy(x_side(400), -100, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 110;
		break;
	case 110:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		prepare_front();
		tact_fsm_case = 120;
		break;
	case 120: //krece prednjom stranom ka ms14
		cur_task = move_on_path(x_side(-700), -550, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 130;
		break;
	case 130:
		cur_task = rot_to_phi(-90, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 140;
		break;
	case 140:
		cur_task = move_to_xy(x_side(-700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 150;
		break;
	case 150:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 160;
		break;
	case 160: //Baner
		cur_task = move_to_xy(x_side(-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 170;
		break;
	case 170:
		cur_task = rot_to_phi (180, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 180;
		break;
	case 180:
		bnr_1();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 185;
		break;
	case 185:
		bnr_2();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 190;
		break;
	case 190:
		cur_task = move_on_dir(500, BACKWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 195;
		break;
	case 195:
		bnr_3();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 200;
		break;
	case 200:
		bnr_4();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 210	;
		break;
	case 210: //Gradi na ca4/1
		cur_task = task_sprat_3_full(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 220;
		break;

	case 220: //krece prednjom stranom ka ms10
		cur_task = move_on_path(x_side(-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 230;
		break;
	case 230:
		cur_task = move_on_path(x_side(-650), 600, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 240;
		break;
	case 240:
		cur_task = rot_to_phi(90, W_MAX_DEF, ALL_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 245;
		break;
	case 245:
		prepare_front();
		tact_fsm_case = 250;
		break;
	case 250:
		cur_task = move_to_xy(x_side(-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 260;
		break;
	case 260:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 270;
		break;
	case 270: //krece zadnjom stranom ka ca1
		cur_task = move_on_path(x_side(-900), 0, 90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 280;
		break;
	case 280:
		cur_task = move_on_path(x_side(-700), -800, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 290;
		break;
	case 290: //gradi
		cur_task = task_sprat_3_half(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 310;
		break;
	case 310: //krece ka kraju
		cur_task = move_on_path(x_side(-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 320;
		break;
	case 320:
		cur_task = move_on_path(x_side(-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 330;
		break;

<<<<<<< Updated upstream
	case 500://ALTERNATIVNA KADA POGRESI NA CA6
		cur_task = move_on_path(x_side(1250), -850, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 510;
		break;
	case 520://krece ka ms21
		cur_task = move_on_path(x_side(1300), 350, 180, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
=======
	case 500: //krece ka ca5
		cur_task = move_on_path(x_side(1250), -850, 90, BACKWARD, 0, V_MAX_DEF,
				0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 510;
		break;
	case 510:
		cur_task = task_sprat_3_full(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 520;
		break;
	case 520: //krece ka ms21
		cur_task = move_on_path(x_side(1300), 350, 180, BACKWARD, 0, V_MAX_DEF,
				0, BACKWARD);
>>>>>>> Stashed changes
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 530;
		break;
	case 530:
		cur_task = move_to_xy(x_side(1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 535;
		break;
	case 535:
		prepare_front();
		tact_fsm_case = 540;
		break;
	case 540:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 550;
		break;
<<<<<<< Updated upstream
	case 550:
		cur_task = move_on_path(x_side(1350), -100, 180, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
=======
	case 550:///ca6
		cur_task = move_on_path(x_side(1350), -100, 180, BACKWARD, 0, V_MAX_DEF,
				0, BACKWARD);
>>>>>>> Stashed changes
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 560;
		break;
	case 560:
		cur_task = task_sprat_3_half(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 570;
		break;
	case 570://krece zadnjom stranom ka ms10
		cur_task = move_on_path(x_side(0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 580;
		break;
	case 580:
		cur_task = move_on_path(x_side(-650), 600, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 585;
		break;
	case 585:
		prepare_back();
		tact_fsm_case = 590;
		break;
	case 590:
		cur_task = move_to_xy(x_side(-650), 750, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 600;
		break;
	case 600:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		prepare_front();
		tact_fsm_case = 610;
		break;
	case 610: //krece prednjom stranom ka ms14
		cur_task = move_on_path(x_side(-900), 0, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 620;
		break;
	case 620:
		cur_task = move_on_path(x_side(-700), -550, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 630;
		break;
	case 630:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 640;
		break;
	case 640: //Baner
		cur_task = move_to_xy(x_side(-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 650;
		break;
	case 650:
		cur_task = rot_to_phi (180, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 660;
		break;
	case 660:
		bnr_1();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 670;
		break;
	case 670:
		bnr_2();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 680;
		break;
	case 680:
		cur_task = move_on_dir(500, BACKWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 690;
		break;
	case 690:
		bnr_3();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 700;
		break;
	case 700:
		bnr_4();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 710	;
		break;
	case 720: //Gradi na ca4/1
		cur_task = task_sprat_3_1_full(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 730;
		break;
	case 730: //krece ka kraju
		cur_task = move_on_path(x_side(-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 740;
		break;
	case 740:
		cur_task = move_on_path(x_side(-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 750;
		break;
<<<<<<< Updated upstream

=======
	case 1000: //krece ka ms13
		cur_task = move_on_path(x_side(-1300), -600, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1005;
		break;
	case 1005:
		front_prepare();
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 1006;
		break;
	case 1006:
		cur_task = move_to_xy(x_side(-1450), -600, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 1010;
		break;

	case 1010:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 1020;
		break;
	case 1020://krece ka ca4
		cur_task = move_on_path(x_side(-300), -400, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1030;
		break;
	case 1030:
		cur_task = task_sprat_3_half(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1040;
		break;
	case 1020://krece ka ms12
		cur_task = move_on_path(x_side(-400), -200, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1025;
		if(cur_task == TASK_FAIL)
			tact_fsm_case = 1500;
		break;
	case 1025:
		back_prepare();
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 1030;
		break;
	case 1030:
		cur_task = move_to_xy(x_side(-400), 0, BACKWARD, V_MAX_DEF, W_MAX_DEF,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1040;
		break;
	case 1040:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		prepare_front();
		tact_fsm_case = 1050;
		break;
	case 1050: //krece ka ms21
		cur_task = move_on_path(x_side(600), 350, 180, BACKWARD, 0, V_MAX_DEF,
				0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1060;
		break;
	case 1060:
		cur_task = move_on_path(x_side(1300), 350, 180, BACKWARD, 0, V_MAX_DEF,
				0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1070;
		break;
	case 1070:
		cur_task = move_to_xy(x_side(1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1080;
		break;
	case 1080:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 1090;
		break;
	case 1090:
		cur_task = move_to_xy(x_side(1450), 350, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1100;
		break;
	case 1100://baner
		cur_task = rot_to_phi(-90, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1110;
		break;
	case 1120:
		bnr_1();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1130;
		break;
	case 1130:
		bnr_2();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1140;
		break;
	case 1140:
		cur_task = move_on_dir(500, BACKWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1150;
		break;
	case 1150:
		bnr_3();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1160;
		break;
	case 1160:
		bnr_4();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1170;
		break;
	case 1170: //gradi na ca6/1
		cur_task = task_sprat_3_1_full(FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1180;
		break;
	case 1180: //krece ka kraju
		cur_task = move_on_path(x_side(0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1190;
		break;
	case 1190:
		cur_task = move_on_path(x_side(-1100), 800, -90, BACKWARD, 1, V_MAX_DEF, 0,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1200;
		break;

	case 1500: //krece prednjom stranom ka ms14
		cur_task = move_on_path(x_side(-300), -400, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1510;
		break;
	case 1510:
		cur_task = move_on_path(x_side(-700), -550, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1515;
		break;
	case 1515:
		prepare_back();
		if(cur_task == TASK_SUCCESS)
			tact_fsm_case = 1516;
		break;
	case 1516:
		cur_task = move_to_xy(x_side(-700), -750, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1517;
		break;
	case 1517:
		grtl_back_grip_all();
		ruc_back_carry();
		lift_back_carry();
		vacuum_back(1);
		prepare_front();
		tact_fsm_case = 1520;
		break;
	case 1520:
		cur_task = move_on_path(x_side(-700), -800, 90, BACKWARD, 0, V_MAX_DEF,	0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1530;
		break;
	case 1530: //gradi
		cur_task = task_sprat_1(BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1540;
		break;
	case 1540://krece ka ms12
		cur_task = move_on_path(x_side(-400), -200, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1550;
		if (cur_task == TASK_FAIL)
			tact_fsm_case = 2000;
		break;
	case 1550:
		cur_task = move_to_xy(x_side(-400), 0, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1560;
		break;
	case 1560:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 1570;
		break;
	case 1570://baner
		cur_task = move_on_path(x_side(1450), 300, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1580;
		break;
	case 1580:
		cur_task = rot_to_phi(-90, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1590;
		break;
	case 1590:
		bnr_1();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1600;
		break;
	case 1600:
		bnr_2();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1610;
		break;
	case 1610:
		cur_task = move_on_dir(500, BACKWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1620;
		break;
	case 1620:
		bnr_3();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1630;
		break;
	case 1630:
		bnr_4();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1635;
		break;
	case 1635:
		cur_task = rot_to_phi(90, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1640;
		break;
	case 1640:
		cur_task = task_sprat_3_half(BACKWARD)();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 1650;
		break;
	case 1650: //krece ka kraju
		cur_task = move_on_path(x_side(0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1660;
		break;
	case 1660:
		cur_task = move_on_path(x_side(-1100), 800, -90, BACKWARD, 1, V_MAX_DEF, 0,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 1670;
		break;

	case 2000:
		cur_task = move_on_path(x_side(-300), -700, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2010;
		break;
	case 2010: //krece prednjom stranom ka ms22
		cur_task = move_on_path(x_side(400), 200, 90, FORWARD, 0, V_MAX_DEF, 0,
				FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2015;
		break;
	case 2015:
		prepare_front();
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2020;
		break;
	case 2020:
		cur_task = move_to_xy(x_side(400), -100, FORWARD, V_MAX_DEF, W_MAX_DEF,
				FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2030;
		break;
	case 2030:
		grtl_front_grip_all();
		ruc_front_carry();
		lift_front_carry();
		vacuum_front(1);
		tact_fsm_case = 2040;
		break;
	case 2040://baner
		cur_task = move_on_path(x_side(1450), 300, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2050;
		break;
	case 2050:
		cur_task = rot_to_phi(-90, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2060;
		break;
	case 2060:
		bnr_1();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 2070;
		break;
	case 2070:
		bnr_2();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 2080;
		break;
	case 2080:
		cur_task = move_on_dir(500, BACKWARD, V_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2090;
		break;
	case 2090:
		bnr_3();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 2100;
		break;
	case 2110:
		bnr_4();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 2120;
		break;
	case 2120:
		cur_task = rot_to_phi(90, W_MAX_DEF, NO_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2130;
		break;
	case 2130:
		cur_task = task_sprat_3_half(BACKWARD)();
		if (delay_nb_2(&tact_delay_1, 500))
			tact_fsm_case = 2140;
		break;
	case 2140: //krece ka kraju
		cur_task = move_on_path(x_side(0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2150;
		break;
	case 2150:
		cur_task = move_on_path(x_side(-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0,
				BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 2160;
		break;





>>>>>>> Stashed changes





	}
	return tact_state;
}

