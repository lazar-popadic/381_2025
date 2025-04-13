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

int8_t
tact_0 ()
{
	switch (tact_fsm_case)
		{
		case 0: //krece prednjom stranom ka ms24
			get_robot_base ()->x = 1400;
			// TODO: strane, ovo je za zutu, ne radi x_side
			get_robot_base ()->phi = 180;
			get_robot_base ()->y = -100;
			prepare_front ();
			tact_fsm_case = 10;
			break;
		case 10:
			cur_task = move_on_path (x_side (700), -550, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;
		case 20:
			cur_task = move_to_xy (x_side (700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
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
		case 40:
			cur_task = move_to_xy (x_side (700), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50: //krece ka ms23
			cur_task = move_to_xy (x_side (1400), -575, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
															BACKWARD);
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
		case 70: //krece ka ca6
			cur_task = move_on_path (x_side (1350), -100, 0, FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 75;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 500;
			break;
		case 75:
			cur_task = rot_to_phi (0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 80;
			break;
		case 80: //gradi na ca6
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 85;
			break;
		case 85:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 90;
			break;
		case 90: //krece prednjom stranom ka ms22
			cur_task = move_on_path (x_side (400), 200, 90, BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 100;
			break;
		case 100:
			cur_task = move_to_xy (x_side (400), -100, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 110;
			break;
		case 110:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			prepare_front ();
			tact_fsm_case = 120;
			break;
		case 120: //krece prednjom stranom ka ms14
			cur_task = move_on_path (x_side (-700), -550, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;
		case 140:
			cur_task = move_to_xy (x_side (-700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 150;
			break;
		case 150:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 160;
			break;
		case 160: //Baner
			cur_task = move_to_xy (x_side (-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 170;
			break;
		case 170:
			cur_task = rot_to_phi (180, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 180;
			break;
		case 180:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 185;
			break;
		case 185:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 190;
			break;
		case 190:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 195;
			break;
		case 195:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 200;
			break;
		case 200:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 210;
			break;
		case 210: //Gradi na ca4/1
			cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 220;
			break;

		case 220: //krece prednjom stranom ka ms10
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 230;
			break;
		case 230:
			cur_task = move_on_path (x_side (-650), 600, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 240;
			break;
		case 240:
			cur_task = rot_to_phi (90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 245;
			break;
		case 245:
			prepare_front ();
			tact_fsm_case = 250;
			break;
		case 250:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 260;
			break;
		case 260:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 270;
			break;
		case 270: //krece zadnjom stranom ka ca1
			cur_task = move_on_path (x_side (-900), 0, 90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 280;
			break;
		case 280:
			cur_task = move_on_path (x_side (-700), -800, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 290;
			break;
		case 290: //gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 310;
			break;
		case 310: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 320;
			break;
		case 320:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 330;
			break;
		case 500: //ALTERNATIVNA KADA POGRESI NA CA6
			cur_task = move_on_path (x_side (1250), -850, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 510;
			break;
		case 510:
			cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 520;
			break;
		case 520: //krece ka ms21
			cur_task = move_on_path (x_side (1300), 350, 180, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);

			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 530;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 1000;
			break;
		case 530:
			cur_task = move_to_xy (x_side (1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 535;
			break;
		case 535:
			prepare_front ();
			tact_fsm_case = 540;
			break;
		case 540:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 550;
			break;
		case 550: ///ca6
			cur_task = move_on_path (x_side (1350), -100, 180, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 560;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 4000;
			break;
		case 560:
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 570;
			break;
		case 570: //krece zadnjom stranom ka ms10
			cur_task = move_on_path (x_side (0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 580;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 4500;
			break;
		case 580:
			cur_task = move_on_path (x_side (-650), 600, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 585;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 4800;
			break;
		case 585:
			prepare_back ();
			tact_fsm_case = 590;
			break;
		case 590:
			cur_task = move_to_xy (x_side (-650), 750, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 600;
			break;
		case 600:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			prepare_front ();
			tact_fsm_case = 610;
			break;
		case 610: //krece prednjom stranom ka ms14
			cur_task = move_on_path (x_side (-900), 0, -90, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 620;
			break;
		case 620:
			cur_task = move_on_path (x_side (-700), -550, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 630;
			break;
		case 630:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 640;
			break;
		case 640: //Baner
			cur_task = move_to_xy (x_side (-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 650;
			break;
		case 650:
			cur_task = rot_to_phi (180, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 660;
			break;
		case 660:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 670;
			break;
		case 670:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 680;
			break;
		case 680:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 690;
			break;
		case 690:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 700;
			break;
		case 700:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 710;
			break;
		case 720: //Gradi na ca4/1
			cur_task = task_sprat_3_1_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 730;
			break;
		case 730: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 740;
			break;
		case 740:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 750;
			break;
		case 1000: //krece ka ms13
			cur_task = move_on_path (x_side (-1300), -600, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1005;
			break;
		case 1005:
			prepare_front ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1006;
			break;
		case 1006:
			cur_task = move_to_xy (x_side (-1450), -600, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1010;
			break;

		case 1010:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 1015;
			break;
		case 1015: //krece ka ca4
			cur_task = move_on_path (x_side (-300), -400, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1020;
			break;
		case 1020:
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1025;
			break;
		case 1025: //krece ka ms12
			cur_task = move_on_path (x_side (-400), -200, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1030;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 1500;
			break;
		case 1030:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1035;
			break;
		case 1035:
			cur_task = move_to_xy (x_side (-400), 0, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1040;
			break;
		case 1040:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			prepare_front ();
			tact_fsm_case = 1050;
			break;
		case 1050: //krece ka ms21
			cur_task = move_on_path (x_side (600), 350, 180, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1060;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3500;
			break;
		case 1060:
			cur_task = move_on_path (x_side (1300), 350, 180, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1070;
			break;
		case 1070:
			cur_task = move_to_xy (x_side (1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1080;
			break;
		case 1080:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 1090;
			break;
		case 1090: //baner
			cur_task = move_to_xy (x_side (1450), 350, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1100;
			break;
		case 1100:
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1110;
			break;
		case 1120:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1130;
			break;
		case 1130:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1140;
			break;
		case 1140:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1150;
			break;
		case 1150:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1160;
			break;
		case 1160:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1170;
			break;
		case 1170: //gradi na ca6/1
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1180;
			break;
		case 1180: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1190;
			break;
		case 1190:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1200;
			break;

		case 1500: //krece prednjom stranom ka ms14
			cur_task = move_on_path (x_side (-300), -400, 90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1510;
			break;
		case 1510:
			cur_task = move_on_path (x_side (-700), -550, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1515;
			break;
		case 1515:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1516;
			break;
		case 1516:
			cur_task = move_to_xy (x_side (-700), -750, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1517;
			break;
		case 1517:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			prepare_front ();
			tact_fsm_case = 1520;
			break;
		case 1520:
			cur_task = move_on_path (x_side (-700), -800, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1530;
			break;
		case 1530: //gradi
			cur_task = task_sprat_1 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1540;
			break;
		case 1540: //krece ka ms12
			cur_task = move_on_path (x_side (-400), -200, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1550;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 2000;
			break;
		case 1550:
			cur_task = move_to_xy (x_side (-400), 0, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1560;
			break;
		case 1560:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 1570;
			break;
		case 1570: //baner
			cur_task = move_on_path (x_side (1450), 300, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1580;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3000;
			break;
		case 1580:
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1590;
			break;
		case 1590:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1600;
			break;
		case 1600:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1610;
			break;
		case 1610:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1620;
			break;
		case 1620:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1630;
			break;
		case 1630:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1635;
			break;
		case 1635:
			cur_task = rot_to_phi (90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1640;
			break;
		case 1640:
			cur_task = task_sprat_3_half (BACKWARD);
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1650;
			break;
		case 1650: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1660;
			break;
		case 1660:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1670;
			break;

		case 2000:
			cur_task = move_on_path (x_side (-300), -700, 90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2010;
			break;
		case 2010: //krece prednjom stranom ka ms22
			cur_task = move_on_path (x_side (400), 200, 90, FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2015;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 2500;
			break;
		case 2015:
			prepare_front ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2020;
			break;
		case 2020:
			cur_task = move_to_xy (x_side (400), -100, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2030;
			break;
		case 2030:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 2040;
			break;
		case 2040: //baner
			cur_task = move_on_path (x_side (1450), 300, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2050;
			break;
		case 2050:
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2060;
			break;
		case 2060:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2070;
			break;
		case 2070:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2080;
			break;
		case 2080:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2090;
			break;
		case 2090:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2100;
			break;
		case 2110:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2120;
			break;
		case 2120:
			cur_task = rot_to_phi (90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2130;
			break;
		case 2130:
			cur_task = task_sprat_3_half (BACKWARD);
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2140;
			break;
		case 2140: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2150;
			break;
		case 2150:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2160;
			break;
		case 2500: //krece prednjom stranom ka ms10
			cur_task = move_on_path (x_side (-900), 0, -90, FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2510;
			break;
		case 2510:
			cur_task = move_on_path (x_side (-650), 600, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2520;
			break;
		case 2520:
			prepare_back ();
			tact_fsm_case = 2530;
			break;
		case 2530:
			cur_task = move_to_xy (x_side (-650), 750, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2540;
			break;
		case 2540:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 2550;
			break;
		case 2550:
			cur_task = move_on_path (x_side (0), 200, 0, FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2510;
			break;
		case 2560:
			cur_task = move_on_path (x_side (-1450), 0, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2570;
			break;
		case 2570: //baner
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2580;
			break;
		case 2580:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2590;
			break;
		case 2590:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2600;
			break;
		case 2600:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2610;
			break;
		case 2610:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2620;
			break;
		case 2620:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2630;
			break;
		case 2630: //gradi na ca6/1
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2640;
			break;
		case 2640: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, 0, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2650;
			break;
		case 2660:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2670;
			break;

		case 3000: //baner krece
			cur_task = move_to_xy (x_side (-400), 0, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3010;
			break;
		case 3010:
			cur_task = move_on_path (x_side (1450), -600, 180, BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3020;
			break;
		case 3030:
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3040;
			break;
		case 3050:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3060;
			break;
		case 3060:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3070;
			break;
		case 3070:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3080;
			break;
		case 3080:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3090;
			break;
		case 3090:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3100;
			break;
		case 3100: // gradi na ca6/1
			cur_task = task_sprat_3_1_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3110;
			break;
		case 3110: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, 0, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3120;
			break;
		case 3120:
			cur_task = move_on_path (x_side (-1100), 800, -90, FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3130;
			break;

		case 3500:
			cur_task = move_on_path (x_side (-400), 0, -90, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3510;
			break;
		case 3510:
			cur_task = move_on_path (x_side (1450), -500, 0, FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3520;
			break;
		case 3520: //baner
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3530;
			break;
		case 3530:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3540;
			break;
		case 3540:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3550;
			break;
		case 3550:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3560;
			break;
		case 3560:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3570;
			break;
		case 3570:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 3575;
			break;
		case 3575: // gradi na ca6/1
			cur_task = task_sprat_1 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3580;
			break;
		case 3580: //krece zadnjom stranom ka ms10
			cur_task = move_on_path (x_side (0), 200, 180, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3590;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3700;
			break;
		case 3590:
			cur_task = move_on_path (x_side (-650), 600, 90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3600;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3800;
			break;
		case 3610:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3620;
			break;
		case 3620:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 3630;
			break;
		case 3630: //krece zadnjom stranom ka ca4
			cur_task = move_on_path (x_side (-900), 0, 90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3640;
			break;
		case 3640:
			cur_task = move_on_path (x_side (-300), -850, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3650;
			break;
		case 3650: //gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3660;
			break;
		case 3660: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3670;
			break;
		case 3670:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3680;
			break;

		case 3700: //krece ka ms14
			cur_task = move_on_path (x_side (900), 0, 90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3710;
			break;
		case 3710:
			cur_task = move_on_path (x_side (-700), -550, 0, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3720;
			break;
		case 3720:
			cur_task = move_to_xy (x_side (-700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3730;
			break;
		case 3730:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 3740;
			break;
		case 3740:
			cur_task = move_to_xy (x_side (-700), -800, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3750;
			break;
		case 3750:
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3760;
			break;
		case 3760: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3770;
			break;
		case 3770:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3780;
			break;

		case 3800: //krece zadnjom stranom ka ms22
			cur_task = move_on_path (x_side (400), 100, 90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3805;
			break;
		case 3805:
			prepare_front ();
			tact_fsm_case = 3810;
			break;
		case 3810:
			cur_task = move_to_xy (x_side (400), -100, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3820;
			break;
		case 3820:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 3830;
			break;
		case 3830: //ca4/1
			cur_task = move_on_path (x_side (1300), -400, 0, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3840;
			break;
		case 3840:
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3850;
			break;
		case 3850: //gradi na ca6
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3860;
			break;
		case 3860: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3870;
			break;
		case 3870:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3880;
			break;

		case 4000: //KRECE KA CA6/1
			cur_task = move_on_path (x_side (1300), 350, 0, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4010;
			break;
		case 4010:
			cur_task = rot_to_phi (-90, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4020;
			break;
		case 4020:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4030;
			break;
		case 4030:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4040;
			break;
		case 4040:
			cur_task = move_on_dir (500, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4050;
			break;
		case 4050:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4060;
			break;
		case 4060:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4070;
			break;
		case 4070: //gradi na ca6/1
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4080;
			break;
		case 4080: //krece ka ms12
			cur_task = move_on_path (x_side (-400), 100, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4090;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5000;
			break;
		case 4090:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4100;
			break;
		case 4100:
			cur_task = move_to_xy (x_side (-400), 0, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4110;
			break;
		case 4110:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			prepare_back ();
			tact_fsm_case = 4120;
			break;
		case 4120: //KRECE KA MS10
			cur_task = move_on_path (x_side (-650), 600, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4130;
			break;
		case 4130:
			cur_task = move_to_xy (x_side (-650), 750, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4140;
			break;
		case 4140:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 4150;
			break;
		case 4150: //krece zadnjom stranom ka ca4
			cur_task = move_on_path (x_side (-900), 0, 90, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4160;
			break;
		case 4160:
			cur_task = move_on_path (x_side (-300), -850, 90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4170;
			break;
		case 4170: //gradi
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4180;
			break;
		case 4180: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4190;
			break;
		case 4190:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4200;
			break;

		case 4500: //krece prednjom stranom ka ms14
			cur_task = move_on_path (x_side (900), 0, -90, FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4505;
			break;
		case 4505:
			cur_task = move_on_path (x_side (-700), -550, -90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4510;
			break;
		case 4510:
			cur_task = move_to_xy (x_side (-700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4520;
			break;
		case 4520:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 4530;
			break;
		case 4530: //Baner
			cur_task = move_to_xy (x_side (-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4540;
			break;
		case 4540:
			cur_task = rot_to_phi (180, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4550;
			break;
		case 4550:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4560;
			break;
		case 4560:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4570;
			break;
		case 4570:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4580;
			break;
		case 4580:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4590;
			break;
		case 4590:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 4600;
			break;
		case 4600: //Gradi na ca4/1
			cur_task = task_sprat_1 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4610;
			break;
		case 4610: //krece prednjom stranom ka ms10
			cur_task = move_on_path (x_side (-900), 0, 90, FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4620;
			break;
		case 4620:
			cur_task = move_on_path (x_side (-650), 600, 90, FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4630;
			break;
		case 4630:
			prepare_front ();
			tact_fsm_case = 4640;
			break;
		case 4640:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4650;
			break;
		case 4650:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 4660;
			break;
		case 4660: // krece zadnjom stranom ka ca1
			cur_task = move_on_path (x_side (-900), 0, 90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4670;
			break;
		case 4670:
			cur_task = move_on_path (x_side (-700), -800, 90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4680;
			break;
		case 4680: // gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4690;
			break;
		case 4690: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, -90, BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4700;
			break;
		case 4700:
			cur_task = move_on_path (x_side (-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4710;
			break;
		case 4800:
			cur_task = move_on_path (x_side (0), 200, 0, FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4500;
			break;
		case 5000:

			break;

		}
	return tact_state;
}

