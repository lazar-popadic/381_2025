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
		case 0: //krece ka ms24
			get_robot_base ()->x = x_side (1216);
			// TODO: strane, ovo je za zutu, ne radi x_side
			get_robot_base ()->phi = phi_side (180);
			get_robot_base ()->y = -78;
			prepare_front ();
			tact_fsm_case = 10;
			break;
		case 10:
			cur_task = move_on_path (x_side (730), -450, phi_side (-90), FORWARD, 0, 0.5, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11;
			break;
		case 11:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;
		case 20:
			cur_task = move_to_xy (x_side (730), -675, FORWARD, 0.5, W_MAX_DEF,
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
		case 40:
			cur_task = move_to_xy (x_side (1035), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50: //krece ka ms23
			cur_task = move_to_xy (x_side (1335), -600, BACKWARD, 0.5,
			W_MAX_DEF,
															NO_SENS);
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
			cur_task = move_on_path (x_side (1120), -120, phi_side (45), FORWARD, 0, 0.5, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 75;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 500;
			break;
		case 75:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 77;
			break;
		case 77:
			cur_task = move_on_dir (100, FORWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 80;
			break;
		case 80: //gradi na ca6
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
		case 90: //krece ka ms22
			cur_task = move_on_path (x_side (450), 200, phi_side (0), BACKWARD, 0, 0.3, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 91;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 10000;
			break;
		case 91:
			prepare_back ();
			tact_fsm_case = 95;
			break;
		case 95:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 100;
			break;
		case 100:
			cur_task = move_on_dir (250, BACKWARD, 0.5, BACKWARD);
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
		case 120: //krece ka ms14
			cur_task = move_on_path (x_side (-730), -450, phi_side (0), BACKWARD, 0, 0.5, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 130;
			break;
		case 130:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;
		case 140:
			cur_task = move_to_xy (x_side (-730), -675, FORWARD, 0.5, W_MAX_DEF,
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
			cur_task = move_to_xy (x_side (-700), -725, FORWARD, 0.5, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 170;
			break;
		case 170:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
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

		case 220: //krece ka ms10
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 230;
			break;
		case 230:
			cur_task = move_on_path (x_side (-650), 600, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 240;
			break;
		case 240:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
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
		case 270: //krece   ka ca1
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 280;
			break;
		case 280:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 285;
			break;
		case 285:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 290;
			break;
		case 290: //gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 310;
			break;
		case 310: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 320;
			break;
		case 320:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 330;
			break;

		case 500: //ca5
			cur_task = move_on_path (x_side (1250), -850, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 505;
			break;
		case 505:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 510;
			break;
		case 510:
			cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 520;
			break;
		case 520: //krece ka ms21
			cur_task = move_on_path (x_side (1300), 350, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 525;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 1000;
			break;
		case 525:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 535;
			break;
		case 535:
			prepare_front ();
			tact_fsm_case = 530;
			break;
		case 530:
			cur_task = move_to_xy (x_side (1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
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
			cur_task = move_on_path (x_side (1350), -100, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 555;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 4000;
			break;
		case 555:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 560;
			break;
		case 560:
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 570;
			break;
		case 570: //krece ka ms10
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 580;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 4500;
			break;
		case 580:
			cur_task = move_on_path (x_side (-650), 600, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 581;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 4800;
			break;
		case 581:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 585;
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
		case 610: //krece ka ms14
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 620;
			break;
		case 620:
			cur_task = move_on_path (x_side (-700), -550, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 625;
			break;
		case 625:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
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
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 740;
			break;
		case 740:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 750;
			break;

		case 1000: //krece ka ms13
			cur_task = move_on_path (x_side (-1300), -600, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1002;
			break;
		case 1002:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (-300), -400, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1019;
			break;
		case 1019:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1020;
			break;
		case 1020:
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1025;
			break;
		case 1025: //krece ka ms12
			cur_task = move_on_path (x_side (-400), -200, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1028;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 1500;
			break;
		case 1028:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1030;
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
			cur_task = move_on_path (x_side (600), 350, phi_side (180), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1060;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3500;
			break;
		case 1060:
			cur_task = move_on_path (x_side (1300), 350, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1065;
			break;
		case 1065:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
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
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1190;
			break;
		case 1190:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1200;
			break;

		case 1500: //krece ka ms14
			cur_task = move_on_path (x_side (-300), -400, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1510;
			break;
		case 1510:
			cur_task = move_on_path (x_side (-700), -550, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1515;
			break;
		case 1514:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1525;
			break;
		case 1525:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1530;
			break;
		case 1530: //gradi
			cur_task = task_sprat_1 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1540;
			break;
		case 1540: //krece ka ms12
			cur_task = move_on_path (x_side (-400), -200, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
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
			cur_task = move_on_path (x_side (1450), 300, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1580;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3000;
			break;
		case 1580:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1640;
			break;
		case 1640:
			cur_task = task_sprat_3_half (BACKWARD);
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 1650;
			break;
		case 1650: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1660;
			break;
		case 1660:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1670;
			break;

		case 2000:
			cur_task = move_on_path (x_side (-300), -700, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2010;
			break;
		case 2010: //krece ka ms22
			cur_task = move_on_path (x_side (400), 200, phi_side (90), FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2014;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 2500;
			break;
		case 2014:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2015;
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
			cur_task = move_on_path (x_side (1450), 300, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2050;
			break;
		case 2050:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2130;
			break;
		case 2130:
			cur_task = task_sprat_3_half (BACKWARD);
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 2140;
			break;
		case 2140: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2150;
			break;
		case 2150:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2160;
			break;

		case 2500: //krece ka ms10
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2510;
			break;
		case 2510:
			cur_task = move_on_path (x_side (-650), 600, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2515;
			break;
		case 2515:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (0), 200, phi_side (0), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2510;
			break;
		case 2560:
			cur_task = move_on_path (x_side (-1450), 0, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2570;
			break;
		case 2570: //baner
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 2650;
			break;
		case 2660:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
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
			cur_task = move_on_path (x_side (1450), -600, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3020;
			break;
		case 3030:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (0), 200, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3120;
			break;
		case 3120:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3130;
			break;

		case 3500:
			cur_task = move_on_path (x_side (-400), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3510;
			break;
		case 3510:
			cur_task = move_on_path (x_side (1450), -500, phi_side (0), FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3520;
			break;
		case 3520: //baner
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
		case 3580: //krece ka ms10
			cur_task = move_on_path (x_side (0), 200, phi_side (180), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3590;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3700;
			break;
		case 3590:
			cur_task = move_on_path (x_side (-650), 600, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3595;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 3800;
			break;
		case 3595:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3610;
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
		case 3630: //krece ka ca4
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3640;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6100;
			break;
		case 3640:
			cur_task = move_on_path (x_side (-300), -850, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3645;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6200;
			break;
		case 3645:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3650;
			break;
		case 3650: //gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3660;
			break;
		case 3660: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3670;
			break;
		case 3670:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3680;
			break;

		case 3700: //krece ka ms14
			cur_task = move_on_path (x_side (900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3710;
			break;
		case 3710:
			cur_task = move_on_path (x_side (-700), -550, phi_side (0), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3715;
			break;
		case 3715:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3770;
			break;
		case 3770:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3780;
			break;

		case 3800: //krece ka ms22
			cur_task = move_on_path (x_side (400), 100, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3802;
			break;
		case 3802:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (1300), -400, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3840;
			break;
		case 3840:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3850;
			break;
		case 3850: //gradi na ca6
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3860;
			break;
		case 3860: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3870;
			break;
		case 3870:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 3880;
			break;

		case 4000: //KRECE KA CA6/1
			cur_task = move_on_path (x_side (1300), 350, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4010;
			break;
		case 4010:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = move_on_path (x_side (-400), 100, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4085;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5000;
			break;
		case 4085:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4090;
			break;
		case 4090:
			prepare_front ();
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
			cur_task = move_on_path (x_side (-650), 600, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4125;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5500;
			break;
		case 4125:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
		case 4150: //krece ka ca4
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4160;
			break;
		case 4160:
			cur_task = move_on_path (x_side (-300), -850, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4165;
			break;
		case 4165:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4170;
			break;
		case 4170: //gradi
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4180;
			break;
		case 4180: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4190;
			break;
		case 4190:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4200;
			break;

		case 4500: //krece ka ms14
			cur_task = move_on_path (x_side (900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4505;
			break;
		case 4505:
			cur_task = move_on_path (x_side (-700), -550, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4507;
			break;
		case 4507:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
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
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
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
		case 4610: //krece ka ms10
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4620;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6300;
			break;
		case 4620:
			cur_task = move_on_path (x_side (-650), 600, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4625;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6500;
			break;
		case 4625:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
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
		case 4660: // krece ka ca1
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4670;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 7000;
			break;
		case 4670:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4675;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 7000;
			break;
		case 4675:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4680;
			break;
		case 4680: // gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4690;
			break;
		case 4690: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4700;
			break;
		case 4700:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4710;
			break;

		case 4800:
			cur_task = move_on_path (x_side (0), 200, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 4500;
			break;

		case 5000: // krece ka ms14
			cur_task = move_on_path (x_side (900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5010;
			break;
		case 5010:
			cur_task = move_on_path (x_side (-700), -550, phi_side (0), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5015;
			break;
		case 5015:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5020;
			break;
		case 5020:
			cur_task = move_to_xy (x_side (-700), -800, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5030;
			break;
		case 5030:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			prepare_front ();
			tact_fsm_case = 5040;
			break;
		case 5040: // krece ka ms10
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5050;
			break;
		case 5050:
			cur_task = move_on_path (x_side (-650), 600, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5060;
			break;
		case 5060:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5070;
			break;
		case 5070:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5080;
			break;
		case 5080:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 5090;
			break;
		case 5090: // krece ka ca4
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5100;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5200;
			break;
		case 5100:
			cur_task = move_on_path (x_side (-300), -850, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5105;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5300;
			break;
		case 5105:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5110;
			break;
		case 5110: // gradi
			cur_task = task_sprat_3_1_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5120;
			break;
		case 5120: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5130;
			break;
		case 5130:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5140;
			break;

		case 5200: //krece ka ca4
			cur_task = move_on_path (x_side (0), 200, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5210;
			break;
		case 5210:
			cur_task = move_on_path (x_side (900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5220;
			break;
		case 5220:
			cur_task = move_on_path (x_side (-300), -850, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5225;
			break;
		case 5225:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5230;
			break;
		case 5230: // gradi
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5240;
			break;
		case 5240: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5250;
			break;
		case 5250:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5260;
			break;

		case 5300: //KRECE KA CA4
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5200;
			break;

		case 5500: //krece ka ms13
			cur_task = move_on_path (x_side (-400), 100, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5510;
			break;
		case 5510:
			cur_task = move_on_path (x_side (-1300), -600, phi_side (0), FORWARD, 180, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5515;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5700;
			break;
		case 5515:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5520;
			break;
		case 5520:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5530;
			break;
		case 5530:
			cur_task = move_to_xy (x_side (-1450), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5540;
			break;
		case 5540:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 5550;
			break;
		case 5550: //krece ka ca4
			cur_task = move_on_path (x_side (-300), -700, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5555;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6000;
			break;
		case 5555:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5560;
			break;
		case 5560: //gradi na ca6
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5570;
			break;
		case 5570: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5580;
			break;
		case 5580:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5590;
			break;

		case 5700: //KRECE KA MS10
			cur_task = move_on_path (x_side (400), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5710;
			break;

		case 5710:
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5720;
			break;
		case 5720:
			cur_task = move_on_path (x_side (-650), 600, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5725;
			break;
		case 5725:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5730;
			break;
		case 5730:
			prepare_back ();
			tact_fsm_case = 5740;
			break;
		case 5740:
			cur_task = move_to_xy (x_side (-650), 750, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5750;
			break;
		case 5750:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 5760;
			break;
		case 5760: // krece ka ca4
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5770;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 5900;
			break;
		case 5770:
			cur_task = move_on_path (x_side (-300), -850, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5775;
			break;
		case 5775:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5780;
			break;
		case 5780:
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5790;
			break;
		case 5790: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5800;
			break;
		case 5800:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5810;
			break;

		case 5900: //krece ka ca4
			cur_task = move_on_path (x_side (0), 200, phi_side (180), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5910;
			break;
		case 5910: //ca5
			cur_task = move_on_path (x_side (900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5920;
			break;
		case 5920: //krece ka ca4
			cur_task = move_on_path (x_side (-300), -850, phi_side (0), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5925;
			break;
		case 5925:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5930;
			break;
		case 5930:
			cur_task = task_sprat_3_1_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5940;
			break;
		case 5940: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5950;
			break;
		case 5950:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 5960;
			break;

			//dodato rot do ovde

		case 6000: //krece ka ca6/1
			cur_task = move_on_path (x_side (-1300), -600, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6010;
			break;
		case 6010:
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6020;
			break;
		case 6020:
			cur_task = move_on_path (x_side (0), 200, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6025;
			break;
		case 6025:
			cur_task = move_on_path (x_side (1300), 0, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6030;
			break;
		case 6030:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6040;
			break;
		case 6040: //gradi na ca6
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6050;
			break;
		case 6050: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6060;
			break;
		case 6060:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6070;
			break;

		case 6100:
			cur_task = move_on_path (x_side (-650), 600, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6110;
			break;
		case 6110: //krece ka ca6
			cur_task = move_on_path (x_side (1350), -100, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6120;
			break;
		case 6120:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6130;
			break;
		case 6130: //gradi na ca6
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6140;
			break;
		case 6140: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6150;
			break;
		case 6150:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6160;
			break;

		case 6200: //krece ka ca4
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6100;
			break;

		case 6300: //krece ka ms13
			cur_task = move_on_path (x_side (-700), -550, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6310;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6600;
			break;
		case 6310:
			cur_task = move_on_path (x_side (-1300), -600, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6320;
			break;
		case 6320:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6330;
			break;
		case 6330:
			prepare_front ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6340;
			break;
		case 6340:
			cur_task = move_to_xy (x_side (-1450), -600, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6350;
			break;
		case 6350:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 6360;
			break;
		case 6360: // krece ka ca1
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6370;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6800;
			break;
		case 6370:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6375;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 6900;
			break;
		case 6375:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6380;
			break;
		case 6380: // gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6390;
			break;
		case 6390: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6400;
			break;
		case 6400:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6410;
			break;

		case 6500:
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6300;
			break;

		case 6600: // krece ka ms10
			cur_task = move_on_path (x_side (900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6610;
			break;
		case 6610:
			cur_task = move_on_path (x_side (-650), 600, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6620;
			break;
		case 6620:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6630;
			break;
		case 6630:
			prepare_front ();
			tact_fsm_case = 6640;
			break;
		case 6640:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6650;
			break;
		case 6650:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 6660;
			break;
		case 6660: // krece ka ca1
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6670;
			break;
		case 6670:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6680;
			break;
		case 6680:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6690;
			break;
		case 6690: // gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6700;
			break;
		case 6700: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6710;
			break;

		case 6800:
			cur_task = move_on_path (x_side (-1300), -600, phi_side (180), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6810;
			break;
		case 6810: //KRECE KA CA6/1
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6820;
			break;
		case 6820:
			cur_task = move_on_path (x_side (0), 200, phi_side (180), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6830;
			break;
		case 6830: //KRECE KA CA6/1
			cur_task = move_on_path (x_side (1350), 100, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6840;
			break;
		case 6840:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6850;
			break;
		case 6850: //gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6860;
			break;
		case 6860: //krece ka kraju
			cur_task = move_on_path (x_side (0), 200, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6870;
			break;
		case 6870:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6880;
			break;

		case 6900: // krece ka ca6
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6800;
			break;

		case 7000:
			cur_task = move_to_xy (x_side (-650), 650, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 6820;
			break;

		case 10000: // krece ka ms13
			cur_task = move_on_path (x_side (1100), -100, phi_side (0), FORWARD, 0, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10010;
			break;
		case 10010:
			cur_task = move_on_path (x_side (-1300), -600, phi_side (0), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10020;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 10500;
			break;
		case 10020:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10030;
			break;
		case 10030:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10040;
			break;
		case 10040:
			cur_task = move_to_xy (x_side (-1450), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10050;
			break;
		case 10050:

			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			prepare_front ();
			tact_fsm_case = 10100;
			break;
		case 10100: //ms14
			cur_task = move_on_path (x_side (-700), -550, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10110;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 12200;
			break;
		case 10110:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10120;
			break;

		case 10120:
			cur_task = move_to_xy (x_side (-700), -750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10130;
			break;
		case 10130:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 10140;
			break;
		case 10140: // Baner
			cur_task = move_to_xy (x_side (-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10150;
			break;
		case 10150:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10160;
			break;
		case 10160:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10170;
			break;
		case 10170:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10180;
			break;
		case 10180:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10190;
			break;
		case 10190:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10200;
			break;
		case 10200:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10210;
			break;
		case 10210: // Gradi na ca4/1
			cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10220;
			break;
		case 10220: // krece ka ms10
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10230;
			break;
		case 10230:
			cur_task = move_on_path (x_side (-650), 600, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10240;
			break;
		case 10240:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10245;
			break;
		case 10245:
			prepare_front ();
			tact_fsm_case = 10250;
			break;
		case 10250:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10260;
			break;
		case 10260:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 10270;
			break;
		case 10270: // krece ka ca1
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10280;
			break;
		case 10280:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10285;
			break;
		case 10285:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10290;
			break;
		case 10290: // gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10300;
			break;
		case 10300: // krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10310;
			break;
		case 10310:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10320;
			break;

		case 10500: // krece ka ms10
			cur_task = move_on_path (x_side (900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10510;
			break;
		case 10510:
			cur_task = move_on_path (x_side (-650), 600, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10520;
			break;
		case 10520:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10530;
			break;
		case 10530:
			prepare_front ();
			tact_fsm_case = 10540;
			break;
		case 10540:
			cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10550;
			break;
		case 10550:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			prepare_back ();
			tact_fsm_case = 10560;
			break;
		case 10560: //krece ka ms11
			cur_task = move_on_path (x_side (-1300), 350, phi_side (0), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10570;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 10800;
			break;
		case 10570:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10580;
			break;
		case 10580:
			cur_task = move_to_xy (x_side (-1400), 350, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10590;
			break;
		case 10600:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 10605;
			break;
		case 10605:
			cur_task = move_on_path (x_side (0), 200, phi_side (0), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10610;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 11200;
			break;
		case 10610: //krece ka ca6
			cur_task = move_on_path (x_side (1350), -100, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10620;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 11300;
			break;
		case 10620:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10630;
			break;
		case 10630:
			cur_task = task_sprat_3_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10640;
			break;
		case 10640: //krece ka ms14
			cur_task = move_on_path (x_side (-700), -550, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10650;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 11500;
			break;
		case 10650:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10655;
			break;
		case 10655:
			prepare_back ();
			tact_fsm_case = 10660;
			break;
		case 10660:
			cur_task = move_to_xy (x_side (-700), -800, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10665;
			break;
		case 10665:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 10670;
			break;
		case 10670: //Baner
			cur_task = move_to_xy (x_side (-700), -950, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10680;
			break;
		case 10680:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10690;
			break;
		case 10690:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10700;
			break;
		case 10700:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10710;
			break;

		case 10710:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10720;
			break;
		case 10720:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10730;
			break;
		case 10730:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10735;
			break;
		case 10735:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10750;
			break;
		case 10750: //Gradi na ca4/1
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10760;
			break;
		case 10760: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10770;
			break;
		case 10770:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10780;
			break;

		case 10800: //krece ka ms21
			cur_task = move_on_path (x_side (-650), 600, phi_side (0), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10810;
			break;
		case 10810:
			cur_task = move_on_path (x_side (1300), 350, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10820;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 11700;

			break;
		case 10820:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10830;
			break;
		case 10830:
			prepare_back ();
			tact_fsm_case = 10840;
			break;
		case 10840:
			cur_task = move_to_xy (x_side (1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10850;
			break;
		case 10850:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 10860;
			break;
		case 10860: //baner
			cur_task = move_to_xy (x_side (1450), 350, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10870;
			break;
		case 10870:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10880;
			break;
		case 10890:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10900;
			break;
		case 10900:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10910;
			break;
		case 10910:
			cur_task = move_on_dir (500, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10920;
			break;
		case 10920:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10930;
			break;
		case 10930:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 10940;
			break;
		case 10940: //gradi na ca6/1
			cur_task = task_sprat_3_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10950;
			break;
		case 10950: //krece ka ms14
			cur_task = move_on_path (x_side (0), 200, phi_side (180), FORWARD, 1, V_MAX_DEF, 0,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10960;
			break;
		case 10960:
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10970;
			break;
		case 10970: //krece ka ms14
			cur_task = move_on_path (x_side (-700), -550, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10980;
			break;
		case 10980:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 10990;
			break;
		case 11000:
			prepare_back ();
			tact_fsm_case = 11010;
			break;
		case 11010:
			cur_task = move_to_xy (x_side (-700), -800, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11020;
			break;
		case 11020:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 11030;
			break;
		case 11030:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11040;
			break;
		case 11040:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11050;
			break;
		case 11050: //gradi
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11060;
			break;
		case 11060: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11070;
			break;
		case 11070:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11080;
			break;

		case 11200: //krece ka ca4
			cur_task = move_on_path (x_side (-1300), 350, phi_side (0), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11210;
			break;
		case 11210:
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11220;
			break;
		case 11220:
			cur_task = move_on_path (x_side (-300), -800, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11230;
			break;
		case 11230:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11240;
			break;
		case 11240: //gradi na ca6
			cur_task = task_sprat_3_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11250;
			break;
		case 11250: //krece ka ms14
			cur_task = move_on_path (x_side (-700), -550, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11260;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 12100;
			break;
		case 11260:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11270;
			break;
		case 11280:
			prepare_back ();
			tact_fsm_case = 11290;
			break;
		case 11290:
			cur_task = move_to_xy (x_side (-700), -800, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11300;
			break;
		case 11300:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 11310;
			break;
		case 11310: //Baner
			cur_task = move_to_xy (x_side (-700), -950, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11320;
			break;
		case 11320:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11330;
			break;
		case 11330:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11340;
			break;
		case 11340:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11350;
			break;
		case 11350:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11360;
			break;
		case 11360:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11370;
			break;
		case 11370:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11375;
			break;
		case 11375:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11380;
			break;
		case 11380: //Gradi na ca4/1
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11390;
			break;
		case 11390: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11400;
			break;
		case 11400:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11410;
			break;

		case 11500:
			cur_task = move_on_path (x_side (1100), -100, phi_side (180), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11510;
			break;
		case 11510: //krece ka ms21
			cur_task = move_on_path (x_side (1300), 350, phi_side (0), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11520;
			break;
		case 11520:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11530;
			break;
		case 11530:
			prepare_back ();
			tact_fsm_case = 11540;
			break;
		case 11540:
			cur_task = move_to_xy (x_side (1400), 350, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11550;
			break;
		case 11550:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 11560;
			break;
		case 11560: //baner
			cur_task = move_to_xy (x_side (1450), 500, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11570;
			break;
		case 11570:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11580;
			break;
		case 11590:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11600;
			break;
		case 11600:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11610;
			break;
		case 11610:
			cur_task = move_on_dir (500, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11620;
			break;
		case 11620:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11630;
			break;
		case 11630:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11640;
			break;
		case 11640: //gradi na ca6/1
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11650;
			break;

		case 11700: //krece ka ms14
			cur_task = move_on_path (x_side (-650), 600, phi_side (0), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11710;
			break;
		case 11710:
			cur_task = move_on_path (x_side (-700), -550, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11715;
			break;
		case 11715:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11720;
			break;
		case 11720:
			prepare_back ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11730;
			break;
		case 11730:
			cur_task = move_to_xy (x_side (-700), -750, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11740;
			break;
		case 11740:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 11750;
			break;
		case 11750: //Baner
			cur_task = move_to_xy (x_side (-700), -950, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11760;
			break;
		case 11760:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11770;
			break;
		case 11770:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11780;
			break;
		case 11780:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11790;
			break;
		case 11790:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11800;
			break;
		case 11800:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11810;
			break;
		case 11810:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 11820;
			break;
		case 11820: //Gradi na ca4/1
			cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11830;
			break;
		case 11830: //krece ka ms12
			cur_task = move_on_path (x_side (-400), -200, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11840;
			if (cur_task == TASK_FAIL)
				tact_fsm_case = 12000;
			break;
		case 11840:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11850;
			break;
		case 11850:
			prepare_front ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11860;
			break;
		case 11860:
			cur_task = move_to_xy (x_side (-400), 0, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11870;
			break;
		case 11870:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 11880;
			break;
		case 11880: //krece ka ca1
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11890;
			break;
		case 11890:
			cur_task = move_on_path (x_side (-700), -800, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11900;
			break;
		case 11900:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11910;
			break;
		case 11910: //gradi
			cur_task = task_sprat_3_half (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11920;
			break;
		case 11920: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (-90), BACKWARD, 1, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11930;
			break;
		case 11930:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (-90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11940;
			break;

		case 12000:
			cur_task = move_on_path (x_side (-700), -800, phi_side (-90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12010;
			break;
		case 12010:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12020;
			break;
		case 12020: // gradi
			cur_task = task_sprat_1 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12030;
			break;

		case 12100:
			cur_task = move_on_path (x_side (-300), -700, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12110;
			break;

		case 12110:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12120;
			break;
		case 12120: //gradi
			cur_task = task_sprat_1 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12130;
			break;

		case 12200: //krece ka ms11
			cur_task = move_to_xy (x_side (-1400), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12210;
			break;
		case 12210: //krece ka ms11
			cur_task = move_on_path (x_side (-1300), 350, phi_side (180), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12220;
			break;
		case 12220:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12225;
			break;
		case 12225:
			prepare_front ();
			tact_fsm_case = 12230;
			break;
		case 12230:
			cur_task = move_to_xy (x_side (-1400), 350, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12240;
			break;
		case 12240:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 12250;
			break;
		case 12250: //krece ka ca6
			cur_task = move_on_path (x_side (100), -100, phi_side (180), BACKWARD, 0, V_MAX_DEF, 0,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12260;
			break;
		case 12260:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12270;
			break;
		case 12270: //gradi na ca6
			cur_task = task_sprat_3_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12280;
			break;

		case 12280: //krece ka ms14
			cur_task = move_on_path (x_side (-700), -550, phi_side (90), BACKWARD, 0, V_MAX_DEF, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12290;
			break;
		case 12290:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12300;
			break;
		case 12300:
			cur_task = move_to_xy (x_side (-700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12310;
			break;
		case 12310:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 12320;
			break;
		case 12320: //Baner
			cur_task = move_to_xy (x_side (-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12330;
			break;
		case 12330:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12340;
			break;
		case 12340:
			bnr_1 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 12350;
			break;
		case 12350:
			bnr_2 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 12360;
			break;
		case 12360:
			cur_task = move_on_dir (500, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12370;
			break;
		case 12370:
			bnr_3 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 12380;
			break;
		case 12380:
			bnr_4 ();
			if (delay_nb_2 (&tact_delay_1, 500))
				tact_fsm_case = 12390;
			break;
		case 12390: //Gradi na ca4/1
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12400;
			break;
		case 12400: //krece ka kraju
			cur_task = move_on_path (x_side (-900), 0, phi_side (90), FORWARD, 1, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12410;
			break;
		case 12410:
			cur_task = move_on_path (x_side (-1100), 800, phi_side (90), FORWARD, 0, V_MAX_DEF, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 12420;
			break;
		}
	return tact_state;
}

