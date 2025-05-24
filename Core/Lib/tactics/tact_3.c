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
tact_3 ()
{
	switch (tact_fsm_case)
		{
		case -1:
			break;
		case 0: //krece ka ms24
			get_robot_base ()->x = x_side (-322);
			get_robot_base ()->phi = phi_side (90);
			get_robot_base ()->y = -710;
			tact_fsm_case = 2;
			break;
		case 2:
			cur_task = move_on_dir (155, BACKWARD, 0.5, NO_SENS);
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
			cur_task = move_on_dir (100, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 11;
			break;
		case 11:
			lift_back_down ();
			grtl_back_close ();
			tact_fsm_case = 12;
			break;
		case 12: //srednji bunt blizi njemu
			if (get_tact_num_ptr ()->side) //plavu
				cur_task = move_on_path (x_side (-420), -400, phi_side (120), FORWARD, 0, V_MAX_DEF_PATH, 0, FORWARD);
			else
				//zuta
				cur_task = move_on_path (x_side (-400), -400, phi_side (120), FORWARD, 0, V_MAX_DEF_PATH, 0, FORWARD);
			if (cur_task == TASK_SUCCESS) // TODO: ako nema idi u alt
				tact_fsm_case = 13;
			break;
		case 13:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 15;
			break;
		case 15:
			prepare_front ();
			tact_delay_1 = 0xFFFF;
			tact_fsm_case = 20;
			break;
		case 20: //pred uzimanje bunta
			cur_task = move_on_dir_ortho (400, FORWARD, 0.2, FORWARD); //300
			if (delay_nb_2 (&tact_delay_1, 1500))
				grtl_front_grip_all ();
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
		case 40: //srednji bunt dalji njemu
			if (get_tact_num_ptr ()->side) //plavu
				cur_task = move_on_path (x_side (380), 240, phi_side (0), FORWARD, 0,
				V_MAX_DEF_PATH,
																	0, FORWARD);
			else
				cur_task = move_on_path (x_side (420), 240, phi_side (0), FORWARD, 0,
				V_MAX_DEF_PATH,
																	0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 60;
			break;
		case 60:
			prepare_back ();
			tact_delay_1 = 0xFFFF;
			tact_fsm_case = 70;
			break;
		case 70: //pred uzimanje bunta
			cur_task = move_on_dir_ortho (400, BACKWARD, 0.2, BACKWARD); //300
			if (delay_nb_2 (&tact_delay_1, 1500))
				grtl_back_grip_all ();
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 80;
			break;
		case 80:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 90;
			break;
		case 90:
			cur_task = move_to_xy (x_side (1200), -125, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
															FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 95;
			break;
		case 95:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 100;
			break;
		case 100:
			cur_task = task_sprat_3_1_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 110;
					add_points (32);
					grtl_front_close ();
				}
			break;
		case 110:
			cur_task = move_to_xy (x_side (730), -450, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			//cur_task = move_on_path(x_side(700), -500, phi_side(90), BACKWARD, 0, V_MAX_DEF_PATH, 0, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 115;
			break;
		case 115:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 120;
			break;
		case 120:
			prepare_back ();
			tact_fsm_case = 130;
			break;
		case 130:
			cur_task = move_on_dir_ortho (190, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;
		case 140:
			ruc_back_full_down ();
			if (check_back ())
				tact_fsm_case = 141;
			if (timeout (1000))
				tact_fsm_case = 500;
			break;
		case 141:
			ruc_back_mid ();
			tact_fsm_case = 142;
			break;
		case 142:
			cur_task = move_on_dir_ortho (300, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (1500))
				tact_fsm_case = 143;
			break;
		case 143:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 145;
			break;
		case 145:
			cur_task = move_on_dir (100, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 150;
			break;
		case 150: //ms23
			cur_task = move_to_xy (x_side (1100), -620, FORWARD,
			V_MAX_DEF,
															W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 155;
			break;
		case 155:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 160;
			break;
		case 160:
			prepare_front ();
			tact_fsm_case = 170;
			break;
		case 170:
			cur_task = move_on_dir_ortho (300, FORWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (1500))
				tact_fsm_case = 180;
			break;
		case 180:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			if (check_front ())
				tact_fsm_case = 190;
			if (timeout (1000))
				tact_fsm_case = 600;
			break;
		case 190:
			cur_task = move_on_dir (400, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 195;
			break;
		case 195:
			cur_task = move_to_xy (x_side (830), -125, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 200;
			break;
		case 200:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 210;
			break;
		case 210:
			cur_task = task_sprat_12 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 220;
			break;
		case 220:
			HAL_Delay (200);
			tact_fsm_case = 230;
			break;
		case 230:
			cur_task = task_sprat_3 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 235;
					add_points (24);
				}
			break;
		case 235:
			grtl_front_close ();
			tact_fsm_case = 240;
			break;
		case 240:
			//if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy (x_side (-715), -400, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
															BACKWARD);
			//else
			//zuta
			//cur_task = move_to_xy(x_side(-715), -400, BACKWARD, V_MAX_DEF,
			//W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 250;
			break;
		case 250:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 255;
			break;
		case 255:
			prepare_front ();
			tact_fsm_case = 256;
			break;
		case 256:
			cur_task = move_on_dir_ortho (210, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 257;
			break;
		case 257:
			ruc_front_full_down ();
			if (check_front ())
				tact_fsm_case = 258;
			if (timeout (1000))
				tact_fsm_case = 700;
			break;
		case 258:
			ruc_front_mid ();
			tact_fsm_case = 260;
			break;
			// gura bunt u manje polje
		case 260:
			cur_task = move_on_dir_ortho (240, FORWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				tact_fsm_case = 261;
			break;
		case 261:
			cur_task = move_on_dir_ortho (260, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 262;
			break;
		case 262:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 263;
			break;
		case 263:
			cur_task = task_sprat_12 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 264;
			break;
		case 264:
			HAL_Delay (200);
			tact_fsm_case = 266;
			break;
		case 266:
			cur_task = task_sprat_3 (BACKWARD);
			if (cur_task == TASK_SUCCESS || timeout (10000))
				{
					tact_fsm_case = 270;
					add_points (28);
				}
			break;
		case 270:
			cur_task = move_to_xy (x_side (-800), -100, FORWARD, V_MAX_DEF, W_MAX_DEF,
			FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 280;
			break;
		case 280:
			cur_task = rot_to_xy (x_side (-1000), 500, FORWARD, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 9999;
			break;

			//ALTERNATIVNA nema bunta pored neprijateljskog manjeg polja
		case 500:
			ruc_back_mid ();
			tact_fsm_case = 502;
			break;
		case 502:
			cur_task = move_on_dir (100, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 505;
			break;
		case 505:
			cur_task = move_to_xy (x_side (1100), -620, FORWARD,
			V_MAX_DEF,
															W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 510;
			break;
		case 510:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 515;
			break;
		case 515:
			prepare_front ();
			tact_fsm_case = 520;
			break;
		case 520:
			cur_task = move_on_dir_ortho (300, FORWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (1500))
				tact_fsm_case = 525;
			break;
		case 525:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			if (check_front ())
				tact_fsm_case = 530;
			if (timeout (1000))
				tact_fsm_case = 650;
			break;
		case 530:
			cur_task = move_on_dir (400, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 535;
			break;
		case 535:
			cur_task = move_to_xy (x_side (830), -125, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 540;
			break;
		case 540:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 545;
			break;
		case 545:
			cur_task = task_sprat_12 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 550;
			break;
		case 550:
			HAL_Delay (200);
			tact_fsm_case = 555;
			break;
		case 555:
			cur_task = task_sprat_3 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 560;
					add_points (24);
				}
			break;
		case 560:
			grtl_front_close ();
			grtl_back_close ();
			tact_fsm_case = 565;
			break;
		case 565:
			//if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy (x_side (-715), -400, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
															BACKWARD);
			//else
			//zuta
			//cur_task = move_to_xy(x_side(-715), -400, BACKWARD, V_MAX_DEF,
			//W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 570;
			break;
		case 570:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 575;
			break;
		case 575:
			prepare_front ();
			tact_fsm_case = 580;
			break;
		case 580:
			cur_task = move_on_dir_ortho (210, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 590;
			break;
		case 590:
			ruc_front_full_down ();
			if (check_front ())
				tact_fsm_case = 591;
			if (timeout (1000))
				tact_fsm_case = 650;
		case 591:
			ruc_front_mid ();
			tact_fsm_case = 592;
			break;
			// gura bunt u manje polje
		case 592:
			cur_task = move_on_dir_ortho (200, FORWARD, 0.4, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				tact_fsm_case = 593;
			break;
		case 593:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 594;
			break;
		case 594:
			cur_task = task_sprat_12 (FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 595;
					add_points (12);
				}
			break;
		case 595:
			cur_task = move_on_dir_ortho (235, BACKWARD, 0.4, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				tact_fsm_case = 270;
			break;

			// ALTERNATIVNA nema bunta kod ivice stola samo
		case 600:
			grtl_front_open ();
			vacuum_front (0);
			ruc_front_up ();
			lift_front_down ();
			tact_fsm_case = 602;
			break;
		case 602:
			cur_task = move_on_dir (400, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 605;
			break;
		case 605:
			cur_task = move_to_xy (x_side (830), -125, BACKWARD, V_MAX_DEF, W_MAX_DEF,
			BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 610;
			break;
		case 610:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 615;
			break;
		case 615:
			cur_task = task_sprat_12 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 620;
			break;
		case 620:
			HAL_Delay (200);
			tact_fsm_case = 625;
			break;
		case 625:
			cur_task = task_sprat_3 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 560;
					add_points (24);
				}
			break;
			// ALTERNATIVNA nema bunta kod ivice stola ni bunta kod protivnickog polja
		case 650:
			vacuum_front (0);
			ruc_front_mid ();
			grtl_front_open ();
			lift_front_down ();
			tact_fsm_case = 651;
			break;
		case 651:
			cur_task = move_on_dir_ortho (60, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 653;
			break;
		case 653:
			grtl_front_close ();
			tact_fsm_case = 655;
			break;
		case 655:
			if (get_tact_num_ptr ()->side) //plavu
				cur_task = move_to_xy (x_side (-730), 330, BACKWARD, V_MAX_DEF,
				W_MAX_DEF,
																BACKWARD);
			else
				cur_task = move_to_xy (x_side (-670), 330, BACKWARD, V_MAX_DEF,
				W_MAX_DEF,
																BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 658;
			break;
		case 658:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 660;
			break;
		case 660:
			prepare_back ();
			tact_fsm_case = 670;
			break;

		case 670:
			cur_task = move_on_dir_ortho (280, BACKWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				tact_fsm_case = 680;
			break;
		case 680:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 690;
			break;
		case 690:
			//if (get_tact_num_ptr()->side)	// plava
			cur_task = move_to_xy (x_side (-715), -400, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
															FORWARD);
			//else
			//zuta
			//cur_task = move_to_xy(x_side(-715), -400, BACKWARD, V_MAX_DEF,
			//W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 255;
			break;
			// ALTERNATIVNA nema poslednjeg bunta

		case 700:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 710;
			break;
		case 710:
			cur_task = move_on_dir_ortho (200, BACKWARD, 0.4, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				tact_fsm_case = 720;
			break;
		case 720:
			cur_task = task_sprat_12 (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 270;
					add_points (12);
				}
			break;

		}
	return tact_state;
}
