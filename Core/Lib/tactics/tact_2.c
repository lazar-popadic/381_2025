/*
 * tact_2.c
 *
 *  Created on: Mar 21, 2025
 *      Author: Lea mhm :-)
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
tact_2 ()
{
	switch (tact_fsm_case)
		{
		case -1:
			break;

		case 0: //krece ka ms24
			get_robot_base ()->x = x_side (1228);
			get_robot_base ()->phi = phi_side (-90);
			get_robot_base ()->y = -60;
			tact_fsm_case = 10;
			break;
		case 10: //kretanje ka buntu blize ostavljanju banera
			cur_task = move_to_xy (x_side (1150), -580, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
															NO_SENS);

			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;
		case 20:
			cur_task = rot_to_phi (phi_side (0), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 25;
			break;
		case 25:
			prepare_front ();
			tact_fsm_case = 30;
			break;
		case 30:
			cur_task = move_on_dir (180, FORWARD, 0.2, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (1500))
				tact_fsm_case = 40;
			break;
		case 40:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 45;
			break;
		case 45:
			cur_task = move_on_dir (40, BACKWARD, V_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF * 0.25, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 60;
			break;
		case 60:
			cur_task = move_on_dir (290, BACKWARD, 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				tact_fsm_case = 70;
			break;
		case 70: //baner
			lift_back_down_bnr ();
			grtl_back_open_outside ();
			if (delay_nb_2 (&tact_delay_1, 500))
				{
					tact_fsm_case = 72;
					add_points (20);
				}
			break;
		case 72:
			cur_task = move_on_dir (200, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 75;
			break;
		case 75:
			grtl_back_close ();
			lift_back_down ();
			tact_fsm_case = 80;
			break;
		case 80: //bunt kod protivnickih sima
			cur_task = move_to_xy (x_side (1120), 320, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 90;
			if (detected_timeout (1000))
				tact_fsm_case = 500;
			break;
		case 90:
			prepare_back ();
			tact_fsm_case = 100;
			break;
		case 100:
			cur_task = rot_to_phi (phi_side (180), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 110;
			break;
		case 110:
			cur_task = move_on_dir (220, BACKWARD, 0.4, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (2000))
				{
					tact_fsm_case = 120;
					reset_all_delays ();
				}
			break;

		case 120:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 125;
			break;
		case 125:
			cur_task = move_on_dir (200, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 130;
			break;
		case 130:
			cur_task = move_to_xy (x_side (1240), -140, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
															FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 135;
			break;
		case 135:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 140;
			break;
		case 140:
			cur_task = task_sprat_3_full (FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 145;
					add_points (28);
					ruc_front_carry ();
					lift_front_carry ();
				}
			break;
		case 145:
			cur_task = move_on_dir (240, FORWARD, V_MAX_DEF, FORWARD); //100
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 150;
			break;
		case 150:
			grtl_back_close ();
			tact_fsm_case = 170;
			break;
//		case 160: //bolji sto
//			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
//			if (cur_task == TASK_SUCCESS)
//				tact_fsm_case = 170;
//			break; //do ovde je sve ok

		case 170: //krece ka ms22// plav
			if (get_tact_num_ptr ()->side)
				cur_task = move_to_xy (x_side (-400), 300, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			else
				cur_task = move_to_xy (x_side (-390), 300, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 180;
			else if (detected_timeout (1000)) //koordinate ok ali zeza detekciju spor je kao da ne radi na 1s
				{
					if (fabs (get_robot_base ()->x) < 300.0)
						tact_fsm_case = 171;
					else
						{
							reset_movement ();
							reset_all_delays ();
						}
				}
			break;
//--- MINI ALTERNATIVNA
		case 171: //krece ka ms22
			if (get_tact_num_ptr ()->side)
				cur_task = move_to_xy (x_side (375), 300, BACKWARD, V_MAX_DEF * 0.8, W_MAX_DEF, BACKWARD);
			else
				cur_task = move_to_xy (x_side (380), 300, BACKWARD, V_MAX_DEF * 0.8, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 172;
			break;
		case 172:
			prepare_back ();
			tact_fsm_case = 173;
			break;
		case 173:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 174;
			break;
		case 174:
			cur_task = move_on_dir (475, BACKWARD, 0.4, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 175;
			break;
		case 175:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			if (check_back ())
				tact_fsm_case = 220;
			else
				{
					tact_fsm_case = 176;
					grtl_back_close ();
					ruc_back_up ();
					lift_back_down ();
					vacuum_back (0);
				}
			break;
		case 176:
			cur_task = move_on_dir (475, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1000;
			break;
//--- KRAJ MINI ALTERNATIVNE

		case 180:
			prepare_back ();
			tact_fsm_case = 190;
			break;
		case 190:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 200;
			break;
		case 200:
			cur_task = move_on_dir (540, BACKWARD, 0.4, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 210;
			break;
		case 210: // ovde bi jako valjalo da rade granicnici na pumpama jer ako ovde ne pokupi mozemo ga slati
			// u polje iza koje nam je garantovano
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			if (check_back ())
				tact_fsm_case = 220;
			else
				tact_fsm_case = 1000;
			break;
		case 220:
			cur_task = move_to_xy (x_side (-250), -350, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
															FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 230;
			break;
		case 230:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 235;
			break;
		case 235:
			cur_task = move_on_dir (325, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 240;
			break;
		case 240:
			cur_task = task_sprat_3_half (FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					add_points (28);
					tact_fsm_case = 245;
					grtl_front_close ();
					grtl_back_close ();
				}
			break;
		case 245:
			cur_task = move_to_xy (x_side (-710), -400, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
															BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 246;
			break;
		case 246:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 247;
			break;

		case 247:
			prepare_back ();
			tact_fsm_case = 248;
			break;
		case 248:
			cur_task = move_on_dir (350, BACKWARD, V_MAX_DEF * 0.3, NO_SENS);
			if (cur_task == TASK_SUCCESS || timeout (1500))
				{
					add_points (4);
					tact_fsm_case = 249;
				}
			break;
		case 249:
			cur_task = move_on_dir (100, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 250;
			break;
		case 250:
			cur_task = move_to_xy (x_side (-600), 100, FORWARD, V_MAX_DEF,
			W_MAX_DEF,
															FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 260;
			break;
		case 260:
			cur_task = rot_to_xy (x_side (-800), 500, FORWARD, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 9999;
			break;

// --- ALTERNATIVNA 1
		case 500:
			cur_task = move_to_xy (x_side (735), -450, BACKWARD, V_MAX_DEF * 0.5,
			W_MAX_DEF,
															BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 510;
			break;
		case 510:
			cur_task = rot_to_phi (phi_side (90), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 520;
			break;
		case 520:
			cur_task = move_to_xy (x_side (735), -700, BACKWARD, 0.2, W_MAX_DEF,
			NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 530;
			break;
		case 530:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			//prepare_back ();
			tact_fsm_case = 540;
			break;
		case 540:
			cur_task = move_on_path (x_side (-100), -300, phi_side (180), FORWARD, //-680
																0, 0.5, 0, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 550;
			break;

//--- ALTERNATIVNA 1, KAD NEMA SREDNJI BUNT IDE PO REZERVISANI
		case 1000:
			grtl_back_close ();
			ruc_back_up ();
			lift_back_down ();
			vacuum_back (0);
			tact_fsm_case = 1005;
		case 1005:
			cur_task = move_to_xy (x_side (-645), 370, BACKWARD, V_MAX_DEF,
			W_MAX_DEF,
															BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1010;
			break;

		case 1010:
			cur_task = rot_to_phi (phi_side (-90), W_MAX_DEF * 0.5, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 1020;
			break;

		case 1020:
			prepare_back ();
			tact_fsm_case = 1030;
			break;

		case 1030:
			cur_task = move_on_dir (290, BACKWARD, 0.3, NO_SENS);

			if (cur_task == TASK_SUCCESS || timeout (1500))
				tact_fsm_case = 1040;
			break;

		case 1040:
			grtl_back_grip_all ();
			ruc_back_carry ();
			lift_back_carry ();
			vacuum_back (1);
			tact_fsm_case = 1050;
			break;

		case 1050:
			cur_task = move_on_dir (250, FORWARD, V_MAX_DEF, FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 220;
			break;

		}
	return tact_state;
}
