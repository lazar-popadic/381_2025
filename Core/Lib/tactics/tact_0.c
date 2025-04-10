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
		prepare_front ();
		if (delay_nb_2 (&tact_delay_1, 500))
		{
			tact_fsm_case = 10;
		}
		break;

	case 10:
		cur_task = move_on_path (x_side (700), -550, -90, FORWARD, 0, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 20;
		break;
	case 20:
		cur_task = move_to_xy (x_side (700), -800, FORWARD, 0, V_MAX_DEF, W_MAX_DEF, FORWARD);
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
		cur_task = move_to_xy (x_side (700), -600, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 50;
		break;
	case 50: //krece ka ms23
		cur_task = move_to_xy (x_side (1400), -575, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
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
		cur_task = move_on_path (x_side (1350), -100, 0, FORWARD, 0, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 80;
		break;
	case 80://gradi na ca6
		cur_task = task_sprat_3_1_full(FORWARD);
		tact_fsm_case = 85;
		break;
	case 85:
		prepare_back ();
		tact_fsm_case = 90;
		break;
	case 90://krece prednjom stranom ka ms22
		cur_task = move_on_path (x_side (400), 200, 90, BACKWARD, 0, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 100;
		break;
	case 100:
		cur_task = move_to_xy (x_side (400), -100, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
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
	case 120://krece prednjom stranom ka ms14
		cur_task = move_on_path (x_side (-700), -550, 90, BACKWARD, 0, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 130;
		break;
	case 130:
		cur_task = rot_relative (-90, W_MAX_DEF, ALL_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 140;
		break;
	case 140:
		cur_task = move_to_xy (x_side (-700), -800, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
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
	case 160://Baner
		cur_task = move_to_xy (x_side (-700), -950, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 170;
		break;
	case 170:
		cur_task = rot_relative (180, W_MAX_DEF, ALL_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 180;
		break;
	case 180:
		if (delay_nb_2 (&tact_delay_1, 1000))
		{
			tact_fsm_case = 190;
		}
		break;
	case 190:
		cur_task = move_to_xy (x_side (-200), -950, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 200;
		break;
	case 200:
		if (delay_nb_2 (&tact_delay_1, 1000))
		{
			tact_fsm_case = 210;
		}
		break;
	case 210://Gradi na ca4/1
		cur_task = task_sprat_3(BACKWARD);
		tact_fsm_case = 220;
		break;

	case 220://krece prednjom stranom ka ms10
		cur_task = move_on_path (x_side(-900), 0, -90, BACKWARD, 1, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 230;
		break;
	case 230:
		cur_task = move_on_path (x_side(-650), 600, -90, BACKWARD, 0, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 240;
		break;
	case 240:
		cur_task = rot_relative (90, W_MAX_DEF, ALL_SENS);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 245;
		break;
	case 245:
		prepare_front();
		tact_fsm_case = 250;
		break;
	case 250:
		cur_task = move_to_xy (x_side (-650), 750, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD);
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
	case 270://krece zadnjom stranom ka ca1
		cur_task = move_on_path (x_side(-900), 0, 90, BACKWARD, 1, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 280;
		break;
	case 280:
		cur_task = move_on_path (x_side(-700), -800, 90, BACKWARD, 0, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 290;
		break;
	case 290://gradi
		cur_task = task_sprat_3(BACKWARD);
		tact_fsm_case = 310;
		break;
	case 310://krece ka kraju
		cur_task = move_on_path (x_side(-900), 0, -90, BACKWARD, 1, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 320;
		break;
	case 320:
		cur_task = move_on_path (x_side(-1100), 800, -90, BACKWARD, 0, V_MAX_DEF, W_MAX_DEF, BACKWARD);
		if (cur_task == TASK_SUCCESS)
			tact_fsm_case = 330;
		break;
	}
	return tact_state;
}

