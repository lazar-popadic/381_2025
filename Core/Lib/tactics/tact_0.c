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
		case 0: //krece prednjom straanom ka ms24
			prepare_front ();
			if (delay_nb_2 (&tact_delay_1, 500))
				{
					tact_fsm_case = 10;
				}
			break;

		case 10:
			cur_task = move_on_path (x_side (700), -550,/*TODO:ugao*/0, FORWARD, 0, V_MAX_DEF, /*TODO: ne moze max brzina da bude negativna*/
																W_MAX_DEF * (-0.25), FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;
		case 20:
			// TODO: nemoj da koristis rot_relative, a i ovde nema smisla da odmah posle rotacije hvata konzerve, vrv treba neki move_to_xy
			cur_task = rot_relative (90, W_MAX_DEF, FORWARD);
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
			cur_task = move_to_xy (x_side (700), -575, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 50;
			break;
		case 50: //krece ka ms23
			cur_task = move_to_xy (x_side (1150), -575, BACKWARD, V_MAX_DEF, W_MAX_DEF, BACKWARD);
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
			cur_task = move_on_path (x_side (1300), -100,/*TODO:ugao*/0, FORWARD, 0, V_MAX_DEF, W_MAX_DEF * (0.25), FORWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 80;
			break;
		}
	return tact_state;
}

