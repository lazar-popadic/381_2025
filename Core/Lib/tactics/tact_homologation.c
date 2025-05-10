/*
 * tact_dev.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 *
 *	Description:
 *			Homologation tactic.
 */

#include "main.h"

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;

int8_t
tact_homologation ()
{
	switch (tact_fsm_case)
		{
		case 0: //krece ka ms24
			get_robot_base ()->x = x_side (-1078);
			get_robot_base ()->phi = phi_side (-90);
			get_robot_base ()->y = 710;
			tact_fsm_case = 10;
			break;

		case 10:
			if (move_to_xy (x_side (-725), -400, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD) == TASK_SUCCESS)
				tact_fsm_case = 20;
			break;

		case 20:
			if (rot_to_phi (phi_side (-90), W_MAX_DEF, NO_SENS))
				tact_fsm_case = 25;
			break;

		case 25:
			prepare_front ();
			HAL_Delay (500);
			tact_fsm_case = 30;
			break;

		case 30:
			if (move_on_dir (400, FORWARD, 0.2, FORWARD) || timeout (3000))
				tact_fsm_case = 35;
			break;

		case 35:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			tact_fsm_case = 40;

			break;

		case 40:
			if (move_on_dir (50, BACKWARD, V_MAX_DEF, BACKWARD))
				tact_fsm_case = 50;
			break;

		case 50:
			if (task_sprat_12 (FORWARD))
				tact_fsm_case = 60;
			break;

		case 60:
			if (move_on_dir (200, BACKWARD, V_MAX_DEF, BACKWARD))
				tact_fsm_case = 70;
			break;

		case 70:
			if (rot_to_xy (x_side (-900), 100, FORWARD, W_MAX_DEF, NO_SENS))
				tact_fsm_case = 80;
			break;

		case 80:
			if (move_to_xy (x_side (-900), 100, FORWARD, V_MAX_DEF, W_MAX_DEF, FORWARD))
				tact_fsm_case = 9999;
			break;
		}
	return tact_state;
}
