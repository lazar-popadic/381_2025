/*
 * tact_dev_2.c
 *
 *		Taktika za razvoj i testiranje taskova.
 *		Trenutno:
 *			-	kvadrat
 *
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

extern int16_t tact_fsm_case;
extern uint32_t tact_delay_1;
extern uint8_t points;
extern int8_t tact_state;
static int8_t cur_task;

int8_t
tact_dev_2 ()
{
	switch (tact_fsm_case)
		{
		case 0:
			prepare_front ();
			tact_fsm_case = 10;
			break;

		case 10:
			cur_task = move_to_xy (200, 0, FORWARD, V_MAX_DEF * 0.5, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 30;
			break;

		case 30:
			grtl_front_grip_all ();
			ruc_front_carry ();
			lift_front_carry ();
			vacuum_front (1);
			prepare_back ();
			tact_fsm_case = 50;
			break;

			/*
			 *	Hvatanje MS 2
			 *	zadnjom stranom
			 */

		case 50:
			cur_task = move_to_xy (-200, 0, BACKWARD, 0.5, W_MAX_DEF, NO_SENS);
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

			/*
			 *	Idi do CA 1
			 */
		case 70:
			//			cur_task = move_on_path (200, -200, 0, FORWARD, 0, V_MAX_DEF, 0, NO_SENS);
			cur_task = move_to_xy (-400, 0, BACKWARD, V_MAX_DEF, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 80;
			break;

			/*
			 *	Izgradi 3 sprata i ostavi preostali 1 sprat
			 */
		case 80:
//			cur_task = task_sprat_3_half (BACKWARD);
//			cur_task = task_sprat_3_full (BACKWARD);
			cur_task = task_sprat_3_2_full (BACKWARD);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = -1;
			break;

		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
