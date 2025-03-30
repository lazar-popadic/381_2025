/*
 * tact_dev.c
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
			tact_fsm_case = 1;
			break;

		case 1:
			cur_task = move_to_xy (x_side (-800), 0, FORWARD, 1.0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 2;
				}
//			else if (cur_task == TASK_FAIL)
//				{
//					tact_fsm_case = 2;
//				}
			break;

		case 2:
			cur_task = move_to_xy (x_side (-800), -400, BACKWARD, 1.0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 3;
				}
//			else if (cur_task == TASK_FAIL)
//				{
//					tact_fsm_case = 3;
//		}
			break;

		case 3:
			cur_task = move_to_xy (0, -400, FORWARD, 1.0, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 4;
				}
//			else if (cur_task == TASK_FAIL)
//				{
//					tact_fsm_case = 4;
//				}
			break;

		case 4:
			cur_task = move_to_xy (0, 0, BACKWARD, 0.5, W_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = -1;
				}
//			else if (cur_task == TASK_FAIL)
//				{
//					tact_fsm_case = -1;
//				}
			break;

		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
