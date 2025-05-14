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
tact_dev_3 ()
{
	switch (tact_fsm_case)
		{
		case 0:
			get_robot_base ()->x = 0;
			get_robot_base ()->phi = 90;
			get_robot_base ()->y = -840;
			tact_fsm_case = 30;
			break;

		case 10:
			cur_task = move_on_dir (2500, FORWARD, V_MAX_DEF, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				tact_fsm_case = 77;
			else if (cur_task == TASK_FAIL)
				tact_fsm_case = 99;
			break;

		case 20:
			cur_task = move_on_path (1000, 0, 0, FORWARD, 0, V_MAX_DEF_PATH, 0, NO_SENS);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 77;
				}
			else if (cur_task == TASK_FAIL)
				{
					tact_fsm_case = 99;
				}
			break;

		case 30:
			cur_task = move_to_xy (1000, 0, FORWARD, V_MAX_DEF, W_MAX_DEF * 0.5, FORWARD);
			if (cur_task == TASK_SUCCESS)
				{
					tact_fsm_case = 77;
				}
			else if (cur_task == TASK_FAIL)
				{
					tact_fsm_case = 99;
				}
			break;

		case -1:
			tact_state = TASK_SUCCESS;
			break;
		}
	return tact_state;
}
