/*
 * tact_dev.c
 *
 *  Created on: Mar 21, 2025
 *      Author: lazar
 */

#include "main.h"

void
test_tact ()
{
	lift_back_down ();
	HAL_Delay (1000);
	vacuum_back (1);
	HAL_Delay (10);
	vacuum_front (1);
	HAL_Delay (10);
	lift_front_down();
	HAL_Delay (10);
	prepare_back ();
	HAL_Delay (1000);
	prepare_front ();
}
