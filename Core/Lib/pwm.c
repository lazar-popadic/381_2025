/*
 * pwm.c
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#include "../Inc/main.h"
#include "../Inc/tim.h"

static uint16_t
to_arr (uint16_t us);
static uint16_t
angleToSG90 (float angle);

void
pwm_left_dc (int16_t duty_cycle)
{
	TIM4->CCR3 = duty_cycle;
}

void
pwm_right_dc (int16_t duty_cycle)
{
	TIM4->CCR1 = duty_cycle;
}

void
pwm_start ()
{
	HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_3);
}

void
pwm_stop ()
{
	HAL_TIM_PWM_Stop (&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop (&htim4, TIM_CHANNEL_3);
}

/*	SG90
 *
 *	Period	=	 20 ms
 *	1	ms 		= 	0	deg
 *	2	ms 		= 180	deg
 *	---
 *	ARR			= 1000 - 1
 *	50  ARR	=		0	deg
 *	100 ARR	=	180	deg
 *	---
 *	1000 ARR	==	20 000 us
 *	1 ARR			==	20 us
 */
void
sg90_init ()
{
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_4);
	bnr_close ();
	// cisto da ne izlazi warning
	(void) angleToSG90 (0);
}

static uint16_t
angleToSG90 (float angle)
{
	// Only works with standard sg90, limited to 180deg
	return angle * 50 / 180 + 50;
}

static uint16_t
to_arr (uint16_t us)
{
	return (uint16_t) us * 0.05;
}

void
sg90_1_move (float us)
{
	TIM2->CCR3 = to_arr (us);
}

void
sg90_2_move (float us)
{
	TIM2->CCR4 = to_arr (us);
}

void
sg90_3_move (float us)
{
	TIM2->CCR1 = to_arr (us);
}

void
sg90_4_move (float us)
{
	TIM2->CCR2 = to_arr (us);
}
