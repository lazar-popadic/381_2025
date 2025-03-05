/*
 * pwm.c
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#include "../Inc/main.h"
#include "../Inc/tim.h"

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
 */
void
sg90_init()
{
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_4);
}

uint16_t
angleToSG90 (float angle)
{
	return angle * 50 / 180 + 50;
}

void
sg90_1_move (float angle)
{
	TIM2->CCR3 = angleToSG90(angle);
}

void
sg90_2_move (float angle)
{
	TIM2->CCR4 = angleToSG90(angle);
}

void
sg90_3_move (float angle)
{
	TIM2->CCR1 = angleToSG90(angle);
}

void
sg90_4_move (float angle)
{
	TIM2->CCR2 = angleToSG90(angle);
}
