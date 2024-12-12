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
