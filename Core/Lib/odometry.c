/*
 * odometry.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "main.h"
#include "tim.h"

float L_wheel;
float L_wheel_recip;
float d_odom_left;
float d_odom_right;
float inc2mm_left;
float inc2mm_right;
float rad2deg;
float deg2rad;
float mid_angle;
int16_t v_l_diff = 0;
int16_t v_r_diff = 0;

volatile int16_t encoder_sum_left;		// [inc]
volatile int16_t encoder_sum_right;		// [inc]
static volatile robot_base *base_ptr;

void
odometry_init ()
{
  L_wheel = 341;									// [mm], rastojanje izmedju odometrijskih tockova
  L_wheel_recip = 1000 / L_wheel;					// [1/m]
  d_odom_left = 72;									// [mm] TODO: L_wheel i d_odom namesti, 381_2k24: L = 342.8, d = 75.8
  d_odom_right = 72;								// [mm]
  inc2mm_left = d_odom_left * M_PI / (4 * 2048);	// [mm/inc]
  inc2mm_right = d_odom_right * M_PI / (4 * 2048);	// [mm/inc]
  rad2deg = 180 / M_PI;								// [deg/rad]
  deg2rad = M_PI / 180;								// [rad/deg]
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  HAL_TIM_Encoder_Start (&htim5, TIM_CHANNEL_ALL);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  HAL_TIM_Encoder_Start (&htim3, TIM_CHANNEL_ALL);
  base_ptr = get_robot_base ();
  encoder_sum_left = 0;
  encoder_sum_right = 0;
}

void
update_odom ()
{
  v_r_diff = (int16_t) TIM3->CNT - encoder_sum_right;				// [inc/ms]
  base_ptr->v_right = (v_r_diff) * inc2mm_right;					// [mm/ms = m/s]
  v_l_diff = (int16_t) TIM5->CNT - encoder_sum_left;				// [inc/ms]
  base_ptr->v_left = (v_l_diff) * inc2mm_left;						// [mm/ms = m/s]
  encoder_sum_right = (int16_t) TIM3->CNT;							// [inc]
  encoder_sum_left = (int16_t) TIM5->CNT;							// [inc]

  base_ptr->v_prev = base_ptr->v;															// take the previous values of v and w
  base_ptr->w_prev = base_ptr->w;
  base_ptr->v = (base_ptr->v_right + base_ptr->v_left) * 0.5;								// [mms/ms = m/s]
  base_ptr->w = (base_ptr->v_right - base_ptr->v_left) * L_wheel_recip * rad2deg;			// [m/s * 1/m * deg/rad = deg/s]
  mid_angle = (base_ptr->phi + base_ptr->w * 0.0005) * deg2rad;								// [(deg + 0.5*deg/s*1ms) * rad/deg = rad]

  base_ptr->x += base_ptr->v * cos (mid_angle);		// [mm/ms * 1ms = mm]
  base_ptr->y += base_ptr->v * sin (mid_angle);		// [mm/ms * 1ms = mm]
  base_ptr->phi += 0.001 * base_ptr->w;				// [1ms * deg/s = 0.001s * deg/s = deg]
  base_ptr->phi = wrap180 (base_ptr->phi);

  base_ptr->a = (base_ptr->v - base_ptr->v_prev)*1000;
  base_ptr->alpha = (base_ptr->w - base_ptr->w_prev)*1000;
}
