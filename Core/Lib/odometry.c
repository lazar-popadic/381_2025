/*
 * odometry.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "../Inc/main.h"
#include "../Inc/tim.h"

float L_wheel;
float L_wheel_recip;
float d_odom_left;
float d_odom_right;
float inc2mm_left;
float inc2mm_right;
float rad2deg;
float deg2rad;
float mid_angle;

void
odometry_init ()
{
  L_wheel = 341;					// [mm], rastojanje izmedju odometrijskih tockova
  L_wheel_recip = 1000 / L_wheel;			// [1/m]
  d_odom_left = 72;					// [mm] TODO: L_wheel i d_odom namesti, 381_2k24: L = 342.8, d = 75.8
  d_odom_right = 72;					// [mm]
  inc2mm_left = d_odom_left * M_PI / (4 * 2048);	// [mm/inc]
  inc2mm_right = d_odom_right * M_PI / (4 * 2048);	// [mm/inc]
  rad2deg = 180 / M_PI;					// [deg/rad]
  deg2rad = M_PI / 180;					// [rad/deg]
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void
update_odom (volatile struct_robot_base *base)
{
  base->v_right = (TIM3->CNT - base->encoder_sum_right) * inc2mm_right;	// [mm/ms = m/s]
  base->v_left = (TIM5->CNT - base->encoder_sum_left) * inc2mm_left;	// [mm/ms = m/s]
  base->encoder_sum_right = TIM3->CNT;					// [inc]
  base->encoder_sum_left = TIM5->CNT;					// [inc]

  base->v = (base->v_right - base->v_left) * 0.5;			// [mms/ms = m/s]
  base->w = (base->v_right - base->v_left) * L_wheel_recip * rad2deg;	// [m/s * 1/m * deg/rad = deg/s]
  mid_angle = (base->phi + base->w * 0.005) * deg2rad;			// [(deg + 0.5*deg/s*1ms) * rad/deg = rad]

  base->x += base->v * cos (mid_angle);					// [mm/ms * 1ms = mm]
  base->y += base->v * sin (mid_angle);					// [mm/ms * 1ms = mm]
  base->phi += 0.001 * base->w;						// [1ms * deg/s = 0.001s * deg/s = deg]
  base->phi = wrapTo180 (base->phi);
}
