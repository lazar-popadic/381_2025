/*
 * base.c
 *
 *  Created on: Dec 4, 2024
 *      Author: lazar
 */

#include "main.h"

volatile robot_base *base_ptr;
static volatile robot_base base;

void
base_init ()
{
	base_ptr = &base;
	// position, velocity, acceleration of base
	base_ptr->x = 0;						// [mm]
	base_ptr->y = 0;						// [mm]
	base_ptr->phi = 0;					// [deg]
	base_ptr->v = 0;						// [mm/ms = m/s]
	base_ptr->w = 0;						// [deg/s]
	base_ptr->a = 0;						// [mm/ms * 1/ms]
	base_ptr->alpha = 0;					// [deg/s * 1/ms]
	base_ptr->v_left = 0;					// [mm/ms = m/s]
	base_ptr->v_right = 0;				// [mm/ms = m/s]
	base_ptr->v_prev = 0;					// [mm/ms = m/s]
	base_ptr->w_prev = 0;					// [deg/s]

	// references for position, velocity of base
	base_ptr->v_ref = 0;					// [mm/ms = m/s]
	base_ptr->w_ref = 0;					// [deg/s]
	base_ptr->x_ref = 0;					// [mm]
	base_ptr->y_ref = 0;					// [mm]
	base_ptr->phi_ref = 0;				// [deg]

	// limits for velocity of base
	base_ptr->v_max = V_MAX_DEF;					// [mm/ms = m/s]
	base_ptr->w_max = W_MAX_DEF;					// [deg/s]
	base_ptr->a_max = A_MAX_DEF;					// [mm/ms * 1/ms]
	base_ptr->alpha_max = ALPHA_MAX_DEF;			// [deg/s * 1/ms]
	base_ptr->j_max = J_MAX_DEF;					// [mm/ms * 1/ms^2]
	base_ptr->j_rot_max = J_ROT_MAX_DEF;			// [deg/s * 1/ms^2]

	// control signals
	base_ptr->v_ctrl = 0;
	base_ptr->w_ctrl = 0;
	base_ptr->motor_l_ctrl = 0;
	base_ptr->motor_r_ctrl = 0;
	base_ptr->motor_l_dir = 0;
	base_ptr->motor_r_dir = 0;
	base_ptr->motor_l_ctrl_uint = 0;
	base_ptr->motor_r_ctrl_uint = 0;

	// status
	base_ptr->moving = 0;			// statusi po poziciji i brzini baze
	base_ptr->on_target = 0;
	base_ptr->movement_started = 0;	// statusi za same taskove kretnje
	base_ptr->movement_finished = 0;
}

volatile robot_base*
get_robot_base ()
{
	return base_ptr;
}

void
set_v_max (float multiplier)
{
	saturation (&multiplier, 1, 0);
	base_ptr->v_max = V_MAX_DEF * multiplier;
}

void
reset_v_max ()
{
	base_ptr->v_max = V_MAX_DEF;
}

void
set_w_max (float multiplier)
{
	saturation (&multiplier, 1, 0);
	base_ptr->w_max = W_MAX_DEF * multiplier;
}

void
reset_w_max ()
{
	base_ptr->w_max = W_MAX_DEF;
}

void
update_base_status ()
{
	if (base_ptr->v < V_MOVING_MIN && base_ptr->w < W_MOVING_MIN)
		base_ptr->moving = 0;
	else
		base_ptr->moving = 1;
}

uint8_t
check_sensors ()
{
	switch (base_ptr->obstacle_dir)
		{
		default:
			base_ptr->obstacle_detected = 0;
			break;
		case FORWARD:
			base_ptr->obstacle_detected = read_sensors_front ();
			break;
		case BACKWARD:
			base_ptr->obstacle_detected = read_sensors_back ();
			break;
		case ALL_SENS:
			base_ptr->obstacle_detected = read_sensors_back () || read_sensors_front ();
			break;
		}
	return base_ptr->obstacle_detected;
}

uint8_t
get_regulation_status ()
{
	return base_ptr->regulation_status;
}

void
set_regulation_status (uint8_t status)
{
	base_ptr->regulation_status = status;
}

uint8_t
get_obstacle_detected ()
{
	return base_ptr->obstacle_detected;
}

void
motors_off ()
{
	base_ptr->v_ref = 0;
	base_ptr->w_ref = 0;
	base_ptr->x_ref = base_ptr->x;
	base_ptr->y_ref = base_ptr->y;
	base_ptr->phi_ref = base_ptr->phi;

	base_ptr->motor_r_dir = 0;
	base_ptr->motor_l_dir = 0;

	base_ptr->motor_r_ctrl = 0;
	base_ptr->motor_l_ctrl = 0;
	base_ptr->motor_r_ctrl_uint = 0;
	base_ptr->motor_l_ctrl_uint = 0;

	set_motor_r_dir (base_ptr->motor_r_dir);
	set_motor_l_dir (base_ptr->motor_l_dir);
	pwm_right_dc (base_ptr->motor_r_ctrl_uint);
	pwm_left_dc (base_ptr->motor_l_ctrl_uint);
}
