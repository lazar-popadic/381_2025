/*
 * signal.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "main.h"

void
wrap180_ptr (volatile float *signal)
{
  if (*signal > 180)
	*signal -= 360;
  if (*signal < -180)
	*signal += 360;
}

void
wrapPi_ptr (volatile float *signal)
{
  if (*signal > M_PI)
	*signal -= M_PI;
  if (*signal < -M_PI)
	*signal += M_PI;
}

float
wrap180 (float signal)
{
  if (signal > 180)
	return signal - 360;
  if (signal < -180)
	return signal + 360;
  return signal;
}

int8_t
get_sign (float num)
{
  if (num > 0)
	return 1;
  if (num < 0)
	return -1;
  return 0;
}

void
saturation (volatile float *signal, float max, float min)
{
  if (*signal > max)
	*signal = max;
  else if (*signal < min)
	*signal = min;
}

void
scale_vel_ref (volatile float *ref_1, volatile float *ref_2, float limit)
{
  float abs_max_var = abs_max (*ref_1, *ref_2);
  if (abs_max_var > limit)
	{
	  float factor = limit / abs_max_var;
	  *ref_1 *= factor;
	  *ref_2 *= factor;
	}
}

float
abs_max (float a, float b)
{
  if (fabs (a) > fabs (b))
	return fabs (a);
  return fabs (b);
}

float
uint_min (uint32_t a, uint32_t b)
{
  if (a < b)
	return a;
  return b;
}

void
vel_ramp_up_ptr (float *signal, float reference, float acc)
{
  if (fabs (reference) - fabs (*signal) > acc)
	*signal += get_sign (reference) * acc;
  else
	*signal = reference;
}

float
vel_ramp_up (float signal, float reference, float acc)
{
  float edited_ref = signal;
  if (fabs (reference) - fabs (signal) > acc)
	edited_ref += get_sign (reference) * acc;
  else
	edited_ref = reference;
  return edited_ref;
}

float
vel_s_curve_up_webots (float *vel, float prev_vel, float vel_ref, float jerk)
{
  float acc_approx = *vel - prev_vel;
  float acc_calc = vel_ref - *vel;
  float out = *vel;

  if (fabs (vel_ref) > fabs (*vel))
	{
	  if (fabs (acc_calc) - fabs (acc_approx) > jerk)
		out = *vel + acc_approx + get_sign (vel_ref) * jerk;
	  else
		out = vel_ref;
	}
  return out;
}

float
vel_s_curve_up (float vel, float accel, float vel_ref, float jerk)
{
  float accel_des = vel_ref - vel;
  float edited_ref = vel_ref;

  if (fabs (vel_ref) > fabs (vel))	// ako ubrzava
	{
	  if (fabs (accel_des) - fabs (accel) > jerk)
		{
		  edited_ref = vel + accel + get_sign (vel_ref) * jerk;
		}
	}
  return edited_ref;
}

float
min3 (float a, float b, float c)
{
  float min = a;
  if (b < min)
	min = b;
  if (c < min)
	min = c;
  return min;
}
