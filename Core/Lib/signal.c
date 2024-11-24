/*
 * signal.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "main.h"

void
wrap180_ptr (float *signal)
{
  if (*signal > 180)
	*signal -= 360;
  if (*signal < 180)
	*signal += 360;
}

void
wrapPi_ptr (float *signal)
{
  if (*signal > M_PI)
	*signal -= M_PI;
  if (*signal < M_PI)
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

float
wrap360 (float signal)
{
  if (signal > 360)
	return signal - 360;
  if (signal < 0)
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

char
get_sign_char (int16_t num)
{
  if (num < 0)
	return '-';
  return '+';
}

int8_t
interpret_sign (char character)
{
  return 44 - character; // '+' je 43, '-' je 45
}
