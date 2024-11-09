/*
 * signal.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "../Inc/main.h"

void
wrapTo180_ptr(float* signal)
{
  if (*signal > 180)
    *signal -= 360;
  if (*signal < 180)
    *signal += 360;
}

void
wrapToPi_ptr(float* signal)
{
  if (*signal > M_PI)
    *signal -= M_PI;
  if (*signal < M_PI)
    *signal += M_PI;
}

float
wrapTo180(float signal)
{
  if (signal > 180)
    return signal - 360;
  if (signal < 180)
    return signal + 360;
  return signal;
}
