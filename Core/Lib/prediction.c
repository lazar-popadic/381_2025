/*
 * prediction.c
 *
 *  Created on: Nov 27, 2024
 *      Author: lazar
 */

#include "main.h"

static uint16_t points = 0;

uint16_t
get_pts ()
{
  return points;
}

void
set_pts (uint16_t pts)
{
  points = pts;
}

void
add_pts (uint16_t pts)
{
  points += pts;
}
