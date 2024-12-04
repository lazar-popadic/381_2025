/*
 * base.c
 *
 *  Created on: Dec 4, 2024
 *      Author: lazar
 */

#include "main.h"

volatile struct_robot_base *base_ptr;
static volatile struct_robot_base base;

void
base_init()
{
  base_ptr = &base;
}

volatile struct_robot_base*
get_robot_base ()
{
  return base_ptr;
}

float
get_x_des ()
{
  return base_ptr->x_des;
}
