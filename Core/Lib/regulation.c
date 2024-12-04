/*
 * regulation.c
 *
 *  Created on: Dec 4, 2024
 *      Author: lazar
 */

#include "main.h"

int8_t reg_type = 0;

static float error_x = 0;				// [mm]
static float error_y = 0;				// [mm]
static float error_phi = 0;				// [deg]
static float error_phi_prim = 0;		// [deg]
static float distance = 0;				// [mm]
static float error_phi_prim_1 = 0;		// [deg]
static float error_phi_prim_2 = 0;		// [deg]
static float error_phi_final = 0;		// [deg]

float x_des = 0;				// [mm]
float y_des = 0;				// [mm]
float phi_des = 0;				// [deg]

static volatile struct_robot_base *base_ptr;

void
regulation_init ()
{
  base_ptr = get_robot_base ();

}

void
position_loop ()
{

}

void
move ()
{
  switch (reg_type)
	{
	case -1:
	  rotate ();
	  break;
	case 1:
	  go_to_xy ();
	  break;
	case 2:
	  follow_curve ();
	  break;
	}
}

static void
rotate ()
{
  error_phi = get_desired ().phi - get_robot ().get_position ().phi;
  wrap180_ptr (&error_phi);
  vel_ref = 0;
  ang_vel_ref = calculate (&ang_loop, error_phi);
  if (fabs (error_phi) < PHI_LIMIT)
	{
	  vel_ref = 0;
	  ang_vel_ref = 0;
	  set_reg_type (0);
	  movement_finished ();
	}
}

static void
go_to_xy ()
{
  error_x = get_desired ().x - get_robot ().get_position ().x;
  error_y = get_desired ().y - get_robot ().get_position ().y;
  error_phi_prim = atan2 (error_y, error_x) * 180 / M_PI + get_dir () * 180 - get_robot ().get_position ().phi;
  wrap180_ptr (&error_phi_prim);

  switch (phase)
	{
	case 0: // rot2pos
	  vel_ref = 0;
	  ang_vel_ref = calculate (&ang_loop, error_phi_prim);
	  if (fabs (error_phi_prim) < PHI_PRIM_LIMIT)
		phase = 1;
	  break;

	case 1: // tran
	  distance = (get_dir () * (-2) + 1) * sqrt (error_x * error_x + error_y * error_y);
	  if (fabs (error_phi_prim) > 90) // TODO: mozda bi ovde bilo dobro da mnozim sa kosinusom. mozda moze i bez faza, samo da uvek mnozim sa kosinusom
		vel_ref = -calculate (&dist_loop, distance);
	  else
		vel_ref = calculate (&dist_loop, distance);
	  // vel_ref = calculate(&dist_loop, distance* cos(error_phi_prim * M_PI / 180));
	  if (fabs (distance) > DISTANCE_LIMIT2)
		ang_vel_ref = calculate (&ang_loop, error_phi_prim);
	  else
		ang_vel_ref = 0;
	  if (fabs (distance) * cos (error_phi_prim) < 0)
		{
		  vel_ref = 0;
		  ang_vel_ref = 0;
		  phase = 0;
		  movement_finished ();
		}
	  break;
	}
}
