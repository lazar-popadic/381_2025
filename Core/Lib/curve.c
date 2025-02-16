/*
 * curve.c
 *
 *  Created on: Feb 16, 2025
 *      Author: lazar
 */

#include "main.h"

int avoid_obst_glb = 0;

int
create_curve (st_curve *curve_ptr, float x_ref, float y_ref, float phi_ref, int dir, int avoid_obst)
{
  curve_ptr->max_ang_change = 0;
  static float p0_x;
  static float p0_y;
  static float p1_x;
  static float p1_y;
  static float p2_x;
  static float p2_y;
  static float p3_x;
  static float p3_y;

  wrap180_ptr(&phi_ref);
  p0_x = get_robot_base ()->x;
  p0_y = get_robot_base ()->y;

  p1_x = get_robot_base ()->x + OFFS_ROBOT * cos ((get_robot_base ()->phi / 180 + dir * (-0.5) + 0.5) * M_PI);
  p1_y = get_robot_base ()->y + OFFS_ROBOT * sin ((get_robot_base ()->phi / 180 + dir * (-0.5) + 0.5) * M_PI);

  p2_x = x_ref - OFFS_DESIRED * cos ((phi_ref / 180 + dir * (-0.5) + 0.5) * M_PI);
  p2_y = y_ref - OFFS_DESIRED * sin ((phi_ref / 180 + dir * (-0.5) + 0.5) * M_PI);

  p3_x = x_ref;
  p3_y = y_ref;

  curve_ptr->dis = 0;
// TODO: dovde
  if (cubic_bezier_pts (curve_ptr, p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y))
	return 1;

  curve_ptr->equ_pts_x = (float*) malloc ((curve_ptr->dis / POINT_DISTANCE + 2) * sizeof(float));
  curve_ptr->equ_pts_y = (float*) malloc ((curve_ptr->dis / POINT_DISTANCE + 2) * sizeof(float));

  equ_coords (curve_ptr);
  if (avoid_obst == 1)
	{
	  avoid_obst_glb = 1;
	}
  else
	avoid_obst_glb = 0;
  return 0;
}

int
cubic_bezier_pts (st_curve *curve_ptr, float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y)
{
  for (int i = 0; i < BEZIER_RESOLUTION; i++)
	{
	  float t = (float) i / BEZIER_RESOLUTION;
	  curve_ptr->pts_x[i] = pow (1 - t, 3) * p0_x + 3 * t * pow (1 - t, 2) * p1_x + 3 * pow (t, 2) * (1 - t) * p2_x + pow (t, 3) * p3_x;
	  curve_ptr->pts_y[i] = pow (1 - t, 3) * p0_y + 3 * t * pow (1 - t, 2) * p1_y + 3 * pow (t, 2) * (1 - t) * p2_y + pow (t, 3) * p3_y;
	  if (i > 0)
		{
		  curve_ptr->dis += sqrt (
			  (curve_ptr->pts_x[i] - curve_ptr->pts_x[i - 1]) * (curve_ptr->pts_x[i] - curve_ptr->pts_x[i - 1])
				  + (curve_ptr->pts_y[i] - curve_ptr->pts_y[i - 1]) * (curve_ptr->pts_y[i] - curve_ptr->pts_y[i - 1]));
		}
	}
  return 0;
}

void
equ_coords (st_curve *curve_ptr)
{
  float temp_length = 0;
  float cur_dis = 0;
  curve_ptr->num_equ_pts = 0;
  curve_ptr->equ_pts_x[0] = curve_ptr->pts_x[0];
  curve_ptr->equ_pts_y[0] = curve_ptr->pts_y[0];
  for (int i = 1; i < BEZIER_RESOLUTION; i++)
	{
	  cur_dis = sqrt (
		  (curve_ptr->pts_x[i] - curve_ptr->pts_x[i - 1]) * (curve_ptr->pts_x[i] - curve_ptr->pts_x[i - 1])
			  + (curve_ptr->pts_y[i] - curve_ptr->pts_y[i - 1]) * (curve_ptr->pts_y[i] - curve_ptr->pts_y[i - 1]));
	  curve_ptr->dis += cur_dis;
	  temp_length = sqrt (
		  (curve_ptr->equ_pts_x[curve_ptr->num_equ_pts] - curve_ptr->pts_x[i])
			  * (curve_ptr->equ_pts_x[curve_ptr->num_equ_pts] - curve_ptr->pts_x[i])
			  + (curve_ptr->equ_pts_y[curve_ptr->num_equ_pts] - curve_ptr->pts_y[i])
				  * (curve_ptr->equ_pts_y[curve_ptr->num_equ_pts] - curve_ptr->pts_y[i]));
	  if (temp_length >= POINT_DISTANCE)
		{
		  curve_ptr->num_equ_pts++;
		  temp_length = 0;
		  curve_ptr->equ_pts_x[curve_ptr->num_equ_pts] = curve_ptr->pts_x[i];
		  curve_ptr->equ_pts_y[curve_ptr->num_equ_pts] = curve_ptr->pts_y[i];
		  if (curve_ptr->num_equ_pts > 1)
			{
			  float cur_ang_change = 180 / M_PI
				  * (atan2 (curve_ptr->equ_pts_y[curve_ptr->num_equ_pts] - curve_ptr->equ_pts_y[curve_ptr->num_equ_pts - 1],
							curve_ptr->equ_pts_x[curve_ptr->num_equ_pts] - curve_ptr->equ_pts_x[curve_ptr->num_equ_pts - 1])
					  - atan2 (curve_ptr->equ_pts_y[curve_ptr->num_equ_pts - 1] - curve_ptr->equ_pts_y[curve_ptr->num_equ_pts - 2],
							   curve_ptr->equ_pts_x[curve_ptr->num_equ_pts - 1] - curve_ptr->equ_pts_x[curve_ptr->num_equ_pts - 2]));

			  curve_ptr->max_ang_change = abs_max (curve_ptr->max_ang_change, cur_ang_change);
			}
		}
	}
  curve_ptr->equ_pts_x[curve_ptr->num_equ_pts] = curve_ptr->pts_x[BEZIER_RESOLUTION - 1];
  curve_ptr->equ_pts_y[curve_ptr->num_equ_pts] = curve_ptr->pts_y[BEZIER_RESOLUTION - 1];
}

