/*
 * curve.c
 *
 *  Created on: Feb 16, 2025
 *      Author: lazar
 */

#include "main.h"

int8_t avoid_obst_glb = 0;
int8_t curve_ready = 0;

void
create_curve (curve *curve_ptr, float x_ref, float y_ref, float phi_ref, int dir, int avoid_obst)
{
	static float p0_x;
	static float p0_y;
	static float p1_x;
	static float p1_y;
	static float p2_x;
	static float p2_y;
	static float p3_x;
	static float p3_y;

	wrap180_ptr (&phi_ref);
	p0_x = get_robot_base ()->x;
	p0_y = get_robot_base ()->y;

	p1_x = get_robot_base ()->x + OFFS_ROBOT * cos ((get_robot_base ()->phi / 180 + dir * (-0.5) + 0.5) * M_PI);
	p1_y = get_robot_base ()->y + OFFS_ROBOT * sin ((get_robot_base ()->phi / 180 + dir * (-0.5) + 0.5) * M_PI);

	p2_x = x_ref - OFFS_DESIRED * cos ((phi_ref / 180 + dir * (-0.5) + 0.5) * M_PI);
	p2_y = y_ref - OFFS_DESIRED * sin ((phi_ref / 180 + dir * (-0.5) + 0.5) * M_PI);

	p3_x = x_ref;
	p3_y = y_ref;

	curve_ptr->dis = 0;
	cubic_bezier_pts (curve_ptr, p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y);

	equ_coords (curve_ptr);
	pad_curve (curve_ptr, dir);
	avoid_obst_glb = avoid_obst;
	curve_ptr->goal_phi = phi_ref;
	curve_ready = 1;
}

int8_t
get_curve_ready ()
{
	return curve_ready;
}

void
set_curve_ready (int8_t ready)
{
	curve_ready = ready;
}

void
cubic_bezier_pts (curve *curve_ptr, float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y)
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
}

void
equ_coords (curve *curve_ptr)
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
				}
		}
	curve_ptr->equ_pts_x[curve_ptr->num_equ_pts] = curve_ptr->pts_x[BEZIER_RESOLUTION - 1];
	curve_ptr->equ_pts_y[curve_ptr->num_equ_pts] = curve_ptr->pts_y[BEZIER_RESOLUTION - 1];
}

void
pad_curve (curve *curve_ptr, int8_t dir)
{
	for (int i = 1; i < PAD_NUM; i++)
		{
			curve_ptr->equ_pts_x[curve_ptr->num_equ_pts + i] = curve_ptr->equ_pts_x[curve_ptr->num_equ_pts]
					+ dir * i * POINT_DISTANCE * cos (curve_ptr->goal_phi * M_PI / 180);
			curve_ptr->equ_pts_y[curve_ptr->num_equ_pts + i] = curve_ptr->equ_pts_y[curve_ptr->num_equ_pts]
					+ dir * i * POINT_DISTANCE * sin (curve_ptr->goal_phi * M_PI / 180);
		}
}
