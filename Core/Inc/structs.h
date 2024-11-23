/*
 * structs.h
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

typedef struct st_robot_base
{
  float x;			// [mm]
  float y;			// [mm]
  float phi;		// [deg]
  float v;			// [mm/ms = m/s]
  float w;			// [deg/s]
  float a;			// [mm/ms * 1/ms]
  float alpha;		// [deg/s * 1/ms]

  float v_ref;			// [mm/ms = m/s]
  float w_ref;			// [deg/s]
  float v_prev;			// [mm/ms = m/s]
  float w_prev;			// [deg/s]
  float v_max;			// [mm/ms = m/s]
  float w_max;			// [deg/s]

  float v_left;			// [mm/ms = m/s]
  float v_right;		// [mm/ms = m/s]
  int16_t encoder_sum_left;	// [inc]
  int16_t encoder_sum_right;	// [inc]

  uint8_t status_moving :1;
  uint8_t status_on_target :1;

} struct_robot_base;

#endif /* INC_STRUCTS_H_ */
