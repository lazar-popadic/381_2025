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
  // position, velocity, acceleration of base
  float x;						// [mm]
  float y;						// [mm]
  float phi;					// [deg]
  float v;						// [mm/ms = m/s]
  float w;						// [deg/s]
  float a;						// [mm/ms * 1/ms]
  float alpha;					// [deg/s * 1/ms]
  float v_left;					// [mm/ms = m/s]
  float v_right;				// [mm/ms = m/s]
  float v_prev;					// [mm/ms = m/s]
  float w_prev;					// [deg/s]

  // references for position, velocity of base
  float v_ref;					// [mm/ms = m/s]
  float w_ref;					// [deg/s]
  float x_ref;					// [mm]
  float y_ref;					// [mm]
  float phi_ref;				// [deg]

  // limits for velocity, acceleration, jerk of base
  float v_max;					// [mm/ms = m/s]
  float w_max;					// [deg/s]
  float a_max;					// [mm/ms * 1/ms]
  float alpha_max;				// [deg/s * 1/ms]
  float j_max;					// [mm/ms * 1/ms^2]	// TODO: proveri jedinice
  float j_rot_max;				// [deg/s * 1/ms^2]

  // control signals
  float v_ctrl;
  float w_ctrl;
  float motor_l_ctrl;
  float motor_r_ctrl;
  int8_t motor_l_dir:2;
  int8_t motor_r_dir:2;
  uint16_t motor_l_ctrl_uint;
  uint16_t motor_r_ctrl_uint;

  // status
  uint8_t moving :1;			// statusi po poziciji i brzini baze
  uint8_t on_target :1;
  uint8_t movement_started :1;	// statusi za same taskove kretnje
  uint8_t movement_finished :1;

} struct_robot_base;

typedef struct st_pid
{
  float p;
  float i;
  float d;
  float lmt;
  float ctrl;
  float ctrl_p;
  float ctrl_pp;
  float err_p;
  float err_sum;
  float err_dif;
  float sum_lmt;

} struct_pid;

typedef struct st_tactic_num
{
  uint8_t side :1;
  uint8_t num :2;

} struct_tactic_num;

#endif /* INC_STRUCTS_H_ */
