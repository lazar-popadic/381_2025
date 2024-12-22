/*
 * time.c
 *
 *  Created on: Aug 13, 2024
 *      Author: lazar
 */

#include "main.h"
#include "tim.h"

uint32_t sys_time_ms = 0;
uint8_t match_started = 0;
uint8_t delay_free = 1;

void
time_ISR ()	// poziva se u stm32f4xx_it.c
{
  sys_time_ms += match_started && 1;

  update_odom ();
//  // TODO: ovo sredi, vidi da li isto robot treba da uradi kada je regulacija iskljucena i kada robot naidje na prepreku: ne treba
//  // kada je regulacija iskljucena ne treba mi brzinska petlja
//  // kada naidje na prepreku treba mi brzinska petlja da bi drzao poziciju
//  // u oba slucaja poziciona treba da dobije trenutnu poziciju kao reference
//  switch (get_obstacle_detected())
//	{
//	case 1:
//	  position_loop ();
//	  break;
//	case 0:
//	  // TODO: ovde bi trebalo da samo postavi brzinske reference na 0, funkciju clear_vel_ref()
//	  // i razmisli da li treba resetujes promenljivu phase
//	  // u go_to_xy treba i nista se ne gubi
//	  // u follow_curve mozda moze biti problem, ali to je ako je u samom kraju krivine, mogu da proverim da li je previse blizu kraja na pocetku zadate kretnje, i da onda imam 3 faze zapravo gde je prva samo ta provera
//	  break;
//	}
//  switch (get_regulation_status ())
//	{
//	case 1:
//	  velocity_loop ();
//	  break;
//	case 0:
//	  stop_robot ();
//	  break;
//	}

  check_sensors ();
  update_base_status ();

  // rpi communication
  update_transmit_buffer ();
  edit_recieved_odom ();
}

uint8_t
delay_nb (uint32_t delay_ms)
{
  static uint32_t start_sys_time_ms;
  if (delay_free == 1)
	{
	  start_sys_time_ms = get_time_ms ();
	  delay_free = 0;
	}

  if (sys_time_ms <= start_sys_time_ms + delay_ms)
	return 0;
  delay_free = 1;
  return 1;
}

uint8_t
delay_nb_2 (uint32_t *start_time, uint32_t delay_ms)
{
  *start_time = uint_min (*start_time, get_time_ms ());

  if (sys_time_ms <= *start_time + delay_ms)
	return 0;
  *start_time = 0xffffffff;
  return 1;
}

void
time_start ()
{
  HAL_TIM_Base_Start_IT (&htim10);
}

void
time_stop ()
{
  HAL_TIM_Base_Stop_IT (&htim10);
}

void
start_match ()
{
  match_started = 1;
}

void
stop_match ()
{
  match_started = 0;
}

void
set_time_ms (uint32_t time)
{
  sys_time_ms = time;
}

uint32_t
get_time_ms ()
{
  return sys_time_ms;
}
