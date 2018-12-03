/********************************************************************************

			Advance control motor of e-puck
			December 2004: first version
			Lucas Meier & Francesco Mondada


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Francesco Mondada, Lucas Meier

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the motors (with timer2)
 *
 * This module manage the motors with the agenda solution (timer2).
 *
 * A little exemple to use the motors with agenda (e-puck turn on himself)
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/advance_one_timer/e_motors.h>
 * #include <motor_led/advance_one_timer/e_agenda.h>
 *
 * int main(void)
 * {
 * 	e_init_port();
 * 	e_init_motors();
 * 	e_set_speed(-500, 500);
 * 	e_start_agendas_processing();
 * 	while(1) {}
 * }
 * \endcode
 * \sa e_agenda.h
 * \author Code: Francesco Mondada, Lucas Meier \n Doc: Jonathan Besuchet
 */

#include "./../e_epuck_ports.h"
#include "e_agenda.h"
#include <stdlib.h>

/* If powersave is enabled, the motor library will not leave
 * the motor's phase powered when running at low speed, thus reducing the heat
 * dissipation and power consumption
 *
 * TRESHV == the "virtual" speed when powersave is enabled, the phase will
 * be keept on 1/TRESHV seconds.
 *
 * Powersave will only be enabled for speed < MAXV
 */
#define POWERSAVE
#define TRESHV 650
#define MAXV 601
// following params are for maximum energy saving (but motors have small torque)
//#define TRESHV 1000
//#define MAXV 999

#if TRESHV <= MAXV
#error TRESHV must be higher than MAXV
#endif

/* internal variables */
static int left_speed = 0;
static int right_speed = 0;

static int left_motor_phase=0;	    // phase can be 0 to 3
static int right_motor_phase=0;    // phase can be 0 to 3

static int nbr_steps_left=0;
static int nbr_steps_right=0;

/*------ internal calls ------*/

/*! Change left motor phase according to the left_speed sign. */
 void run_left_motor(void)  // interrupt for motor 1 (of two) = left motor
{
  // increment or decrement phase depending on direction

#ifdef POWERSAVE
  static int phase_on = 0;
  if(phase_on && abs(left_speed) < MAXV) {
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
	MOTOR1_PHC = 0;
	MOTOR1_PHD = 0;
	phase_on = 0;
	e_set_agenda_cycle(run_left_motor, 10000/abs(left_speed) - 10000/TRESHV);
	return;
  }
#endif

  if (left_speed > 0) // inverted for the two motors
  {
    nbr_steps_left++;
    left_motor_phase--;
    if (left_motor_phase < 0) left_motor_phase = 3;
  }
  else
  {
    nbr_steps_left--;
    left_motor_phase++;
    if (left_motor_phase > 3) left_motor_phase = 0;
  }

  // set the phase on the port pins

  switch (left_motor_phase)
  {
    case 0:
    {
      MOTOR1_PHA = 0;
      MOTOR1_PHB = 1;
      MOTOR1_PHC = 0;
      MOTOR1_PHD = 1;
      break;
    }
    case 1:
    {
      MOTOR1_PHA = 0;
      MOTOR1_PHB = 1;
      MOTOR1_PHC = 1;
      MOTOR1_PHD = 0;
      break;
    }
    case 2:
    {
      MOTOR1_PHA = 1;
      MOTOR1_PHB = 0;
      MOTOR1_PHC = 1;
      MOTOR1_PHD = 0;
      break;
    }
    case 3:
    {
      MOTOR1_PHA = 1;
      MOTOR1_PHB = 0;
      MOTOR1_PHC = 0;
      MOTOR1_PHD = 1;
      break;
    }
  }
#ifdef POWERSAVE
  if(abs(left_speed) < MAXV) {
    phase_on = 1;
    e_set_agenda_cycle(run_left_motor,10000/TRESHV);
  }
#endif
}

/*! Change right motor phase according to the right_speed sign */
void run_right_motor(void)  // interrupt for motor 2 (of two) = right motor
{
  // increment or decrement phase depending on direction
#ifdef POWERSAVE
  static int phase_on = 0;
  if(phase_on && abs(right_speed) < MAXV) {
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
	MOTOR2_PHC = 0;
	MOTOR2_PHD = 0;
	phase_on = 0;
	e_set_agenda_cycle(run_right_motor, 10000/abs(right_speed) - 10000/TRESHV);
	return;
  }
#endif
  if (right_speed < 0)
  {
	nbr_steps_right--;
    right_motor_phase--;
    if (right_motor_phase < 0) right_motor_phase = 3;
  }
  else
  {
    nbr_steps_right++;
    right_motor_phase++;
    if (right_motor_phase > 3) right_motor_phase = 0;
  }

  // set the phase on the port pins

  switch (right_motor_phase)
  {
    case 0:
    {
      MOTOR2_PHA = 0;
      MOTOR2_PHB = 1;
      MOTOR2_PHC = 0;
      MOTOR2_PHD = 1;
      break;
    }
    case 1:
    {
      MOTOR2_PHA = 0;
      MOTOR2_PHB = 1;
      MOTOR2_PHC = 1;
      MOTOR2_PHD = 0;
      break;
    }
    case 2:
    {
      MOTOR2_PHA = 1;
      MOTOR2_PHB = 0;
      MOTOR2_PHC = 1;
      MOTOR2_PHD = 0;
      break;
    }
    case 3:
    {
      MOTOR2_PHA = 1;
      MOTOR2_PHB = 0;
      MOTOR2_PHC = 0;
      MOTOR2_PHD = 1;
      break;
    }
  }
#ifdef POWERSAVE
  if(abs(right_speed) < MAXV) {
    phase_on = 1;
    e_set_agenda_cycle(run_right_motor,10000/TRESHV);
  }
#endif
}

/* ---- user calls ---- */

/*! \brief Initialize the motors's agendas
 *
 * This function initialize the agendas used by the motors. In fact
 * it call \ref e_activate_agenda(void (*func)(void), int cycle) function.
 * \sa e_activate_agenda
 */
void e_init_motors(void)
{
  e_activate_agenda(run_left_motor, 0);
  e_activate_agenda(run_right_motor, 0);
}

/*! \brief Manage the left motor speed
 *
 * This function manage the left motor speed by changing the MOTOR1
 * phases. The changing phases frequency (=> speed) is controled by
 * the agenda (throw the function \ref e_set_agenda_cycle(void (*func)(void), int cycle)).
 * \param motor_speed from -1000 to 1000 give the motor speed in steps/s,
 * positive value to go forward and negative to go backward.
 * \sa e_set_agenda_cycle
 */
void e_set_speed_left(int motor_speed)
{
    // this is to avoid high current drain when the speed is changed very frequently
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
    MOTOR1_PHC = 0;
    MOTOR1_PHD = 0;

  // speed null
  if (motor_speed == 0)
  {
    left_speed = 0;
    e_set_agenda_cycle(run_left_motor, 0);
  }
  // speed inferior to the minimum value
  else if(motor_speed < -1000)
  {
    left_speed = -1000;
    e_set_agenda_cycle(run_left_motor, (int)-10000/left_speed);
  }
  // speed superior to the maximum value
  else if(motor_speed > 1000)
  {
    left_speed = 1000;
    e_set_agenda_cycle(run_left_motor, (int) 10000/left_speed);
  }
  else
  {
    left_speed = motor_speed;
  // negative speed
	if(motor_speed < 0)
		e_set_agenda_cycle(run_left_motor, (int)-10000/motor_speed);
  // positive speed
	else
		e_set_agenda_cycle(run_left_motor, (int) 10000/motor_speed);

  }
}

/*! \brief Manage the right motor speed
 *
 * This function manage the right motor speed by changing the MOTOR2
 * phases. The changing phases frequency (=> speed) is controled by
 * the agenda (throw the function \ref e_set_agenda_cycle(void (*func)(void), int cycle)).
 * \param motor_speed from -1000 to 1000 give the motor speed in steps/s,
 * positive value to go forward and negative to go backward.
 * \sa e_set_agenda_cycle
 */
void e_set_speed_right(int motor_speed)  // motor speed in percent
{
    // this is to avoid high current drain when the speed is changed very frequently
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
    MOTOR2_PHC = 0;
    MOTOR2_PHD = 0;

  // speed null
  if (motor_speed == 0)
  {
    right_speed = 0;
    e_set_agenda_cycle(run_right_motor, 0);
  }
  // speed inferior to the minimum value
  else if (motor_speed < -1000)
  {
    right_speed = -1000;
    e_set_agenda_cycle(run_right_motor, (int)-10000/right_speed);
  }
  // speed superior to the maximum value
  else if (motor_speed > 1000)
  {
    right_speed = 1000;
    e_set_agenda_cycle(run_right_motor, (int) 10000/right_speed);
  }
  else
  {
    right_speed = motor_speed;

  // negative speed
	if(motor_speed < 0)
		e_set_agenda_cycle(run_right_motor, (int)-10000/motor_speed);
  // positive speed
	else
		e_set_agenda_cycle(run_right_motor, (int) 10000/motor_speed);

  }
}

/*! \brief Manage linear/angular speed
 *
 * This function manage the speed of the motors according to the
 * desired linear and angular speed.
 * \param linear_speed	the speed in the axis of e-puck
 * \param angular_speed	the rotation speed (trigonometric)
 */
void e_set_speed(int linear_speed, int angular_speed)
{
	if(abs(linear_speed) + abs(angular_speed) > 1000)
		return;
	else
	{
		e_set_speed_left (linear_speed - angular_speed);
		e_set_speed_right(linear_speed + angular_speed);
	}
}

/*! \brief Give the number of left motor steps
 * \return The number of phases steps made since the left motor
 * is running.
 */
int e_get_steps_left()
{
	return nbr_steps_left;
}

/*! \brief Set the number of left motor steps
 * \param set_steps The number of changed phases that you want set.
 */
void e_set_steps_left(int set_steps)
{
	nbr_steps_left=set_steps;
}

/*! \brief Give the number of right motor steps
 * \return The number of phases steps made since the right motor
 * is running.
 */
int e_get_steps_right()
{
	return nbr_steps_right;
}

/*! \brief Set the number of right motor steps
 * \param set_steps The number of changed phases that you want set.
 */
void e_set_steps_right(int set_steps)
{
	nbr_steps_right=set_steps;
}
