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

#ifndef _MOTORS
#define _MOTORS

/* internal functions */
//void run_left_motor(void);
//void run_right_motor(void);

/* user called function */
void e_init_motors(void); 				// init to be done before using the other calls

void e_set_speed_left(int motor_speed);  // motor speed: from -1000 to 1000
void e_set_speed_right(int motor_speed); // motor speed: from -1000 to 1000
void e_set_speed(int linear_speed, int angular_speed);

void e_set_steps_left(int steps_left);
void e_set_steps_right(int steps_right);

int e_get_steps_left();
int e_get_steps_right();

#endif
