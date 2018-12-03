/********************************************************************************

			Advance control led of e-puck
			December 2004: first version
			Lucas Meier & Francesco Mondada
			August 2007: Led effects added
			Jonathan Besuchet


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Francesco Mondada, Lucas Meier
(c) 2007 Jonathan Besuchet

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the LEDs with blinking possibility (timer2).
 *
 * Here we use the agenda solution to make the LED blinking.
 *
 * A little exemple for LEDs blinking with agenda (all LEDs blink with 100ms delay)
 * \warning this program uses the \ref e_blink_led(void) function.
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/advance_one_timer/e_led.h>
 * #include <motor_led/advance_one_timer/e_agenda.h>
 *
 * int main(void)
 * {
 * 	e_init_port();
 * 	e_activate_agenda(e_blink_led, 1000); //blink with 100ms
 * 	e_start_agendas_processing();
 * 	while(1) {}
 * }
 * \endcode
 * \sa e_agenda.h
 * \author Code: Francesco Mondada, Lucas Meier, Jonathan Besuchet \n Doc: Jonathan Besuchet
 */

#ifndef _LED
#define _LED

/* functions */
void e_set_led(unsigned int led_number, unsigned int value); // set led_number (0-7) to value (0-1)
void e_led_clear(void);
void e_blink_led(void);
void e_blink_led0(void);
void e_blink_led1(void);
void e_blink_led2(void);
void e_blink_led3(void);
void e_blink_led4(void);
void e_blink_led5(void);
void e_blink_led6(void);
void e_blink_led7(void);

void e_set_body_led(unsigned int value); // value (0=off 1=on higher=inverse)
void e_set_front_led(unsigned int value); //value (0=off 1=on higher=inverse)

void e_start_led_blinking(int cycle);
void e_stop_led_blinking(void);

void flow_led(void);
void snake_led(void);
void k2000_led(void);
void right_led(void);
void left_led(void);

#endif
