/********************************************************************************

			control IR receiver module
			september 2005 : first version
			Valentin Longchamp


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Valentin Longchamp

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
 * \author Code: Valentin Longchamp \n Doc: Jonathan Besuchet
 */

/********************************************************************
* 			control IR receiver module								*
*			september 2005 : first version							*
*			Valentin Longchamp										*
*																	*
********************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the IR receiver module (timer2)
 *
 * This module manage the IR receiver with the agenda solution (timer2).
 *
 * A little exemple to manage the IR remote (the body LED change his state when
 * you press a button of the IR controller).
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <motor_led/advance_one_timer/e_remote_control.h>
 * #include <motor_led/advance_one_timer/e_agenda.h>
 *
 * int main(void)
 * {
 * 	int ir_check;
 * 	int previous_check = 0;
 * 	e_init_port();
 * 	e_init_remote_control();
 * 	e_start_agendas_processing();
 * 	while(1)
 * 	{
 * 		ir_check = e_get_check();
 * 		if(ir_check != previous_check)
 * 			BODY_LED = BODY_LED^1;
 * 		previous_check = ir_check;
 * 	}
 * }
 * \endcode
 * \sa e_agenda.h
 * \author Jonathan Besuchet
 */

#ifndef _IR_REMOTE_CONTROLE
#define _IR_REMOTE_CONTROLE

/* defines for the keys on the remote controler
 * the numbers 0 to 9 are not defined since they are the same */
#define BOTTOMR		10 	// -/-- key
#define BOTTOMl		11 	// p<p key
#define	STANDBY		12
#define MUTE		13
#define VOL_UP		16
#define VOL_DOWN	17
#define CHAN_UP		32
#define CHAN_DOWN	33
#define I_II		35
#define OUT_AUX_1	56



/* functions */
void e_init_remote_control(void);
void e_read_remote_control(void);

unsigned char e_get_check(void);
unsigned char e_get_address(void);
unsigned char e_get_data(void);

#endif
