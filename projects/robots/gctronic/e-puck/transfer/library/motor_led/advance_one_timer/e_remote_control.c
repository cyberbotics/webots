/********************************************************************************

			Control IR receiver module
			December 2005: first version
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
 * \brief Manage the IR receiver module (timer2)
 *
 * This module manage the IR receiver with the agenda solution (timer2).
 *
 * Alittle exemple to manage the IR remote (the body LED change his state when
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
 * \author Code: Francesco Mondada, Lucas Meier \n Doc: Jonathan Besuchet
 */

#include "e_remote_control.h"
#include "../e_epuck_ports.h"
#include "e_agenda.h"
#include "e_led.h"

/*------ internal variables ------*/

static unsigned char address_temp = 0;
static unsigned char data_temp = 0;
static unsigned char check_temp = 0;

static unsigned char address = 0;
static unsigned char data = 0;
static unsigned char check = 2;


/*! \brief Initialise the IR receiver ports */
void e_init_remote_control(void) // initialisation for IR interruptions on INT0
{
	REMOTE_DIR = INPUT_PIN;			//sets the RF6 pin as input
	INTCON2bits.INT0EP = 1;   	//set interrupt polarity to falling edge
	IFS0bits.INT0IF = 0;		//clear to enable interrupt
	IEC0bits.INT0IE = 1;		//enable interrupt on INT0
	return;
}

void __attribute__((__interrupt__, auto_psv))
 _INT0Interrupt(void) // interrupt for IR receiver
{
	IEC0bits.INT0IE = 0;   			//disable interrup from falling edge
//	e_set_led(1,1);
	e_activate_agenda(e_read_remote_control, 21); //activate the IR Receiver agenda with a 2.1[ms] cycle value
	check_temp = address_temp = data_temp = 0;
	return;
}

/*! \brief Read the signal and stock the information */
void e_read_remote_control(void) // interrupt from timer for next bits
{
	static int i = -1;


	if (i == -1)	// start bit confirm  change timer period
	{
		if(REMOTE){
			//if high it is only a noise
				IEC0bits.INT0IE = 1;   	//enable interrupt from falling edge
				IFS0bits.INT0IF = 0;    //clear interrupt flag from first receive !
				e_destroy_agenda(e_read_remote_control);
				i = -1;
			}
		else			   // read the check bit
			{
				e_set_agenda_cycle(e_read_remote_control, 9); //cycle value is 0.9 to go to check bit[ms]
				check_temp = address_temp = data_temp = 0;
				//e_set_led(1,1);
				i=0;
			}
	}
//	e_set_led(2,1);

	else if (i == 1)	// check bit read and change timer period
	{
//	e_set_led(3,1);
		check_temp = REMOTE;	   // read the check bit
		e_set_agenda_cycle(e_read_remote_control, 18); //cycle value is 1.778[ms]
		e_set_led(1,1);
	}
	else if ((i > 1) && (i < 7)) // we read address
	{
//	e_set_led(4,1);

		unsigned char temp = REMOTE;
		temp <<= 6-i;
		address_temp += temp;
	}
	else if ((i > 6) && (i < 13 )) // we read data
	{
//			e_set_led(5,1);

		unsigned char temp = REMOTE;
		temp <<= 6+6-i;
		data_temp += temp;
	}

	else if (i == 13) // last bit read
	{
		e_set_led(1,0);
		IEC0bits.INT0IE = 1;   	//enable interrupt from falling edge
		IFS0bits.INT0IF = 0;    //clear interrupt flag from first receive !
		e_destroy_agenda(e_read_remote_control);
		i = -1;
		check = check_temp;
		address = address_temp;
		data = data_temp;
	}

	if(i!=-1)
		i++;
}

/*------ user calls ------*/

/** \brief Read the check bit
 * \return	check	check bit of the signal
 */
unsigned char e_get_check(void) {
	return check;
}

/** \brief Read the adress of the commande
 * \return	adress	adress part of the signal
 */
unsigned char e_get_address(void) {
	return address;
}

/** \brief Read the data of the command
 * \return	data	data part of the signal
 */
unsigned char e_get_data(void) {
	return data;
}
