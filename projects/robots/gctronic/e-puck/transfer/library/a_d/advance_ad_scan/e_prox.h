/********************************************************************************

			Accessing the proximity sensor data (advance)
			Novembre 7 2005	Lucas Meier


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Lucas Meier

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup a_d
 * \brief Accessing proximity sensor of e-puck.
 *
 * The functions of this file are made to deal with the proximitiy
 * data. You can know the value of the ambient light detected by the
 * sensor. You can estimate the distance between the e-puck and an
 * obstacle by using e_get_prox function.
 *
 * A little exemple which turn the LED0 when an obstacle is detected
 * by the proximity sensor number 0.
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <a_d/advance_ad_scan/e_prox.h>
 * #include <a_d/advance_ad_scan/e_ad_conv.h>
 *
 * int main(void)
 * {
 * 	int proxy0;
 * 	e_init_port();
 * 	e_init_ad_scan();
 * 	while(1)
 * 	{
 * 		long i;
 * 		proxy0 = e_get_prox(0);
 * 		if(proxy0 < 1000)
 * 			LED0 = 0;
 * 		else
 * 			LED0 = 1;
 * 		for(i=0; i<100000; i++) { asm("nop"); }
 * 	}
 * }
 * \endcode
 * \author Code: Lucas Meier \n Doc: Jonathan Besuchet
 */

#ifndef _PROX
#define _PROX

void e_calibrate_ir();
int e_get_prox(unsigned int sensor_number); // to get a prox value
int e_get_calibrated_prox(unsigned int sensor_number);
int e_get_ambient_light(unsigned int sensor_number); // to get ambient light value

#endif
