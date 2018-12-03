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
 * obstacle by using \ref e_get_prox(unsigned int sensor_number) function.
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

#include "e_ad_conv.h"
#include "../../motor_led/e_epuck_ports.h"
#include "e_prox.h"

extern int e_ambient_ir[10];						// ambient light measurement
extern int e_ambient_and_reflected_ir[10];		// light when led is on

static int init_value_ir[10] = {0,0,0,0,0,0,0,0,0,0};

/*! \brief To calibrate your ir sensor
 * \warning Call this function one time before calling e_get_calibrated_prox
 */
void e_calibrate_ir()
{
	int i=0,j=0;
	int long t;
	int long tmp[10];

	for (;i<10;++i) {
		init_value_ir[i]=0;
		tmp[i]=0;
	}

	for (;j<100;++j) {
		for (i=0;i<10;++i) {
			tmp[i]+=(e_get_prox(i));
			for (t=0;t<1000;++t);
		}
	}

	for (i=0;i<10;++i) {
		init_value_ir[i]=(int)(tmp[i]/(j*1.0));
	}
}

/*! \brief To get the analogic proxy sensor value of a specific ir sensor
 *
 * To estimate the proxymity of an obstacle, we do the following things:
 * - measure the ambient light
 * - turn on the IR led of the sensor
 * - measure the reflected light + ambient light
 * - calculate: reflected light = (reflected light + ambient light) - ambient light
 * - turn off the IR led of the sensor
 *
 * The result value of this function is: reflected light. More this value is great,
 * more the obsacle is near.
 * \param sensor_number The proxy sensor's number that you want the value.
 *                      Must be between 0 to 7.
 * \return The analogic value of the specified proxy sensor
 */
int e_get_prox(unsigned int sensor_number)
{
	if (sensor_number > 9)
		return 0;
	else
		return e_ambient_ir[sensor_number] - e_ambient_and_reflected_ir[sensor_number];
}

/*! \brief To get the calibrated value of the ir sensor
 *
 * This function return the calbrated analogic value of the ir sensor.
 * \warning Befroe using this function you have to calibrate your ir sensor (only one time)
 * by calling e_calibrate_ir().
 * \param sensor_number The proxy sensor's number that you want the calibrated value.
 *                      Must be between 0 to 7.
 * \return The analogic value of the specified proxy sensor
 */
int e_get_calibrated_prox(unsigned int sensor_number)
{
	int temp;
	if (sensor_number > 9)
		return 0;
	else
	{
		temp=(e_ambient_ir[sensor_number] - e_ambient_and_reflected_ir[sensor_number])
				- init_value_ir[sensor_number];
		if (temp>0)
			return temp;
		else
			return 0;
	}
}

/*! \brief To get the analogic ambient light value of a specific ir sensor
 *
 * This function return the analogic value of the ambient light measurement.
 * \param sensor_number The proxy sensor's number that you want the value.
 *                      Must be between 0 to 7.
 * \return The analogic value of the specified proxy sensor
 */
int e_get_ambient_light(unsigned int sensor_number)
{
	if (sensor_number > 9)
		return 0;
	else
		return e_ambient_ir[sensor_number];
}
