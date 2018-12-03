/********************************************************************************

			Accessing the accelerometer data (advance)
			Novembre 7 2005 Borter Jean-Joel


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005 Borter Jean-Joel

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup a_d
 * \brief Accessing the accelerometer data.
 *
 * The functions of this file are made to deal with the accelerometer
 * data. You can know the magnitude, the orientation, the inclination, ...
 * of the acceleration that the e-puck is enduring.
 * \n \n Two structures are used:
 * - TypeAccSpheric to store the acceleration data on sherical coordinates.
 * - TypeAccRaw to store the acceleration data on cartesian coordinates.
 *
 * A little exemple to read the accelerator.
 * \code
 * #include <p30F6014A.h>
 * #include <motor_led/e_epuck_ports.h>
 * #include <motor_led/e_init_port.h>
 * #include <a_d/advance_ad_scan/e_ad_conv.h>
 * #include <a_d/advance_ad_scan/e_acc.h>
 *
 * int main(void)
 * {
 * 	int z;
 *  	e_init_port();
 *  	e_init_ad_scan();
 *  	while(1)
 *  	{
 *  		long i;
 * 			z = e_get_acc(2);
 *  		if(z < 2100)	//LED4 on if e-puck is on the back
 * 		{
 *  			LED0 = 0;
 *  			LED4 = 1;
 * 		}
 *  		else		//LED0 on if e-puck is on his wells
 *  		{
 *  			LED0 = 1;
 *  			LED4 = 0;
 * 		}
 * 		for(i=0; i<100000; i++) { asm("nop"); }
 *		}
 *  }
 * \endcode
 * \author Code: Borter Jean-Joël \n Doc: Jonathan Besuchet
 */

#ifndef _ACC_DEFS
#define _ACC_DEFS


#define CST_RADIAN		(180.0/3.1415)	// used to convert radian in degrees
#define ANGLE_ERROR		666.0			// value returned if an angle can't be defined
#define FILTER_SIZE		5				// define the size of the averaging filter

// ID of the different captor in their respective array

#define ACCX_BUFFER	0
#define ACCY_BUFFER	1
#define ACCZ_BUFFER	2


#define GRAVITY 768             // 1 g fot 10 bits accelerometer
#define GRAVITY_LSM330 16384    // 1 g for 16 bits accelerometer

/*! \struct TypeAccSpheric
 * \brief struct to store the acceleration vector in spherical coord
 */
typedef struct
{
	float acceleration;		/*!< lenght of the acceleration vector
							 * = intensity of the acceleration */
	float orientation;		/*!< orientation of the acceleration vector
							 * in the horizontal plan, zero facing front
							 * - 0° = inclination to the front
							 * (front part lower than rear part)
							 * - 90° = inclination to the left
							 * (left part lower than right part)
							 * - 180° = inclination to the rear
							 * (rear part lower than front part)
							 * - 270° = inclination to the right
							 * (right part lower than left part) */
	float inclination;		/*!< inclination angle with the horizontal plan
							 * - 0° = e-puck horizontal
							 * - 90° = e-puck vertical
							 * - 180° = e-puck horizontal but up-side-down */
} TypeAccSpheric;

/*! \struct TypeAccRaw
 * \brief struct to store the acceleration raw data
 * in carthesian coord
 */
typedef struct
{
	int acc_x;	/*!< The acceleration on x axis */
	int acc_y;	/*!< The acceleration on y axis */
	int acc_z;	/*!< The acceleration on z axis */
} TypeAccRaw;


/**********************************************************************
 * ---------------------- Functions from acc.c ------------------------
 **********************************************************************/

int e_get_acc(unsigned int captor);
int e_get_acc_filtered(unsigned int captor, unsigned int filter_size);
TypeAccSpheric e_read_acc_spheric(void);
float e_read_orientation(void);
float e_read_inclination(void);
float e_read_acc(void);

TypeAccRaw e_read_acc_xyz(void);
int e_read_acc_x(void);
int e_read_acc_y(void);
int e_read_acc_z(void);

void e_acc_calibr(void);
void e_display_angle(void);
#endif
