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
 *			z = e_get_acc(2);
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
 * \author Code: Borter Jean-Joel \n Doc: Jonathan Besuchet
 */

#include "math.h"
#include "e_acc.h"
#include "e_ad_conv.h"
#include "../../motor_led/e_epuck_ports.h"
#include "../../motor_led/advance_one_timer/e_led.h"
#include "../../acc_gyro/e_lsm330.h"
#include "../../motor_led/e_init_port.h"
#include <stdlib.h>

extern unsigned char updateAccI2CCounter;
extern int e_acc_scan[3][ACC_SAMP_NB];
extern unsigned int e_last_acc_scan_id;
unsigned char isCalibratingFlag = 0;

/*****************************************************
 * internal variables                                *
 *****************************************************/
static int angle_mem = 0;			//used in the display_angle function

static int centre_x = 0;			//zero value for x axe
static int centre_y = 0;			//zero value for y axe
int centre_z = 2000;			//zero value for z axe

static int acc_x, acc_y, acc_z;	//raw acceleration values in carthesian coord.
static float acceleration, orientation, inclination;	//spherical coord. of the accel vector
int lastAccX, lastAccY, lastAccZ;

/*****************************************************
 * internal function                                 *
 *****************************************************/

/*! \brief Read the last value of a given accelerator axes
 * \param captor		ID of the AD channel to read
 *						(must be 0 = x, 1 = y or 2 = z)
 * \return value		filtered channel's value
 */
int e_get_acc(unsigned int captor)
{
    int tempValue=0;
    if(isEpuckVersion1_3()) {
        // Try to optimize the I2C requests: the time is based on the ADC ISR (64*1/16384=about 3.9 ms, that is 256 Hz as with analog accelerometer),
        // so we get and update for all 3 channels (with a single call) every 256 Hz independently of the requested channel.
        // This is to avoid especially situations in which the user requires all 3 channels (to do some maths) one after the other, in this case
        // it would requires 3 I2C calls that we can avoid this way.
        if(updateAccI2CCounter == 64) {
            getAllAxesAcc(&lastAccX, &lastAccY, &lastAccZ);
            updateAccI2CCounter = 0;
        }
	switch(captor) {
            // Shift by 4 the value received from the sensor (16 bits) to get 12 bits value as with the analog accelerometer (to maintain compatibility with e-puck rev < 1.3)
            // Moreover decrease the resulting value by 1/4 to get about the same resolution for the g:
            // - e-puck rev < 1.3: 1 g = about 800
            // - e-puck rev 1.3: 1 g = about 1000 => decreased by 1/4 => about 750 (5% difference)
            case 0: tempValue = lastAccY - centre_y;   // exchange x and y axes in order to be consistent with e-puck rev < 1.3
                return (tempValue>>4) - (tempValue>>6);
            case 1: tempValue = lastAccX - centre_x;
                return (tempValue>>4) - (tempValue>>6);
            case 2: tempValue = lastAccZ - centre_z;
                return (tempValue>>4) - (tempValue>>6);
            default: return((int)ANGLE_ERROR);
	}
    } else {
	switch(captor) {
            case 0: return (e_acc_scan[captor][e_last_acc_scan_id] - centre_x);
            case 1: return (e_acc_scan[captor][e_last_acc_scan_id] - centre_y);
            case 2: return (e_acc_scan[captor][e_last_acc_scan_id] - centre_z);
            default: return((int)ANGLE_ERROR);
	}
    }
}

/*! \brief Read the value of a channel, filtered by an averaging filter
 * \param captor		ID of the AD channel to read (must be 0 to 2)
 * \param filter_size	size of the filter (must be between 1 and SAMPLE_NUMBER)
 * \return value		filtered channel's value
 */
int e_get_acc_filtered(unsigned int captor, unsigned int filter_size)
{
    long temp = 0;
    int i,j,tempValue;

    if(isEpuckVersion1_3()) {
        // Filter no more required for the new chip used with e-puck rev 1.3.

        // Try to optimize the I2C requests: the time is based on the ADC ISR (64*1/16384=about 3.9 ms, that is 256 Hz as with analog accelerometer),
        // so we get and update for all 3 channels (with a single call) every 256 Hz independently of the requested channel.
        // This is to avoid especially situations in which the user requires all 3 channels (to do some maths) one after the other, in this case
        // it would requires 3 I2C calls that we can avoid this way.
        if(updateAccI2CCounter == 64) {
            getAllAxesAcc(&lastAccX, &lastAccY, &lastAccZ);
            updateAccI2CCounter = 0;
        }
        if(isCalibratingFlag) {
            switch(captor) {
                case 0: return getYAxisAcc();   // exchange x and y axes in order to be consistent with e-puck 1.0;
                case 1: return getXAxisAcc();
                case 2: return getZAxisAcc();
                default: return((int)ANGLE_ERROR);
            }
        } else {
            switch(captor) {
                // Shift by 4 the value received from the sensor (16 bits) to get 12 bits value as with the analog accelerometer (to maintain compatibility with e-puck rev < 1.3)
                // Moreover decrease the resulting value by 1/4 to get about the same resolution for the g:
                // - e-puck rev < 1.3: 1 g = about 800
                // - e-puck rev 1.3: 1 g = about 1000 => decreased by 1/4 => about 750 (5% difference)
                // Add an offset of 2048 because it represents the value of 0 g with the analog accelerometer (to maintain compatibility with e-puck rev < 1.3).
                case 0: tempValue = lastAccY;  // exchange x and y axes in order to be consistent with e-puck rev < 1.3;
                    return ((tempValue>>4)-(tempValue>>6))+2048;
                case 1: tempValue = lastAccX;
                    return ((tempValue>>4)-(tempValue>>6))+2048;
                case 2: tempValue = lastAccZ;
                    return ((tempValue>>4)-(tempValue>>6))+2048;
                default: return((int)ANGLE_ERROR);
            }
        }
    } else {

	// channel ID must be between 0 to 13 and
	// filter_size must be between 1 to SAMPLE_NUMBER
	if ((captor < 3) &&
		(filter_size > 0) && (filter_size <= ACC_SAMP_NB))
	{
		for (i=0, j=(ACC_SAMP_NB-(filter_size-(e_last_acc_scan_id+1)))%ACC_SAMP_NB ; i<filter_size ; i++, j=(j+1)%ACC_SAMP_NB)
		{
			temp += e_acc_scan[captor][j];
		}
	}
	return ((int)(temp/filter_size));
    }
}

/*! \brief read the x, y, z values, apply an averaging filter
 * and finaly subbstract the center values.
 */
void calculate_acc_raw(void)
{
    if(isEpuckVersion1_3()) {
        acc_x = e_get_acc(0);
        acc_y = e_get_acc(1);
        acc_z = e_get_acc(2);
    } else {
	acc_x = e_get_acc_filtered(ACCX_BUFFER, FILTER_SIZE) - centre_x;	// generates positive
	acc_y = e_get_acc_filtered(ACCY_BUFFER, FILTER_SIZE) - centre_y;	// and negative value
	acc_z = e_get_acc_filtered(ACCZ_BUFFER, FILTER_SIZE) - centre_z;	// to make the use easy
    }
	return;
}

/*! \brief calculate the intensity of the acceleration vector,
 * and the Euler's angles
 */
void calculate_acc_spherical(void)
{
	calculate_acc_raw();

// calculat the absolute acceleration value
	acceleration = sqrtf((float)(((long)acc_x * (long)acc_x) + ((long)acc_y * (long)acc_y) + ((long)acc_z * (long)acc_z)));

	inclination =  90.0 - atan2f((float)(acc_z), sqrtf( (float)(((long)acc_x * (long)acc_x) + ((long)acc_y * (long)acc_y) ))) * CST_RADIAN;

	if (inclination<5 || inclination>160)
		orientation=0;
	else
		orientation = (atan2f((float)(acc_x), (float)(acc_y)) * CST_RADIAN) + 180.0;		// 180 is added to have 0 to 360° range

	return;
}


/*****************************************************
 * user called function                               *
 *****************************************************/

/*! \brief initialize de ad converter and calculate the zero
 * values
 *
 * It reads two times the average_size to avoid
 * edge effects then it reads 100 values and average them to initiate
 * the "zero" value of the accelerometer
 */
void e_acc_calibr(void)
{
        isCalibratingFlag = 1;
	centre_x = e_get_acc_filtered(ACCX_BUFFER, 50);
	centre_y = e_get_acc_filtered(ACCY_BUFFER, 50);
        if(isEpuckVersion1_3()) {
            centre_z = e_get_acc_filtered(ACCZ_BUFFER, 50) - GRAVITY_LSM330;
        } else {
            centre_z = e_get_acc_filtered(ACCZ_BUFFER, 50) - GRAVITY;
        }
        isCalibratingFlag = 0;
}

/*! \brief calculate and return the accel. in spherical coord
 * \return acceleration in spherical coord
 * \sa TypeAccSpheric
 */
TypeAccSpheric e_read_acc_spheric(void)
{
	TypeAccSpheric result;

	calculate_acc_spherical();
	result.acceleration = acceleration;
	result.orientation  = orientation;
	result.inclination  = inclination;

	return(result);
}

/*! \brief calculate and return the inclination angle
 * \return inclination angle of the robot
 */
float e_read_inclination(void)
{
	calculate_acc_spherical();

	return(inclination);
}

/*! \brief calculate and return the orientation angle
 * \return orientation of the accel vector
 */
float e_read_orientation(void)
{
	calculate_acc_spherical();

	return(orientation);
}

/*! \brief calculate and return the intensity of the acceleration
 * \return intensity of the acceleration vector
 */
float e_read_acc(void)
{
	calculate_acc_spherical();	//calculate intesnsity and euler's angle

	return(acceleration);
}

/*! \brief calculate and return acceleration on the x,y,z axis
 * \return acceleration on the x,y,z axis
 * \sa TypeAccRaw
 */
TypeAccRaw e_read_acc_xyz(void)
{
	TypeAccRaw result;

	calculate_acc_raw();
	result.acc_x = acc_x;
	result.acc_y = acc_y;
	result.acc_z = acc_z;

	return(result);
}

/*! \brief calculate and return acceleration on the x axis
 * \return acceleration on the x axis
 */
int e_read_acc_x(void)
{
	calculate_acc_raw();

	return(acc_x);
}

/*! \brief calculate and return acceleration on the y axis
 * \return acceleration on the y axis
 */
int e_read_acc_y(void)
{
	calculate_acc_raw();

	return(acc_y);
}

/*! \brief calculate and return acceleration on the z axis
 * \return acceleration on the z axis
 */
int e_read_acc_z(void)
{
	calculate_acc_raw();

	return(acc_z);
}

/*! \brief light the led according to the orientation angle */
void e_display_angle(void)
{
	float angle = 0.0;

// 	To avoid oscillation the limite of variation is limited at
//  a fix value	wich is 1/9 of the LED resolution
	angle = e_read_orientation();
	if ( abs(angle_mem - angle) > 5.0)
	{
		LED0=LED1=LED2=LED3=LED4=LED5=LED6=LED7=0;
			// table of selection
		if ( 	  (angle > (360.0 - 22.5)) |  (angle <= (0.0   + 22.5)) && angle != ANGLE_ERROR)
			LED0 = 1;
		else if ( angle > (45.0 - 22.5)  && angle <= (45.0  + 22.5))
			LED7 = 1;
		else if ( angle > (90.0 - 22.5)  && angle <= (90.0  + 22.5))
			LED6 = 1;
		else if ( angle > (135.0 - 22.5) && angle <= (135.0 + 22.5))
			LED5 = 1;
		else if ( angle > (180.0 - 22.5) && angle <= (180.0 + 22.5))
			LED4 = 1;
		else if ( angle > (225.0 - 22.5) && angle <= (225.0 + 22.5))
			LED3 = 1;
		else if ( angle > (270.0 - 22.5) && angle <= (270.0 + 22.5))
			LED2 = 1;
		else if ( angle > (315.0 - 22.5) && angle <= (315.0 + 22.5))
			LED1 = 1;
		angle_mem = angle;	// value to compare with the next one
	}
}
