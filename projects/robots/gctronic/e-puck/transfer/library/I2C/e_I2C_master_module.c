/********************************************************************************

			I2C master module
			Version 1.0 may 2005 Davis Daidie


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Davis Daidie

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup i2c
 * \brief Manage I2C basics.
 *
 * This module manage the I2C basics functions (low level
 * I2C functions).
 * \n They are made to perform the basics tasks like:
 * - initializing the I2C on the microcontroller
 * - sending the Start bit (e_i2c_start)
 * - sending the Restart bit (e_i2c_restart)
 * - sending the Stop bit (e_i2c_stop)
 * - sending the acknowledgement bit (e_i2c_ack)
 * - writing or receiving a byte (e_i2c_write, e_i2c_read)
 * - ...
 *
 * \author Code: Davis Daidie \n Doc: Jonathan Besuchet
 */

#include "e_I2C_master_module.h"

char e_i2c_mode;
int  e_interrupts[3];

/*! \brief Wait until I2C Bus is Inactive */
void idle_i2c(void)
{
    while(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT);
}

/*! \brief Initialize the microcontroller for I2C uses
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_init(void)
{
	long i;
	I2CBRG=150;					// frequency of SCL at 100kHz
	I2CCONbits.I2CEN=0;			// disable I2C
	I2CCONbits.I2CEN=1;			// enable I2C
	IFS0bits.MI2CIF=0;			// clear master interrupt flag
	IFS0bits.SI2CIF=0;			// clear slave interrupt flag
	IPC3bits.MI2CIP=5;			// priority level
//	IEC0bits.MI2CIE=1;			// enabhle master I2C interrupt
	IEC0bits.SI2CIE=0;			// diseble slave I2C interrupt


	for(i=10000;i;i--);
	return 1;
}

char e_i2c_deinit(void) {
	I2CCONbits.I2CEN=0;			// disable I2C
	IFS0bits.MI2CIF=0;			// clear master interrupt flag
	IFS0bits.SI2CIF=0;			// clear slave interrupt flag
        IEC0bits.MI2CIE=0;			// disable master I2C interrupt
	IEC0bits.SI2CIE=0;			// diseble slave I2C interrupt
        return 1;
}

/*! \brief Reset the microcontroller for I2C uses
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_reset(void)
{
	long i=0;
	I2CCONbits.I2CEN=0;			// disable I2C and stop peripheric
	IFS0bits.MI2CIF=0;			//
	IFS0bits.SI2CIF=0;			//
	IEC0bits.SI2CIE=0;			//
        I2CSTAT = 0;
	for(i=10000;i;i--);

	e_i2c_init();				// intit I2C
	e_i2c_enable();				//enable interrupt

	for(i=10000;i;i--);

	return 1;
}

/*! \brief Enable special I2C interrupt
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_enable(void)
{
/*	e_interrupts[0]=IEC0;
	IEC0=0;
	e_interrupts[1]=IEC1;
	IEC1=0;
	e_interrupts[2]=IEC2;
	IEC2=0;	*/

	IFS0bits.MI2CIF=0;			// clear master interrupt flag
	IEC0bits.MI2CIE=1;			// enable master I2C interrupt
	return 1;
}

/*! \brief Disable special I2C interrupt
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_disable(void)
{

/*	IEC0=e_interrupts[0];
	IEC1=e_interrupts[1];
	IEC2=e_interrupts[2];*/
	IFS0bits.MI2CIF=0;			// clear master interrupt flag
	IEC0bits.MI2CIE=0;			// disable master I2C interrupt
	return 1;
}

/*! \brief Make the start bit
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_start(void)
{
	long i;
	e_i2c_mode=START;
	if(I2CSTATbits.P)
	{
		I2CCONbits.SEN=1;
		for(i=10000;i;i--)
			if(!e_i2c_mode)
				return 1;
		return 0;
	}else
		return 0;
}

/*! \brief Make the restart bit
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_restart(void)
{
	long i;
	e_i2c_mode=RESTART;
	if(I2CSTATbits.S)
	{
		I2CCONbits.RSEN=1;
		for(i=10000;i;i--)
			if(!e_i2c_mode)
				return 1;
		return 0;
	}else
		return 0;
}

/*! \brief Make the stop bit
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_stop(void)
{
	long i;
	e_i2c_mode=STOP;

		I2CCONbits.PEN=1;

	for(i=10000;i;i--)
		if(!e_i2c_mode)
			return 1;
	return 0;
}

/*! \brief Make the acknowledgement bit
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_ack(void)
{
	long i;
	e_i2c_mode=ACKNOWLEDGE;

	// make sure I2C bus is inactive
    if(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN || I2CCONbits.ACKEN || I2CCONbits.RSEN)
		return 0;

	// set ACK mode
	I2CCONbits.ACKDT=0;
	I2CCONbits.ACKEN=1;

	for(i=10000;i;i--)
			if(!e_i2c_mode)
				return 1;
	return 0;
}

/*! \brief Make the non-acknowledgement bit
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_nack(void)
{
	long i;
	e_i2c_mode=ACKNOWLEDGE;

	// make sure I2C bus is inactive
    if(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN || I2CCONbits.ACKEN || I2CCONbits.RSEN)
		return 0;

	// set NACK mode
	I2CCONbits.ACKDT=1;
	I2CCONbits.ACKEN=1;

	for(i=10000;i;i--)
		if(!e_i2c_mode)
			return 1;
	return 0;
}

/*! \brief Read the I2C input register
 * \param buf A pointer to store the datas received
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_read(char *buf)
{
	long i=10000;
	char read_ok=0;
//	int	test=0;
	e_i2c_mode=READ;

	for(i=10000;i;i--)
		if(!(I2CCONbits.SEN || I2CCONbits.PEN || I2CCONbits.RCEN || I2CCONbits.ACKEN || I2CSTATbits.TRSTAT))
		{
			read_ok=1;
			break;
		}
	if(!read_ok)
		return 0;

	// start receive mode for I2C
	I2CCONbits.RCEN=1;

	// keep polling for I2C interrupt
	for(i=100000;i;i--)
		if(!e_i2c_mode)		// once I2C interrupt is tripped, read buffer and return 1
		{
//			test=I2CSTAT;	// used for debug purposes
			*buf=I2CRCV;
			return 1;
		}
	return 0;

}

/*! \brief Write on the I2C output register
 * \param byte What you want to send on I2C
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2c_write(char byte)
{
	long i;
	e_i2c_mode=WRITE;
	I2CTRN=byte;

	// poll for I2C interrupt
	for(i=10000;i;i--)
		if(!e_i2c_mode)	// return 1(transmisison OK) if interrupt was tripped)
			return 1;
	return 0;
}

// interrupt  routine:
void  __attribute__((__interrupt__, auto_psv)) _MI2CInterrupt(void)
{
	IFS0bits.MI2CIF=0;			// clear master interrupt flag
	e_i2c_mode=OPERATION_OK;
}
