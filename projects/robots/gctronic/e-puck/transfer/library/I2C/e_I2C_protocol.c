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
 * \brief Manage I2C protocole.
 *
 * This module manage the I2C protocole. The function's module are
 * made to directly send or receive data from or to a specified slave.
 * \warning This file must be include to communicate with an PO3030K
 * camera throught the I2C communication protocol
 * \author Code: Davis Daidie \n Doc: Jonathan Besuchet
 */

#include "e_I2C_protocol.h"

/*! \brief Initialize the microcontroller for I2C uses
 * \return 1 to confirme the oparation and 0 for an error
 */
void e_i2cp_init(void)
{
	e_i2c_init();
}

void e_i2cp_deinit(void)
{
	e_i2c_deinit();
}

/*! \brief Enable special I2C interrupt
 * \return 1 to confirme the oparation and 0 for an error
 */
void e_i2cp_enable(void)
{
	e_i2c_enable();
}

/*! \brief Disable special I2C interrupt
 * \return 1 to confirme the oparation and 0 for an error
 */
void e_i2cp_disable(void)
{
	e_i2c_disable();
}

/*! \brief Read a specific register on a device
 * \param device_add The address of the device you want information
 * \param reg The register address you want read on the device
 * \return The readed value
 */
char e_i2cp_read(char device_add, char reg)
{
	char error=0;
	char value;
	while(!error)
	{
		error=1;
		error&=e_i2c_start();
		error&=e_i2c_write(device_add);    	// Device address
		error&=e_i2c_write(reg);     		// Register address

		error&=e_i2c_restart();
		error&=e_i2c_write(device_add+1);   // To change data direction ([bit 0]=1)
 		error&=e_i2c_read(&value);    		// read single byte
		e_i2c_nack();						// only 1 byte is being read, so send nack
		e_i2c_stop();             			// end read cycle
		if(error)
			break;
		e_i2c_reset();
	}

   	return value;
}

// ATTENTION: This function doesn't work
/*
char e_i2cp_read_string(char device_add, unsigned char read_buffer[], char start_address, char string_length)
{
	char error=0;
	char i = 0;
	while(!error)
	{
		error=1;
		error&=e_i2c_start();
		error&=e_i2c_write(device_add);    		// Device address
		error&=e_i2c_write(start_address);		// address of first register to be read
		error&=e_i2c_restart();
		error&=e_i2c_write(device_add+1);    	// To change data direction ([bit 0]=1)

		for (i=0;i < string_length;i++)
		{
//	 		error&=e_i2c_read(&read_buffer[i]);  // read the next byte
			if (i == (string_length-1))			// the last byte to be read, must send nack
				error&=e_i2c_nack();
			else
				error&=e_i2c_ack();				// not the last byte, send ack
		}
		e_i2c_stop();             				// End read cycle
		if(error)
			break;
		e_i2c_reset();
	}
	return error;
}
*/

/*! \brief Write a specific register on a device
 * \param device_add The address of the device you want information
 * \param reg The register address you want read on the device
 * \param value The data you want to write
 * \return 1 to confirme the oparation and 0 for an error
 */
char e_i2cp_write (char device_add, char reg, char value)
{
	char error=0;

	while(!error)
	{
		error=1;
		error&=e_i2c_start();
		error&=e_i2c_write(device_add);		// Writing the device (slave) address
		error&=e_i2c_write(reg);			// Device register address
		error&=e_i2c_write(value);			// Data to device
		error&=e_i2c_stop();				// Ending the communication
		if(error)
			break;
		e_i2c_reset();
	}
	return error;
}

//BOF: ne marche surement pas
/*
char e_i2cp_write_string (char device_add, unsigned char write_buffer[], char start_address, char string_length)
{
	char error=0;
	int i = 0;

	while(!error)
	{
		error=1;
		error&=e_i2c_start();
		error&=e_i2c_write(device_add);    // Writing the device (slave) I2C address
		error&=e_i2c_write(start_address);     		// Device register address
		for (i=0;i<string_length;i++)
			error&=e_i2c_write(write_buffer[i]);       // Data to device
		error&=e_i2c_stop();             // Ending the communication
		if(error)
			break;
//		e_i2c_reset();
	}
	return error;
}
*/
