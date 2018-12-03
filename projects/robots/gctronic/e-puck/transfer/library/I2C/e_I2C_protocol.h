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

#ifndef _I2C_PROTOCOL
#define _I2C_PROTOCOL

#include "e_I2C_master_module.h"
#include "../motor_led/e_epuck_ports.h"


//public interface

// use void e_i2cp_init(void); in your initialisation. no interrupt is enable
// use void e_i2cp_enable(void); before any other operation to enable the interrupts
// use e_i2cp_write,I2Cp_read to write or read in the Camera registers

void e_i2cp_init(void);
void e_i2cp_deinit(void);
char e_i2cp_write (char device_add,char reg, char value);
char e_i2cp_read(char device_add,char reg);
char e_i2cp_read_string(char device_add, unsigned char read_buffer[], char start_address, char string_length);
char e_i2cp_write_string (char device_add, unsigned char write_buffer[], char start_address, char string_length);
void e_i2cp_enable(void);
void e_i2cp_disable(void);

#endif
