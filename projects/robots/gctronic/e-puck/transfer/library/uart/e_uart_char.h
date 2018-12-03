/********************************************************************************

			UART module
			December 2004: first version Michael Bonani


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Michael Bonani

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup uart
 * \brief Manage UART.
 *
 * This module manage all the UART ressource.
 * \n The e-puck's microcontroller has two integreted UART:
 * UART1 and UART2.
 *
 * A little exemple to comunicate with the e-puck through the uart.
 * For this exemple you have to connect your e-puck to your PC with
 * bluetooth (if you don't know how it works, look at the end of page 3
 * of this doc: http://moodle.epfl.ch/mod/resource/view.php?id=12851).
 * Then open the HyperTerminal with the correct port COM
 * and launch the connection. "Give a character:" should appears on the
 * HyperTerminal.
 * \code
 * #include <motor_led/e_init_port.h>
 * #include <uart/e_uart_char.h>
 *
 * int main(void)
 * {
 * 	char car;
 * 	e_init_port();
 * 	e_init_uart1();
 * 	e_send_uart1_char("\f\a", 2);		//new page on HyperTerminal
 * 	while(1)
 * 	{
 * 		e_send_uart1_char("Give a character:\r\n", 19);
 *		// do nothing while the text is not sent and while nothing is comming from the user
 * 		while(e_uart1_sending() || !e_ischar_uart1()) {}
 * 		e_getchar_uart1(&car);			// read the character entered...
 * 		e_send_uart1_char("You have wrote: ", 16);
 * 		e_send_uart1_char(&car, 1);		//... and resend him to uart.
 * 		e_send_uart1_char("\r\n\r\n",4);
 * 	}
 * }
 * \endcode
 * \author Code: Michael Bonani \n Doc: Jonathan Besuchet
 */

/*! \defgroup uart UART
 *
 * \section intro_sec Introduction
 * This package contains all the ressources you need to control the UART
 * (universal asynchronous receiver transmitter). The microcontroller p30f6014A
 * has two UART controller: UART1 and UART2.
 * \warning In this package, the functions are written in ASM. We have "e_init_uartX.s" file
 * for the initializing functions, "e_uartX_rx.s" file for receiving data functions
 * and "e_uartX_tx.s" file for transmitting data functions (X can be 1 or 2).
 * \n Even these files are written in ASM, you can call them by including the e_uart_char.h
 * files in your C code (look at the exemple in e_uart_char.h).
 * \section bluettothUart_sec Bluetooth
 * The e-puck has his bluetooth module connected on the uart1. Two ways are possibles when you
 * work with bluetooth:
 * - you are the master, look at this: \ref bluetooth
 * - you are the slave.
 *
 * When you are the slave, you can communicate with the master device exactely by the same way
 * as you do to communicate through the uart. The bluetooth protocole is made to look like as
 * transparent as possible. This is possible because the master initialize the communication
 * and the bluetooth module answer automatically to create the connection. After that the
 * connection was created, you can communicate with the master by using the uart protocole.
 * \author Doc: Jonathan Besuchet
 */

#ifndef _UART_TXRX_CHAR
#define _UART_TXRX_CHAR

#define BAUD115200 7
#define BAUD230400 3
#define BAUD460800 1
#define BAUD921600 0

/*! \brief Init uart 1 at 115200bps, 8 data bits, 1 stop bit, Enable ISR for RX and TX */
void e_init_uart1(void);

/*! \brief Check if something is comming on uart 1
 * \return the number of characters available, 0 if none are available */
int  e_ischar_uart1();

/*! \brief If available, read 1 char and put it in pointer
 * \param car The pointer where the caracter will be stored if available
 * \return 1 if a char has been readed, 0 if no char is available
 */
int  e_getchar_uart1(char *car);

/*! \brief Send a buffer of char of size length
 * \param buff The top of the array where the data are stored
 * \param length The length of the array to send
 */
void e_send_uart1_char(const char * buff, int length);

/*! \brief  To check if the sending operation is done
 * \return 1 if buffer sending is in progress, return 0 if not
 */
int  e_uart1_sending(void);


/*! \brief Init uart 2 at 115200bps, 8 data bits, 1 stop bit, Enable ISR for RX and TX */
void e_init_uart2(int);

/*! \brief Check if something is comming on uart 2
 * \return the number of characters available, 0 if none are available */
int  e_ischar_uart2();

/*! \brief If available, read 1 char and put it in pointer
 * \param car The pointer where the caracter will be stored if available
 * \return 1 if a char has been readed, 0 if no char is available
 */
int  e_getchar_uart2(char *car);

/*! \brief Send a buffer of char of size length
 * \param buff The top of the array where the datas are stored
 * \param length The length of the array
 */
void e_send_uart2_char(const char * buff, int length);

/*! \brief  To check if the sending operation is done
 * \return 1 if buffer sending is in progress, return 0 if not
 */
int  e_uart2_sending(void);


extern void *e_uart1_int_clr_addr; //address to be clear on interrupt
extern int e_uart1_int_clr_mask; //mask to be use to clear on interrupt
extern void *e_uart2_int_clr_addr; //address to be clear on interrupt
extern int e_uart2_int_clr_mask; //mask to be use to clear on interrupt


#endif
