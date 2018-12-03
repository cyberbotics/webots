/**************************************************************************
* 			Defintition of all port of the e-puck 	 	                  *
*			Version 1.0 november 2005			                          *
*			Michael Bonani, Francesco Mondada, Davis Dadie                *
*									                                      *
**************************************************************************/
/********************************************************************************

			Defintition of all port of the e-puck
			Version 1.0 november 2005
			Michael Bonani, Francesco Mondada, Davis Dadie


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Michael Bonani, Francesco Mondada, Davis Dadie

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Define all the usefull names corresponding of e-puck's hardware.
 * \author Code: Michael Bonani, Francesco Mondada, Davis Dadie \n Doc: Jonathan Besuchet
 */

/*! \mainpage e-puck standard library documentation
 * \image html logo.gif
 * \section intro_sec Introduction
 * This project has been started at the Ecole Polytechnique Federale de Lausanne as
 * collaboration between the Autonomous Systems Lab, the Swarm-Intelligent Systems
 * group and the Laboratory of Intelligent System.
 * \n \n An educational robot:
 * The main goal of this project is to develop a miniature mobile robot for educational
 * purposes at university level. To achieve this goal the robot needs, in our opinion,
 * the following features:
 * - Good structure. The robot should have a clean mechanical structure, simple to
 * understand. The electronics, processor structure and software have to be a good
 * example of a clean modern system.
 * - Flexibility. The robot should cover a large spectrum of educational activities and
 * should therefore have a large potential in its sensors, processing power and
 * extensions. Potential educational fields are, for instance, mobile robotics,
 * real-time programming, embedded systems, signal processing, image or sound feature
 * extraction, human-machine interaction or collective systems.
 * - User friendly. The robot should be small and easy to exploit on a table next to a
 * computer. It should need minimal wiring, battery operation and optimal working comfort.
 * - Good robustness and simple maintenance. The robot should resist to student use
 * and be simple and cheap to repair.
 * - Cheap. The robot, for large use, should be cheap (450-550 euros)
 * \section brief_sec Documentation organization
 * This documentation is divided in five sections (as you can see on the top of the page):
 * - Main Page: The startup page.
 * - Modules: An overview of all the modules that compose this library. Here you can see
 * all the files containing by each module and a detailed description of each module. Look
 * at these pages to have a better idea of what each module is doing.
 * - Data Structures: Here are listed all the C-struct of the library.
 * - Files: All the library's files listed by alphabetical order.
 * - Directories: The directories architectures of the library.
 * \section link_sec External links
 * - http://www.e-puck.org/                 The official site of the e-puck
 * - https://gna.org/projects/e-puck/       The developers area at gna
 * - http://lsro.epfl.ch/                   The site of the lab where the e-puck has been created
 * - http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45 The license
 */

/*! \defgroup motor_LED Ports, motors and LEDs
 *
 * \section intro_sec_motors Introduction
 * This package contains all the resources you need to control the ports,
 * the motors, the LED and the IR receiver of the e-puck.
 *
 * \subsection intro_subsec1_ports Ports
 * The standard port's name of the p30F6014A microcontroller is not explicit in the
 * e-puck context, so we need to redefine these names to make them more user friendly.
 * \n This work is made in the file: e_epuck_ports.h.
 *
 * \subsection intro_subsec2_motors Motors
 * The e-puck has two step by step motors called MOTOR1 (left) and MOTOR2 (right).
 * To control the changing phase's sequence of these motors we need to use timers.
 * Four possibilities are offered to you:
 * - standard: we use the timer4 for MOTOR2 and timer5 for MOTOR1. This solution is
 *   exploited by the file library\motor_led\e_motors.c.
 * - one timer standard: we use the timer3 for both MOTOR1 and MOTOR2. This solution
 *   is exploited by the file library\motor_led\e_motors_timer3.c
 * - advance one timer: we use the timer2 for both MOTOR1 and MOTOR2, but this time
 *   the mechanism work on the agenda method (see below or e_agenda.h
 *   for more information about agenda). This solution is exploited by the file
 *   library\motor_led\advance_one_timer\e_motors.c.
 * - fast agenda: we use the timer1,2,3 for both MOTOR1 and MOTOR2, but this time
 *   the mechanism work on the fast_agenda method (see below or e_agenda_fast.h
 *   for more information about fast_agenda). This solution is exploited by the file
 *   library\motor_led\advance_one_timer\fast_agenda\e_motors.c.
 *
 * \subsection intro_subsec3_led LED
 * The e-puck has 8 reds LEDs, a front LED and a body LED. All the functions needed
 * to control these LEDs are in the file library\motor_led\e_led.c. This file is
 * made for basics use. If you want blinking functions
 * you have to work with these following files: library\motor_led\advance_one_timer\e_led.c
 * or library\motor_led\advance_one_timer\fast_agenda\e_led.c. In the case you will
 * work with agenda solution (see below or e_agenda.h or e_agenda_fast.h for more
 * information about agenda or fast agenda).
 *
 * \subsection intro_subsec4_ir IR remote
 * The e-puck has a IR receptor. To control this receptor look at this file:
 * library\motor_led\advance_one_timer\e_remote_control.c.
 * \warning The IR remote uses the agenda solution, then it use timer2 (see below or
 * e_agenda.h for more information about agenda).
 *
 * \section timer_sect_timer Timer's problems
 * The p30F6014A microcontroller has five timers. The camera's package uses the
 * timer4 and the timer5, so we can't exploit them to make the motors work when we
 * want to use the camera. For this reason we can't use the standard solution above.
 * \warning If you are using the camera, you have to work with one of this three
 * solutions explained above:
 * - one timer standard
 * - advance one timer
 * - fast agenda
 *
 * \section agenda_sect_agenda Agenda solution
 * As we have seen, we can use the agenda solution to make the motors work.
 * \n \n So what is an agenda ?
 * \n An agenda is a structure which can launch a specific function (called callback
 * function) with a given intervals. The agenda structure is made to work as
 * chained list.
 * \n \n How it works ?
 * \n You create an agenda by specifying:
 * - the callback function you will call
 * - the delay between two calls
 * - the next element of the chained list
 *
 * On each timer overflow all the agenda chained list is scanned and if an agenda
 * in the list has reached the delay, then the callback function is called.
 * \sa e_agenda.c, e_agenda_fast.c
 *
 * \author Doc: Jonathan Besuchet
 */

#ifndef _EPUCK_PORTS
#define _EPUCK_PORTS

#include "p30F6014A.h"

/*********************GENERAL SETUP************************/

#define FOSC   7.3728e6     // 7.3728Mhz crystal in XTL mode
#define PLL    8.0       	// 8x PLL


#define FCY     ((FOSC*PLL)/(4.0))	// Instruction cycle frequency
#define MILLISEC  (FCY/1.0e3)		// 1mSec delay constant
#define MICROSEC  (FCY/1.0e6)		// 1uSec delay constant
#define NANOSEC   (FCY/1.0e9)		// 1nSec delay constant

#define	TCY_PIC		(1e9/FCY)		//time instruction cycle in [ns]
#define	INTERRUPT_DELAY	(10*TCY_PIC)//delay to start an interrupt in [ns] (observe with p30f6014)

#define TRUE	1
#define FALSE	0


/********************** OUTPUTS ***************************/
#define OUTPUT_PIN 0
/*LEDS*/
/*First in front of robot than turning clokwise*/
#define LED0 _LATA6
#define LED1 _LATA7
#define LED2 _LATA9
#define LED3 _LATA12
#define LED4 _LATA10
#define LED5 _LATA13
#define LED6 _LATA14
#define LED7 _LATA15

#define LED0_DIR _TRISA6
#define LED1_DIR _TRISA7
#define LED2_DIR _TRISA9
#define LED3_DIR _TRISA12
#define LED4_DIR _TRISA10
#define LED5_DIR _TRISA13
#define LED6_DIR _TRISA14
#define LED7_DIR _TRISA15

#define FRONT_LED _LATC1
#define FRONT_LED_DIR _TRISC1

#define BODY_LED _LATC2
#define BODY_LED_DIR _TRISC2

/*IR*/
#define PULSE_IR0 _LATF7		// pulse IR 0 and 4
#define PULSE_IR1 _LATF8		// pulse IR 1 and 5
#define PULSE_IR2 _LATG0		// pulse IR 2 and 6
#define PULSE_IR3 _LATG1		// pulse IR 3 and 7

#define PULSE_IR0_DIR _TRISF7
#define PULSE_IR1_DIR _TRISF8
#define PULSE_IR2_DIR _TRISG0
#define PULSE_IR3_DIR _TRISG1

/*First in front right of robot than turning clokwise*/
#define IR0 8  // ir proximity sensor 0 on AD channel 8
#define IR1 9  // ir proximity sensor 1 on AD channel 9
#define IR2 10  // ir proximity sensor 2 on AD channel 10
#define IR3 11  // ir proximity sensor 3 on AD channel 11
#define IR4 12  // ir proximity sensor 4 on AD channel 12
#define IR5 13  // ir proximity sensor 5 on AD channel 13
#define IR6 14  // ir proximity sensor 6 on AD channel 14
#define IR7 15  // ir proximity sensor 7 on AD channel 15


/*analog*/
#define MIC1 2  // microphone 1 on AD channel 2
#define MIC2 3  // microphone 2 on AD channel 3
#define MIC3 4  // microphone 3 on AD channel 4


#define ACCX 5  // X Axis of accelerometer on AD channel 5
#define ACCY 6  // Y Axis of accelerometer on AD channel 6
#define ACCZ 7  // Z Axis of accelerometer on AD channel 7


/*basic audio*/
#define AUDIO_ON _LATF0
#define AUDIO_ON_DIR _TRISF0

/*motors*/
#define MOTOR1_PHA _LATD0
#define MOTOR1_PHB _LATD1
#define MOTOR1_PHC _LATD2
#define MOTOR1_PHD _LATD3
#define MOTOR2_PHA _LATD4
#define MOTOR2_PHB _LATD5
#define MOTOR2_PHC _LATD6
#define MOTOR2_PHD _LATD7

#define MOTOR1_PHA_DIR _TRISD0
#define MOTOR1_PHB_DIR _TRISD1
#define MOTOR1_PHC_DIR _TRISD2
#define MOTOR1_PHD_DIR _TRISD3
#define MOTOR2_PHA_DIR _TRISD4
#define MOTOR2_PHB_DIR _TRISD5
#define MOTOR2_PHC_DIR _TRISD6
#define MOTOR2_PHD_DIR _TRISD7

/*camera*/
#define CAM_RESET _LATC13
#define CAM_RESET_DIR _TRISC13

/* I2C */
#define SIO_D	_LATG3
#define SIO_D_DIR	_TRISG3

#define SIO_C	_LATG2
#define SIO_C_DIR	_TRISG2

/********************** INPUTS **************************/
#define INPUT_PIN 1

/*low battery signal active low when Vbatt<3.4V*/
#define BATT_LOW _RF1
#define BATT_LOW_DIR _TRISF1

/* selector on normal extension*/
#define SELECTOR0 _RG6
#define SELECTOR1 _RG7
#define SELECTOR2 _RG8
#define SELECTOR3 _RG9

#define SELECTOR0_DIR _TRISG6
#define SELECTOR1_DIR _TRISG7
#define SELECTOR2_DIR _TRISG8
#define SELECTOR3_DIR _TRISG9

/*IR TV receiver on normal extension*/
#define REMOTE _RF6
#define REMOTE_DIR _TRISF6

/*CAMERA*/
/*data higher 8 bits of port D*/
#define CAM_DATA PORTD;

#define CAM_y0 _RD8
#define CAM_y1 _RD9
#define CAM_y2 _RD10
#define CAM_y3 _RD11
#define CAM_y4 _RD12
#define CAM_y5 _RD13
#define CAM_y6 _RD14
#define CAM_y7 _RD15

#define CAM_y0_DIR _TRISD8
#define CAM_y1_DIR _TRISD9
#define CAM_y2_DIR _TRISD10
#define CAM_y3_DIR _TRISD11
#define CAM_y4_DIR _TRISD12
#define CAM_y5_DIR _TRISD13
#define CAM_y6_DIR _TRISD14
#define CAM_y7_DIR _TRISD15

/*clock interupt*/
#define CAM_PWDN _RC2
#define CAM_VSYNC _RC4
#define CAM_HREF _RC3
#define CAM_PCLK _RC14

#define CAM_PWDN_DIR _TRISC2
#define CAM_VSYNC_DIR _TRISC4
#define CAM_HREF_DIR _TRISC3
#define CAM_PCLK_DIR _TRISC14

/*********************** ASEMBLER SMALL FUNCTCION********************** */
#define NOP() {__asm__ volatile ("nop");}
#define CLRWDT() {__asm__ volatile ("clrwdt");}
#define SLEEP() {__asm__ volatile ("pwrsav #0");}
#define IDLE() {__asm__ volatile ("pwrsav #1");}
#define INTERRUPT_OFF() {__asm__ volatile ("disi	#10000");}	//disable interrupts with priority 0-6 for 10000 cycles
#define INTERRUPT_ON() {__asm__ volatile ("disi	#2");}
#define RESET() {__asm__ volatile ("reset");}


#define STOP_TMR1 IEC0bits.T1IE = 0
#define STOP_TMR2 IEC0bits.T2IE = 0
#define STOP_TMR3 IEC0bits.T3IE = 0
#define STOP_TMR4 IEC1bits.T4IE = 0
#define STOP_TMR5 IEC1bits.T5IE = 0

#endif
