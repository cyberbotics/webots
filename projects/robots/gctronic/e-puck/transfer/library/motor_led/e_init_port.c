/********************************************************************************

			Initialization of port of e-puck
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
 * \brief Initialize the ports on standard configuration.
 * \author Code: Michael Bonani, Francesco Mondada, Davis Dadie \n Doc: Jonathan Besuchet
 */

#include "e_epuck_ports.h"
#include "../I2C/e_I2C_protocol.h"
#include "../acc_gyro/e_lsm330.h"

unsigned char isPresentFlag = 0;

/* set configuration bit for MPLAB
!!!remove this macros if you have want different configuration bit!!!*/
_FOSC(CSW_FSCM_OFF & XT_PLL8);
_FWDT(WDT_OFF);
_FBORPOR(PBOR_OFF & MCLR_EN);
_FGS(CODE_PROT_OFF);

void testAccGyroPresence() {
    unsigned char ret;
    e_i2cp_init();
    e_i2c_reset();
    e_i2cp_enable();
    ret = e_i2cp_read(0x3C, 0x0F); // read WHO_AM_I register of accelerometer
    e_i2cp_disable();
    if (ret == 0x40) {
        isPresentFlag = 1;
    } else {
        isPresentFlag = 0;
    }
}

/*! \brief Initialize all ports (in/out)
 *
 * Call this method to set all the standards output
 * components (LEDs, IR, camera, motors, I2C, audio) on
 * their defaults values and set their corresponding PIN
 * to "output".
 * The method also set the corresponding PIN to "input"
 * for all the standards inputs components
 * (IR receiver, selector, camera, battery level).
 */
void e_init_port(void) {

    /********************** OUTPUTS ***************************/
    /*LEDS*/
    LED0 = 0;
    LED1 = 0;
    LED2 = 0;
    LED3 = 0;
    LED4 = 0;
    LED5 = 0;
    LED6 = 0;
    LED7 = 0;
    LED0_DIR = OUTPUT_PIN;
    LED1_DIR = OUTPUT_PIN;
    LED2_DIR = OUTPUT_PIN;
    LED3_DIR = OUTPUT_PIN;
    LED4_DIR = OUTPUT_PIN;
    LED5_DIR = OUTPUT_PIN;
    LED6_DIR = OUTPUT_PIN;
    LED7_DIR = OUTPUT_PIN;

    FRONT_LED = 0;
    FRONT_LED_DIR = OUTPUT_PIN;

    BODY_LED = 0;
    BODY_LED_DIR = OUTPUT_PIN;

    /*IR*/
    PULSE_IR0 = 0;
    PULSE_IR1 = 0;
    PULSE_IR2 = 0;
    PULSE_IR3 = 0;
    PULSE_IR0_DIR = OUTPUT_PIN;
    PULSE_IR1_DIR = OUTPUT_PIN;
    PULSE_IR2_DIR = OUTPUT_PIN;
    PULSE_IR3_DIR = OUTPUT_PIN;

    /*basic audio*/
    AUDIO_ON = 0; /*turn of speaker and codec*/
    AUDIO_ON_DIR = OUTPUT_PIN;

    /*motors*/
    MOTOR1_PHA = 0;
    MOTOR1_PHB = 0;
    MOTOR1_PHC = 0;
    MOTOR1_PHD = 0;
    MOTOR2_PHA = 0;
    MOTOR2_PHB = 0;
    MOTOR2_PHC = 0;
    MOTOR2_PHD = 0;
    MOTOR1_PHA_DIR = OUTPUT_PIN;
    MOTOR1_PHB_DIR = OUTPUT_PIN;
    MOTOR1_PHC_DIR = OUTPUT_PIN;
    MOTOR1_PHD_DIR = OUTPUT_PIN;
    MOTOR2_PHA_DIR = OUTPUT_PIN;
    MOTOR2_PHB_DIR = OUTPUT_PIN;
    MOTOR2_PHC_DIR = OUTPUT_PIN;
    MOTOR2_PHD_DIR = OUTPUT_PIN;

    /*camera*/
    CAM_RESET = 0;
    CAM_RESET_DIR = OUTPUT_PIN;

    /*I2C*/
    SIO_C = 0;
    SIO_D = 0;

    SIO_C_DIR = OUTPUT_PIN;
    SIO_D_DIR = OUTPUT_PIN;

    /********************** INPUTS **************************/

    /*low battery signal active low when Vbatt<3.4V*/
    BATT_LOW_DIR = INPUT_PIN;

    /*IR TV receiver on normal extension*/
    REMOTE_DIR = INPUT_PIN;

    /* selector*/
    SELECTOR0_DIR = INPUT_PIN;
    SELECTOR1_DIR = INPUT_PIN;
    SELECTOR2_DIR = INPUT_PIN;
    SELECTOR3_DIR = INPUT_PIN;

    /*camera*/
    CAM_y0_DIR = INPUT_PIN;
    CAM_y1_DIR = INPUT_PIN;
    CAM_y2_DIR = INPUT_PIN;
    CAM_y3_DIR = INPUT_PIN;
    CAM_y4_DIR = INPUT_PIN;
    CAM_y5_DIR = INPUT_PIN;
    CAM_y6_DIR = INPUT_PIN;
    CAM_y7_DIR = INPUT_PIN;

    testAccGyroPresence();
    if (isPresentFlag) {
        initAccAndGyro();
    }
}

unsigned char isEpuckVersion1_3(void) {
    return isPresentFlag;
}

