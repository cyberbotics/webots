/********************************************************************************

			Advance Analogic/Digital conversion
			december 2005: first version Francesco Mondada
			april 2006: debug and optimisation Michael Bonani
			Borter Jean-Joel


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005 Francesco Mondada
(c) 2006-2007 Michael-Bonani & Borter Jean-Joel

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup a_d
 * \brief Module for the advance Analogic/Digital conversion.
 *
 * The advance converter module is set to operate by itself. It uses the
 * ADC interrupt to launch the acquisitions. Then no timer is needed.
 * \n \n The data sampled are stored in the corresponding array:
 * - \ref e_mic_scan[3][MIC_SAMP_NB]       Array to store the mic values
 * - \ref e_acc_scan[3][ACC_SAMP_NB]       Array to store the acc values
 * - \ref e_ambient_ir[8]                  Array to store ambient light measurement
 * - \ref e_ambient_and_reflected_ir[8]    Array to store ambient and reflected light measurement
 *
 * In all the files of this module (e_acc.c, e_micro.c, e_prox.c), these
 * arrays are declared has "extern". In this way we can access the arrays
 * for exemple like this (function e_get_acc from e_acc.c): \n
 * \code
 * extern int e_acc_scan[3][ACC_SAMP_NB];
 *
 * int e_get_acc(unsigned int captor)
 * {
 * 	if (captor < 3)
 * 		return (e_acc_scan[captor][e_last_acc_scan_id]);
 * 	else
 * 		return((int)ANGLE_ERROR);
 * }
 * \endcode
 * \author Code: Francesco Mondada, Michael-Bonani & Borter Jean-Joel \n Doc: Jonathan Besuchet
 */

#ifndef _AD_CONV
#define _AD_CONV

	/////////////////////////////////////////////////////////////
	// WARNING:  This file is to be used with dsPIC30F6014A    //
	//                     with 8x PLL                         //
	/////////////////////////////////////////////////////////////

// sampling frequency for the microphones
#define MIC_SAMP_FREQ 16384.0	// WARNING: must be the highest one. This is the base to calculate ad_cycle
								//16384.0 is max could also use 12288


// sampling frequency for the accelerometres and proximetres
#define ACC_PROX_SAMP_FREQ 256.0	// WARNING: should be a fraction of MIC_SAMP_FREQ
									//			to ensure a good timing precision
// lenght of the IR pulse in seconds
#define PULSE_LENGHT 0.0003

//Calculation of the periodes in ad_cycle
//ad_cycle = 1/MIC_SAMP_FREQ
#define ACC_PROX_PERIOD (int)(MIC_SAMP_FREQ/ACC_PROX_SAMP_FREQ)	// (64) acc and prox periode in [ad_cycle]
#define PULSE_PERIOD (int)(PULSE_LENGHT*MIC_SAMP_FREQ)			// (5)  pulse length in [ad_cycle]

//ADCS calculation to allways have the same time for an AD conversion scan
//with a different numbers of channels
#define ADCS_3_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*3)-1)	// SAMPLETIME 3TAD
#define ADCS_5_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*5)-1)	//
#define ADCS_6_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*6)-1)	//
#define ADCS_2_CHAN	(int)(2.0*FCY/(MIC_SAMP_FREQ*(14+1)*2)-1)	//

#define MIC_SAMP_NB 100	// number of microphone samples to store (put 100 to use with sercom_adv, otherwise 256)
#define ACC_SAMP_NB  50	// number of accelerometer samples to store

#define MICRO_ONLY 1
#define ALL_ADC 0

#define ADC_ISR_PERIOD_MS 0.061 //(1000.0/MIC_SAMP_FREQ)

void e_init_ad_scan(unsigned char only_micro);
unsigned char e_ad_is_array_filled(void);
unsigned char e_ad_is_acquisition_completed(void);
void e_ad_scan_on(void);
void e_ad_scan_off(void);

#endif /*_AD_CONV*/

/* End of File : ad_conv.h */
