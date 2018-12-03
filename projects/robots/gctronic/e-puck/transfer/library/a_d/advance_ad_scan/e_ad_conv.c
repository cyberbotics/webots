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
 * \author Code: Francesco Mondada, Michael-Bonani & Borter Jean-Joel \n Doc: Jonathan Besuchet
 */

#include "./../../motor_led/e_epuck_ports.h"
#include "e_ad_conv.h"
//#include "./../../fft/e_fft.h"
#include "../../utility/utility.h"
#include "../../motor_led/e_init_port.h"

int e_mic_scan[3][MIC_SAMP_NB];			/*!< Array to store the mic values */
int e_acc_scan[3][ACC_SAMP_NB];			/*!< Array to store the acc values */
unsigned int e_last_mic_scan_id = 0;	//ID of the last scan in the mic array (must be int else probleme of overchange)
unsigned int e_last_acc_scan_id = 0;	//ID of the last scan in the acc array

int e_ambient_ir[10];					/*!< Array to store the ambient light measurement */
int e_ambient_and_reflected_ir[10];		/*!< Array to store the light when IR led is on */

static unsigned char is_ad_acquisition_completed = 0;
static unsigned char is_ad_array_filled = 0;
static unsigned char micro_only = 0;
int selector;

unsigned char updateAccI2CCounter = 0;
extern int centre_z;
unsigned int tickAdcIsr = 0;    // tick resolution = ADC_ISR_PERIOD = 1/16384 = about 61 us

/*! \brief Initialize all the A/D register needed
 *
 * Set up the different ADC register to process the AD conversion
 * by scanning the used AD channels. Each value of the channels will
 * be stored in a different AD buffer register and an inturrupt will
 * occure at the end of the scan.
 * \param only_micro Put MICRO_ONLY to use only the three microphones
 * at 33kHz. Put ALL_ADC to use all the stuff using the ADC.
 */
void e_init_ad_scan(unsigned char only_micro)
{
	selector = getselector();

	if(only_micro == MICRO_ONLY)
		micro_only = MICRO_ONLY;
	else
		micro_only = ALL_ADC;

	ADCON1 = 0;						//reset to default value
	ADCON2 = 0;						//reset to default value
	ADCON3 = 0;						//reset to default value
	ADCHS = 0;						//reset to default value

	// ADPCFGbits.PCFGx
	// = 0 for Analog input mode,
	// = 1 for digital input mode (default)
	if(selector == 10) {	// gumstix extension
        IPC2bits.ADIP = 6;  // Give high priority to the ADC interrupt in order to be able to correctly read the sampled values of the
                            // additionals proximities, otherwise their values could be overwritten (before actually be read) if another
                            // higher priority interrupt is raised, since in the meanwhile the ADC continues sampling/converting and filling
                            // the buffer starting from the locations related to the additionals proximities.
		ADPCFGbits.PCFG0 = 0;   // ir9, right
		ADPCFGbits.PCFG1 = 0;   // ir8, left
	} else {
		ADPCFGbits.PCFG0 = 1;   // PGD
		ADPCFGbits.PCFG1 = 1;   // PGC
	}
	ADPCFGbits.PCFG2 = 0;   // micro 0
	ADPCFGbits.PCFG3 = 0;   // micro 1
	ADPCFGbits.PCFG4 = 0;   // micro 2
	ADPCFGbits.PCFG5 = 0;   // axe x acc
	ADPCFGbits.PCFG6 = 0;   // axe y acc
	ADPCFGbits.PCFG7 = 0;   // axe z acc
	ADPCFGbits.PCFG8 = 0;   // ir0
	ADPCFGbits.PCFG9 = 0;   // ir1
	ADPCFGbits.PCFG10 = 0;  // ir2
	ADPCFGbits.PCFG11 = 0;  // ir3
	ADPCFGbits.PCFG12 = 0;  // ir4
	ADPCFGbits.PCFG13 = 0;  // ir5
	ADPCFGbits.PCFG14 = 0;  // ir6
	ADPCFGbits.PCFG15 = 0;  // ir7

	//specifie the channels to be scanned
	ADCSSLbits.CSSL0 = 0;   // Debugger or ir9 on gumstix ext.
	ADCSSLbits.CSSL1 = 0;   // Debugger or ir8 on gumstix ext.
	ADCSSLbits.CSSL2 = 1;   // micro 0
	ADCSSLbits.CSSL3 = 1;   // micro 1
	ADCSSLbits.CSSL4 = 1;   // micro 2
	ADCSSLbits.CSSL5 = 0;   // axe x acc
	ADCSSLbits.CSSL6 = 0;   // axe y acc
	ADCSSLbits.CSSL7 = 0;   // axe z acc
	ADCSSLbits.CSSL8 = 0;   // ir0
	ADCSSLbits.CSSL9 = 0;   // ir1
	ADCSSLbits.CSSL10 = 0;  // ir2
	ADCSSLbits.CSSL11 = 0;  // ir3
	ADCSSLbits.CSSL12 = 0;  // ir4
	ADCSSLbits.CSSL13 = 0;  // ir5
	ADCSSLbits.CSSL14 = 0;  // ir6
	ADCSSLbits.CSSL15 = 0;  // ir7

	ADCON1bits.FORM = 0;	//output = unsigned int
	ADCON1bits.ASAM = 1;	//automatic sampling on
	ADCON1bits.SSRC = 7;	//automatic convertion mode

	ADCON2bits.SMPI = 3-1;	//interupt on 3rd sample
	ADCON2bits.CSCNA = 1;	//scan channel input mode on

	ADCON3bits.SAMC = 1;	//number of cycle between acquisition and conversion (need 2 for the prox)
	if(micro_only)
		ADCON3bits.ADCS = 19;			//acquisition only for micro = 33kHz
	else
		ADCON3bits.ADCS = ADCS_3_CHAN;	//Tad = (ADCS + SAMC) * Tcy/2 = 2170[ns],
										//WARNING: Tad min must be 667 [ns]

	IFS0bits.ADIF = 0;		//Clear the A/D interrupt flag bit
	IEC0bits.ADIE = 1;		//Set the A/D interrupt enable bit

	ADCON1bits.ADON = 1;	//enable AD conversion

	//wait for the array to be filled one time
	if(!micro_only)
		do
		{
			NOP();
		}
		while (e_last_acc_scan_id < ACC_SAMP_NB-1);

	if(isEpuckVersion1_3()) {
		centre_z = 0;
	}

}


/*! \brief Save the AD buffer registers in differents arrays */
void __attribute__((__interrupt__, auto_psv)) _ADCInterrupt(void)
{
	is_ad_acquisition_completed = 0;
	is_ad_array_filled = 0;

	volatile unsigned int * adc_ptr;
	static unsigned char period_counter = 0;	// period counter for accelerometres and proximetres
	static unsigned char prox_number = 0;		// number goes from 0 to 3 (4 couples of sensors)

	//Clear the A/D Interrupt flag bit or else the CPU will
	//keep vectoring back to the ISR
	IFS0bits.ADIF = 0;

	if(updateAccI2CCounter < 64) {  // 64*1/16384 = about 3.9 ms (256 Hz)
		updateAccI2CCounter++;      // tell when the i2c accelerometer (e-puck rev 1.3) need an update
	}

	if(tickAdcIsr < 65535) {
		tickAdcIsr++;   // max measurable time is 1/16384*65536 = 4 seconds
	}

	if(micro_only)
	{
		//////////////////////////////////////
		//  Copy of the buffer regs in the  //
		//  approprieted array              //
		//////////////////////////////////////
		adc_ptr = &ADCBUF0;

		e_mic_scan[0][e_last_mic_scan_id] = *adc_ptr++;
		e_mic_scan[1][e_last_mic_scan_id] = *adc_ptr++;
		e_mic_scan[2][e_last_mic_scan_id] = *adc_ptr++;

		if (++e_last_mic_scan_id >= MIC_SAMP_NB) {
			e_last_mic_scan_id = 0;
			is_ad_array_filled = 1; 	 // indicate that the array is filled
		}
	}
	else
	{
		//Disable the AD converter in order to be able to modify its behevior.
		ADCON1bits.ADON = 0;

		////////////////////////////////////
		// Configure AD reg for next scan //
		////////////////////////////////////

		// mic channels are always selected for next scan
		ADCSSL = 0x001C;		// micro selected
		ADCON2bits.SMPI = 3-1;	//interupt on 3th sample
		ADCON3bits.ADCS = ADCS_3_CHAN;

		// mic + acc
		if (period_counter == ACC_PROX_PERIOD-3) // cycle before last cycle of the periode
		{
		// acc channels selection
			ADCSSL=0x00FC;	// mic + acc selected
			ADCON2bits.SMPI = 6-1;	//interupt on 6th sample
			ADCON3bits.ADCS = ADCS_6_CHAN;
		}
		// mic + prox
		else if (period_counter == ACC_PROX_PERIOD-2) // cycle before last cycle of the periode
		{
			ADCON2bits.SMPI = 5-1;	//interupt on 5th sample
			ADCON3bits.ADCS = ADCS_5_CHAN;// prox channels selection

			switch (prox_number)
			{
				// ir sensors 0 and 4
				case 0:	ADCSSL=0x111C;
						break;
				// ir sensors 1 and 5
				case 1:	ADCSSL=0x221C;
						break;
				// ir sensors 2 and 6
				case 2:	ADCSSL=0x441C;
						break;
				// ir sensors 3 and 7
				case 3:	ADCSSL=0x881C;
						break;
				// gumstix additional ir sensors 8 and 9
				case 4: ADCSSL=0x001F;
						break;
			}
		}
		// micro + prox
		else if (period_counter == PULSE_PERIOD-2) // cycle before last cycle of the periode
		{
			ADCON2bits.SMPI = 5-1;	//interupt on 5th sample
			ADCON3bits.ADCS = ADCS_5_CHAN;// prox channels selection

			switch (prox_number)
			{
				// ir sensors 0 and 4
				case 0:	ADCSSL=0x111C;
						break;
				// ir sensors 1 and 5
				case 1:	ADCSSL=0x221C;
						break;
				// ir sensors 2 and 6
				case 2:	ADCSSL=0x441C;
						break;
				// ir sensors 3 and 7
				case 3:	ADCSSL=0x881C;
						break;
				// gumstix additional ir sensors 8 and 9
				case 4: ADCSSL=0x001F;
						break;
			}
		}

		//reenable the AD converter
		ADCON1bits.ADON = 1;

		//////////////////////////////////////
		//  Copy of the buffer regs in the  //
		//        approprieted array        //
		//////////////////////////////////////
		adc_ptr = &ADCBUF0;

        if(	!((period_counter==ACC_PROX_PERIOD-1) && (prox_number==4)) &&
            !((period_counter==PULSE_PERIOD-1) && (prox_number==4))) { // the 2 gumstix ir sensors are sampled before the mic, so in that case the mic will be saved later.
            //mic channels are always copied
            e_mic_scan[0][e_last_mic_scan_id] = *adc_ptr++;
            e_mic_scan[1][e_last_mic_scan_id] = *adc_ptr++;
            e_mic_scan[2][e_last_mic_scan_id] = *adc_ptr++;

            if(++e_last_mic_scan_id>=MIC_SAMP_NB) {
                e_last_mic_scan_id=0;
                is_ad_array_filled = 1; 	 // indicate that the array is filled
            }
        }

		if (period_counter == ACC_PROX_PERIOD-2)
		{
			if(++e_last_acc_scan_id>ACC_SAMP_NB-1)
				e_last_acc_scan_id=0;

			//acc channels copy
			e_acc_scan[0][e_last_acc_scan_id] = *adc_ptr++;
			e_acc_scan[1][e_last_acc_scan_id] = *adc_ptr++;
			e_acc_scan[2][e_last_acc_scan_id] = *adc_ptr;

		}
		else if(period_counter == ACC_PROX_PERIOD-1)
		{
			//prox channels copy (ambient)
			switch (prox_number)
			{
				// prox 0 and 4
				case 0: e_ambient_ir[0] = *adc_ptr++;
						e_ambient_ir[4] = *adc_ptr;
						PULSE_IR0 = 1;
						break;
				// prox 1 and 5
				case 1: e_ambient_ir[1] = *adc_ptr++;
						e_ambient_ir[5] = *adc_ptr;
						PULSE_IR1 = 1;
						break;
				// prox 2 and 6
				case 2: e_ambient_ir[2] = *adc_ptr++;
						e_ambient_ir[6] = *adc_ptr;
						PULSE_IR2 = 1;
						break;
				// prox 3 and 7
				case 3: e_ambient_ir[3] = *adc_ptr++;
						e_ambient_ir[7] = *adc_ptr;
						PULSE_IR3 = 1;
						break;
				// Additional prox 8 and 9 of the gumstix extension.
				case 4: e_ambient_ir[9] = *adc_ptr++;
						e_ambient_ir[8] = *adc_ptr++;
						e_mic_scan[0][e_last_mic_scan_id] = *adc_ptr++;
						e_mic_scan[1][e_last_mic_scan_id] = *adc_ptr++;
						e_mic_scan[2][e_last_mic_scan_id] = *adc_ptr;
						if(++e_last_mic_scan_id>=MIC_SAMP_NB) {
							e_last_mic_scan_id=0;
							is_ad_array_filled = 1; 	 // indicate that the array is filled
						}
						BODY_LED = 1;
						FRONT_LED = 1;
						break;
			}
		}
		//prox channels copy (ambient and reflected)
		else if (period_counter == PULSE_PERIOD-1)
		{
			switch (prox_number)
			{
				// prox 0 and 4
				case 0: e_ambient_and_reflected_ir[0] = *adc_ptr++;
						e_ambient_and_reflected_ir[4] = *adc_ptr;
						PULSE_IR0 = 0;
						prox_number = 1;
						break;
				// prox 1 and 5
				case 1: e_ambient_and_reflected_ir[1] = *adc_ptr++;
						e_ambient_and_reflected_ir[5] = *adc_ptr;
						PULSE_IR1 = 0;
						prox_number = 2;
						break;
				// prox 2 and 6
				case 2: e_ambient_and_reflected_ir[2] = *adc_ptr++;
                        e_ambient_and_reflected_ir[6] = *adc_ptr;
                        PULSE_IR2 = 0;
						prox_number = 3;
						break;
				// prox 3 and 7
				case 3: e_ambient_and_reflected_ir[3] = *adc_ptr++;
						e_ambient_and_reflected_ir[7] = *adc_ptr;
						PULSE_IR3 = 0;
						if(selector==10) {
							prox_number = 4;
						} else {
                            prox_number = 0;
						}
						break;
				// Additional prox 8 and 9 of the gumstix extension.
				case 4: e_ambient_and_reflected_ir[9] = *adc_ptr++;
						e_ambient_and_reflected_ir[8] = *adc_ptr++;
						BODY_LED = 0;
						FRONT_LED = 0;
						prox_number = 0;
						e_mic_scan[0][e_last_mic_scan_id] = *adc_ptr++;
						e_mic_scan[1][e_last_mic_scan_id] = *adc_ptr++;
						e_mic_scan[2][e_last_mic_scan_id] = *adc_ptr;
						if(++e_last_mic_scan_id>=MIC_SAMP_NB) {
							e_last_mic_scan_id=0;
							is_ad_array_filled = 1; 	 // indicate that the array is filled
						}
						break;
			}
		}

		if(period_counter++>=ACC_PROX_PERIOD)
			period_counter=0;
	}
	is_ad_acquisition_completed = 1; // indicate a new sample taken

}

/*! \brief To know if the ADC acquisitionn is completed
 * \return 0 if the new acquisition is not made, 1 if completed.
 */
unsigned char e_ad_is_acquisition_completed(void)
{
	return is_ad_acquisition_completed;
}

/*! \brief To know if the ADC acquisitionn of microphone only is completed
 * \return 0 if the new acquisition is not made, 1 if completed.
 */
unsigned char e_ad_is_array_filled(void)
{
	return is_ad_array_filled;
}

/*! \brief Enable the ADC conversion
 */
void e_ad_scan_on(void)
{
	ADCON1bits.ADON = 1;	//enable AD conversion
}

/*! \brief Disable the ADC conversion
 */
void e_ad_scan_off(void)
{
	ADCON1bits.ADON = 0;	//disable AD conversion
}
