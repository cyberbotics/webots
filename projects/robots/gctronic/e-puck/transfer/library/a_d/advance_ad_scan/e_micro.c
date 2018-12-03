/********************************************************************************

			Accessing the microphone data (advance)
			Novembre 7 2005	Borter Jean-Joel


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2005-2007 Borter Jean-Joel

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup a_d
 * \brief Accessing the microphone data.
 *
 * The functions of this file are made to deal with the microphones
 * data. You can simply get the current value of a given microphone.
 * You can know the volume of noise that the e-puck is enduring. You
 * can average the signal with a specified size.
 * \author Code: Borter Jean-Joel \n Doc: Jonathan Besuchet
 */

#include "p30F6014A.h"
#include "e_ad_conv.h"
#include "../../motor_led/e_epuck_ports.h"

/*****************************************************
 * external variables                                *
 *****************************************************/
extern int e_mic_scan[3][MIC_SAMP_NB];
extern unsigned int e_last_mic_scan_id;


/*****************************************************
 * user function                                     *
 *****************************************************/

/*! \brief Get the last value of a given micro
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \return result		last value of the micro
 */
int e_get_micro(unsigned int micro_id)
{
	return e_mic_scan[micro_id][e_last_mic_scan_id];
}


/*! \brief Get the average on a given number of sample from a micro
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \param filter_size	number of sample to average
 * \return result		last value of the micro
 */
int e_get_micro_average(unsigned int micro_id, unsigned int filter_size)
{
	long temp = 0;
	int i,j;

	// channel ID must be between 0 to 2 and
	// filter_size must be between 1 to SAMPLE_NUMBER
	if ((micro_id < 3) &&
		(filter_size > 0) && (filter_size <= MIC_SAMP_NB))
	{
		for (i=0, j=(MIC_SAMP_NB-(filter_size-(e_last_mic_scan_id+1)))%MIC_SAMP_NB ; i<filter_size ; i++, j=(j+1)%MIC_SAMP_NB)
		{
			temp += e_mic_scan[micro_id][j];
		}
	}
	return ((int)(temp/filter_size));
}


/*! \brief Get the difference between the highest and lowest sample.
 * Beware that from e-puck rev 1.3 the microphone sensitivity resulted a little bit different from the previous hardware revision;
 * some empirical tests show that the difference is about +/-15%.
 *
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \return result		volume
 */
int e_get_micro_volume (unsigned int micro_id)
{
	int i, highest = 0, lowest = 4999;

	for (i=0; i<MIC_SAMP_NB; i++)
	{
		if (e_mic_scan[micro_id][i] > highest)
		{
			highest = e_mic_scan[micro_id][i];
		}
		else if (e_mic_scan[micro_id][i] < lowest)
		{
			lowest = e_mic_scan[micro_id][i];
		}
	}
	return (highest - lowest);
}


/*! \brief Write to a given array, the last values of one micro
 *
 * Write to a given array, the last values of one micro. The values are
 * stored with the last one first, and the older one at the end of the array.
 * \n [ t ][ t-1 ][ t-2 ][ t-3 ]...[ t-(samples_nb-1) ][ t-samples_nb ]
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \param *result		pointer on the result array
 * \param samples_nb	size of the result array
 *							(must be between 1 and \ref MIC_SAMP_NB)
 */

void e_get_micro_last_values (int micro_id, int * result, unsigned samples_nb)
{
	int i;
	if (samples_nb > 0 && samples_nb <= MIC_SAMP_NB)
	{
		for (i=0 ; i<samples_nb ; i++)
		{
			result[samples_nb-1 - i] = e_mic_scan[micro_id][(e_last_mic_scan_id + i) % MIC_SAMP_NB];
		}
	}
}
