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

#ifndef _MICRO
#define _MICRO


// ID of the different captor in their respective array

#define MIC0_BUFFER	0
#define MIC1_BUFFER	1
#define MIC2_BUFFER	2

/***********************************************************************
 * -------------------- Functions from micro.c -----------------------
 **********************************************************************/
int e_get_micro(unsigned int micro_id);
int e_get_micro_average(unsigned int micro_id, unsigned int filter_size);
int e_get_micro_volume (unsigned int micro_id);

#endif /*_MICRO*/

/* End of File : ad_conv.h */
