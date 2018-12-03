/********************************************************************************

			Advance agenda events of e-puck
			December 2004: first version
			Lucas Meier & Francesco Mondada


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Francesco Mondada, Lucas Meier

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \ingroup motor_LED
 * \brief Manage the agendas (timer2)
 *
 * This module manage the agendas with the timer2.
 * \n \n An agenda is a structure made to work as chained list. It containts:
 * the function you want to launch, the time setup between two launching events,
 * a counter to measure the current time, a pointer to the next element of the list.
 * \n \n Each times the timer2 has an interrupt, all the agenda chained list is
 * scanned to look if an agenda has to be treated according to the cycle value
 * and current counter value.
 * \n \n If one (or more) agenda has to be treated, his callback function is launch.
 * \author Code: Francesco Mondada, Lucas Meier \n Doc: Jonathan Besuchet
 */

#include "e_agenda.h"
#include "../e_epuck_ports.h"
#include <stdlib.h>

#define EXIT_OK 1

/*!pointer on the end of agenda chained list */
static Agenda *agenda_list = 0;


/*! \brief Start the agendas processing
 *
 * Start the agendas processing by starting the Timer2.
 */
void e_start_agendas_processing(void)
{
	T2CON = 0;							// reset Timer2 CONtrol register
	T2CONbits.TCKPS = 0;				// precsaler = 1
	TMR2 = 0;							// clear timer 2
	PR2 = (int)(0.1*MILLISEC);		// interrupt every 0.1 ms with 64 prescaler
	IFS0bits.T2IF = 0;					// clear interrupt flag
	IEC0bits.T2IE = 1;					// set interrupt enable bit
	T2CONbits.TON = 1;					// start Timer2
}


/*! \brief Stop all the agendas
 *
 * Stop all the agendas by disabling Timer2
 * \warning the memory allocated for the agenda isn't freed,
 * use \ref e_destroy_agenda(void (*func)(void)) for that.
 * \sa e_destroy_agenda
 */
void e_end_agendas_processing(void)
{
	T2CONbits.TON = 0;    // disable Timer2
}


/*! \brief Activate an agenda
 *
 * Activate an agenda and allocate memory for him if there isn't already
 * an agenda with the same callback function
 * (the agenda is active but isn't processed if he
 * has a null cycle value).
 * \param func	 function called if the cycle value is reached by the counter
 * \param cycle      cycle value in millisec/10
 * \return \ref EXIT_OK if the agenda has been created, exit the programme otherwise
 */
int e_activate_agenda(void (*func)(void), int cycle)
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
			return(AG_ALREADY_CREATED);
		else
			current = current->next;
	}
	if(!(current = malloc(sizeof(Agenda))))
		exit (EXIT_FAILURE);

	current->cycle = cycle;
	current->counter = 0;
	current->activate = 1;
	current->function = func;
	current->next = agenda_list;

	agenda_list = current;
	return(EXIT_OK);
}


/*! \brief Destroy an agenda
 *
 * Destroy the agenda with a given callback function.
 * \param func		 function to test
 * \return \ref EXIT_OK if the agenda has been destroyed, \ref AG_NOT_FOUND otherwise
 */
int e_destroy_agenda(void (*func)(void))
{
	Agenda *preceding = 0;
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			if (preceding)
				preceding->next = current->next;
			else
				agenda_list		= current->next;

			free(current);
			return(EXIT_OK);
		}
		else
		{
			preceding = current;
			current = current->next;
		}
	}
	return(AG_NOT_FOUND);
}


/*! \brief Change the cycle value of an agenda
 *
 * Change the cycle value of an agenda with a given callback function.
 * \param func		 function to test
 * \param cycle      new cycle value in millisec/10
 * \return \ref EXIT_OK if the cycle of the agenda has been modified,
 *         \ref AG_NOT_FOUND otherwise
 */
int e_set_agenda_cycle(void (*func)(void), int cycle)
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			current->cycle = cycle;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);
}


/*! \brief  Reset an agenda's counter
 *
 * Reset an agenda's counter with a given callback function.
 * \param func		 function to reset
 * \return \ref EXIT_OK if the cycle of the agenda has been reseted,
 *         \ref AG_NOT_FOUND otherwise
 * \warning This function RESET the agenda, if you just want a pause tell
 * \ref e_pause_agenda(void (*func)(void))
 * \sa e_pause_agenda
 */
int e_reset_agenda(void (*func)(void))
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			current->counter = 0;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);
}

/*! \brief Pause an agenda
 *
 * Pause an agenda but do not reset its information.
 * \param func		 function to pause
 * \return \ref EXIT_OK the agenda has been paused,
 *         \ref AG_NOT_FOUND otherwise
 */
int e_pause_agenda(void (*func)(void))
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			current->activate = 0;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);

}

/*! \brief Restart an agenda previously paused
 *
 * Restart an agenda previously paused.
 * \param func		 function to restart
 * \return \ref EXIT_OK if he agenda has been restarted,
 *         \ref AG_NOT_FOUND otherwise
 * \sa e_pause_agenda
 */
int e_restart_agenda(void (*func)(void))
{
	Agenda *current = agenda_list;

	while (current)
	{
		if (current->function == func)
		{
			current->activate = 1;
			return(EXIT_OK);
		}
		else
			current = current->next;
	}
	return(AG_NOT_FOUND);

}


/*! \brief Interrupt from timer2
 *
 * Parse the chained list of agenda.
 * \n Increment counter only.
 * \n Check if agenda has to be treated according to the cycle value
 * and current counter value.
 * \n Do it for number of cycle positive or null.
 * \n Check if a service has to be activated.
 */
void __attribute__((interrupt, auto_psv))
 _T2Interrupt(void)
{
	Agenda *current = agenda_list;

	IFS0bits.T2IF = 0;

	while (current)
	{
		// agenda must be active with a positive non-null cycle value
		if(current->activate == 1 && current->cycle > 0)
		{
			current->counter++;
			// check if the agenda event must be triggered
			if(current->counter > current->cycle-1) // a cycle value of 1 will be performed every interupt
			{
				current->function();	// trigger the associeted function
				current->counter=0;		// reset the counter
			}
		}
		current = current->next;
	}
  return;
}

/* End of File : alarm.c */
