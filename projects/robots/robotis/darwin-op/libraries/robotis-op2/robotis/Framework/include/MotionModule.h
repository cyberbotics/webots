/*
 *   MotionModule.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MOTION_MODULE_H_
#define _MOTION_MODULE_H_

#include "JointData.h"

namespace Robot
{
	
	/*
	Represents an abstract motion (maybe instanciated by a walking, etc.)
	*/
	class MotionModule
	{
	private:

	protected:

	public:
	/*state of all the articulations (the motors MX-28)*/
		JointData m_Joint;

		static const int TIME_UNIT = 8; //msec 

		virtual void Initialize() = 0;
		virtual void Process() = 0;
	};
}

#endif
