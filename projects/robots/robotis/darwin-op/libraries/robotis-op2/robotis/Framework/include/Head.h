/*
 *   Head.h
 *   represents the head of the robot
 *   Author: ROBOTIS
 *
 */

#ifndef _HEAD_H_
#define _HEAD_H_

#include <string.h>

#include "minIni.h"
#include "MotionModule.h"
#include "Point.h"

#define HEAD_SECTION    "Head Pan/Tilt"
#define INVALID_VALUE   -1024.0

namespace Robot
{
	class Head : public MotionModule
	{
	private:
		static Head* m_UniqueInstance;
		double m_LeftLimit;
		double m_RightLimit;
		double m_TopLimit;
		double m_BottomLimit;
		double m_Pan_Home;
		double m_Tilt_Home;
		double m_Pan_err;
		double m_Pan_err_diff;
		double m_Pan_p_gain;
		double m_Pan_d_gain;
		double m_Tilt_err;
		double m_Tilt_err_diff;
		double m_Tilt_p_gain;
		double m_Tilt_d_gain;
		double m_PanAngle;
		double m_TiltAngle;
		
		Head();
		void CheckLimit();

	public:
		static Head* GetInstance() { return m_UniqueInstance; }
		
		~Head();

		void Initialize();
		void Process();

		double GetTopLimitAngle()		{ return m_TopLimit; }
		double GetBottomLimitAngle()	{ return m_BottomLimit; }
		double GetRightLimitAngle()		{ return m_RightLimit; }
		double GetLeftLimitAngle()		{ return m_LeftLimit; }

		double GetPanAngle()		{ return m_PanAngle; }
		double GetTiltAngle()		{ return m_TiltAngle; }

		void MoveToHome();
		void MoveByAngle(double pan, double tilt);
		void MoveByAngleOffset(double pan, double tilt);
		void InitTracking();
		void MoveTracking(Point2D err); // For image processing
		void MoveTracking();


/*Read/write from a INI file*/
		
        void LoadINISettings(minIni* ini);
        void LoadINISettings(minIni* ini, const std::string &section);
        void SaveINISettings(minIni* ini);
        void SaveINISettings(minIni* ini, const std::string &section);
	};
}

#endif
