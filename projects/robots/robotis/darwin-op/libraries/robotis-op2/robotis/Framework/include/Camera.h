/*
 *   Camera.h
 *   This class contains the constants about the camera.
 *   Author: ROBOTIS
 *
 */

#ifndef _CAMERA_H_
#define _CAMERA_H_


namespace Robot
{
	class Camera
	{
	public:
		static constexpr double VIEW_V_ANGLE = 46.0; //degree
		static constexpr double VIEW_H_ANGLE = 58.0; //degree

		static int WIDTH;
		static int HEIGHT;
	};

}

#endif
