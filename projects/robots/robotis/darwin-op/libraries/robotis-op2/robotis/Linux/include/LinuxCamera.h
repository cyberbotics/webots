/*
 *   LinuxCamera.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _LINUX_CAMERA_H_
#define _LINUX_CAMERA_H_

#include <stdlib.h>
#include <linux/videodev2.h>
#include <sys/time.h>

#include "Image.h"
#include "minIni.h"

namespace Robot
{
    class CameraSettings
    {
    private:

    protected:

    public:
        int brightness; /* 0 ~ 255 */
        int contrast;   /* 0 ~ 255 */
        int saturation; /* 0 ~ 255 */
        int gain;       /* 0 ~ 255 */
        int exposure;   /* 0 ~ 10000 */

        CameraSettings() :
            brightness(-1),
            contrast(-1),
            saturation(-1),
            gain(255),
            exposure(1000)
        {}
    };

	class LinuxCamera
	{
	private:
        static LinuxCamera* uniqueInstance;

        CameraSettings settings;

	    int camera_fd;
	    struct buffer {
	        void * start;
	        size_t length;
	    };
	    struct buffer * buffers;
	    unsigned int n_buffers;

        LinuxCamera();

        void ErrorExit(const char* s);
	    int ReadFrame();
	    int ReadFrameWb();  // for Webots only

	protected:

	public:
		bool DEBUG_PRINT;
        FrameBuffer* fbuffer;

		~LinuxCamera();

        static LinuxCamera* GetInstance() { return uniqueInstance; }

        int Initialize(int deviceIndex);

	    int v4l2GetControl(int control);
	    int v4l2SetControl(int control, int value);
	    int v4l2ResetControl(int control);

	    void LoadINISettings(minIni* ini);
	    void SaveINISettings(minIni* ini);

	    void SetCameraSettings(const CameraSettings& newset);
	    const CameraSettings& GetCameraSettings();

	    void SetAutoWhiteBalance(int isAuto) { v4l2SetControl(V4L2_CID_AUTO_WHITE_BALANCE, isAuto); }
	    unsigned char GetAutoWhiteBalance() { return (unsigned char)(v4l2GetControl(V4L2_CID_AUTO_WHITE_BALANCE)); }

	    void CaptureFrame();
	    void CaptureFrameWb(); // for Webots only
	};
}

#endif
