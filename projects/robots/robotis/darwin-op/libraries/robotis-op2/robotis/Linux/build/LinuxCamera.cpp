/*
 *   LinuxCamera.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include "Camera.h"
#include "LinuxCamera.h"
#include "ImgProcess.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

using namespace Robot;

LinuxCamera* LinuxCamera::uniqueInstance = new LinuxCamera();

LinuxCamera::LinuxCamera() :
        settings(CameraSettings()),
        camera_fd(-1),
        buffers(0),
        n_buffers(0)
{
	DEBUG_PRINT = false;
    fbuffer = new FrameBuffer(Camera::WIDTH, Camera::HEIGHT);
}

LinuxCamera::~LinuxCamera()
{
    /* streaming off */
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(camera_fd, VIDIOC_STREAMOFF, &type) == -1)
        ErrorExit("VIDIOC_STREAMOFF");

    /* unmap buffers */
    for(unsigned int i = 0; i < n_buffers; i++)
        if(munmap(buffers[i].start, buffers[i].length) == -1)
            ErrorExit("munmap");
    free(buffers);

    /* close device */
    close(camera_fd);
    camera_fd = -1;

    delete fbuffer;
}

void LinuxCamera::ErrorExit(const char* s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    if(errno == 16)
        fprintf(stderr, "\nYou must free up camera resources used by running programs. \n"
                "Kill a program that uses the camera.\n\n");
    exit(EXIT_FAILURE);
}

int LinuxCamera::Initialize(int deviceIndex)
{
    struct stat st;
    char devName[15] = {0, };

    sprintf(devName, "/dev/video%d", deviceIndex);

    if (-1 == stat (devName, &st)) {
        fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                 devName, errno, strerror (errno));
        exit (EXIT_FAILURE);
    }

    if (!S_ISCHR (st.st_mode)) {
        fprintf (stderr, "%s is no device\n", devName);
        exit (EXIT_FAILURE);
    }

    camera_fd = open(devName, O_RDWR | O_NONBLOCK, 0);
    if (-1 == camera_fd) {
        fprintf (stderr, "Cannot open '%s': %d, %s\n",
                 devName, errno, strerror (errno));
        exit (EXIT_FAILURE);
    }

    /* set format */
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(ioctl(camera_fd, VIDIOC_CROPCAP, &cropcap) == 0)
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        ioctl(camera_fd, VIDIOC_S_CROP, &crop);
    }
    else
    {
        /* Errors ignored. */
    }

    struct v4l2_format fmt;
    CLEAR(fmt);

    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = Camera::WIDTH;
    fmt.fmt.pix.height      = Camera::HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

    if (-1 == ioctl (camera_fd, VIDIOC_S_FMT, &fmt))
        ErrorExit ("VIDIOC_S_FMT");

    unsigned int min = fmt.fmt.pix.width * 2;
    if(fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if(fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    /* set frame rate */
    struct v4l2_streamparm fps;
    CLEAR(fps);

    fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(camera_fd, VIDIOC_G_PARM, &fps) == -1)
        ErrorExit("VIDIOC_G_PARM");

    fps.parm.capture.timeperframe.numerator = 1;
    fps.parm.capture.timeperframe.denominator = 30;
    if(ioctl(camera_fd, VIDIOC_S_PARM, &fps) == -1)
        ErrorExit("VIDIOC_S_PARM");

    /* init mmap */
    struct v4l2_requestbuffers req;
    CLEAR(req);

    req.count           = 4;
    req.type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory          = V4L2_MEMORY_MMAP;

    if (-1 == ioctl (camera_fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf (stderr, "%s does not support "
                     "memory mapping\n", devName);
            exit (EXIT_FAILURE);
        } else {
            ErrorExit ("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        fprintf (stderr, "Insufficient buffer memory on %s\n",
                 devName);
        exit (EXIT_FAILURE);
    }

    buffers = (buffer *)calloc(req.count, sizeof(*buffers));
    if (!buffers) {
        fprintf (stderr, "Out of memory\n");
        exit (EXIT_FAILURE);
    }

    for(n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory  = V4L2_MEMORY_MMAP;
        buf.index   = n_buffers;

        if (-1 == ioctl (camera_fd, VIDIOC_QUERYBUF, &buf))
            ErrorExit ("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
                mmap(NULL,
                     buf.length,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     camera_fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
                ErrorExit ("mmap");
    }

    /* queue buffers */
    for(unsigned int i = 0; i < n_buffers; ++i)
    {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory  = V4L2_MEMORY_MMAP;
        buf.index   = i;

        if (-1 == ioctl (camera_fd, VIDIOC_QBUF, &buf))
            ErrorExit ("VIDIOC_QBUF");
    }

    /* streaming on */
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl (camera_fd, VIDIOC_STREAMON, &type))
        ErrorExit ("VIDIOC_STREAMON");

    /* get camera default setting */
    settings.brightness = v4l2GetControl(V4L2_CID_BRIGHTNESS);
    settings.contrast   = v4l2GetControl(V4L2_CID_CONTRAST);
    settings.saturation = v4l2GetControl(V4L2_CID_SATURATION);
    settings.gain       = v4l2GetControl(V4L2_CID_GAIN);
    settings.exposure   = v4l2GetControl(V4L2_CID_EXPOSURE);

    /* set camera auto off */
    v4l2SetControl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    v4l2SetControl(V4L2_CID_AUTO_WHITE_BALANCE, 0);
    v4l2SetControl(V4L2_CID_AUTOGAIN, 0);
    v4l2SetControl(V4L2_CID_HUE_AUTO, 0);

    return 1;
}

void LinuxCamera::LoadINISettings(minIni* ini)
{
    int value = -2;
    CameraSettings newset = GetCameraSettings();

    if((value = ini->geti("Camera", "Brightness", -2)) != -2)   newset.brightness = value;
    if((value = ini->geti("Camera", "Contrast", -2)) != -2)     newset.contrast = value;
    if((value = ini->geti("Camera", "Saturation", -2)) != -2)   newset.saturation = value;
    if((value = ini->geti("Camera", "Gain", -2)) != -2)         newset.gain = value;
    if((value = ini->geti("Camera", "Exposure", -2)) != -2)     newset.exposure = value;

    SetCameraSettings(newset);
}

void LinuxCamera::SaveINISettings(minIni* ini)
{
    ini->put("Camera", "Brightness",settings.brightness);
    ini->put("Camera", "Contrast",  settings.contrast);
    ini->put("Camera", "Saturation",settings.saturation);
    ini->put("Camera", "Gain",      settings.gain);
    ini->put("Camera", "Exposure",  settings.exposure);
}

const CameraSettings& LinuxCamera::GetCameraSettings()
{
    settings.brightness = v4l2GetControl(V4L2_CID_BRIGHTNESS);
    settings.contrast   = v4l2GetControl(V4L2_CID_CONTRAST);
    settings.saturation = v4l2GetControl(V4L2_CID_SATURATION);
    settings.gain       = v4l2GetControl(V4L2_CID_GAIN);
    settings.exposure   = v4l2GetControl(V4L2_CID_EXPOSURE_ABSOLUTE);

    return settings;
}

void LinuxCamera::SetCameraSettings(const CameraSettings &newset)
{
    if(newset.brightness == -1)
        v4l2ResetControl(V4L2_CID_BRIGHTNESS);
    else if(newset.brightness != v4l2GetControl(V4L2_CID_BRIGHTNESS))
        v4l2SetControl(V4L2_CID_BRIGHTNESS, newset.brightness);

    if(newset.contrast == -1)
        v4l2ResetControl(V4L2_CID_CONTRAST);
    else if(newset.contrast != v4l2GetControl(V4L2_CID_CONTRAST))
        v4l2SetControl(V4L2_CID_CONTRAST, newset.contrast);

    if(newset.saturation == -1)
        v4l2ResetControl(V4L2_CID_SATURATION);
    else if(newset.saturation != v4l2GetControl(V4L2_CID_SATURATION))
        v4l2SetControl(V4L2_CID_SATURATION, newset.saturation);

    if(newset.gain == -1)
        v4l2ResetControl(V4L2_CID_GAIN);
    else if(newset.gain != v4l2GetControl(V4L2_CID_GAIN))
        v4l2SetControl(V4L2_CID_GAIN, newset.gain);

    if(newset.exposure == -1)
        v4l2ResetControl(V4L2_CID_EXPOSURE_ABSOLUTE);
    else if(newset.exposure != v4l2GetControl(V4L2_CID_EXPOSURE_ABSOLUTE))
        v4l2SetControl(V4L2_CID_EXPOSURE_ABSOLUTE, newset.exposure);

    GetCameraSettings();
}

int LinuxCamera::v4l2GetControl(int control)
{
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control control_s;
    int err;

    queryctrl.id = control;
    if (ioctl(camera_fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
        return -1;
    control_s.id = control;
    if ((err = ioctl (camera_fd, VIDIOC_G_CTRL, &control_s)) < 0)
        return -1;

    return control_s.value;
}

int LinuxCamera::v4l2SetControl(int control, int value)
{
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    int min, max, step, val_def;
    int err;

    queryctrl.id = control;
    if(ioctl(camera_fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
        return -1;
    if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        return -1;

    min = queryctrl.minimum;
    max = queryctrl.maximum;
    step = queryctrl.step;
    val_def = queryctrl.default_value;
    if((value >= min) && (value <= max)) {
        control_s.id = control;
        control_s.value = value;
        if((err = ioctl (camera_fd, VIDIOC_S_CTRL, &control_s)) < 0)
            return -1;
    }
    return 0;
}

int LinuxCamera::v4l2ResetControl(int control)
{
    struct v4l2_control control_s;
    struct v4l2_queryctrl queryctrl;
    int val_def;
    int err;

    queryctrl.id = control;
    if (ioctl(camera_fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
        return -1;
    val_def = queryctrl.default_value;
    control_s.id = control;
    control_s.value = val_def;
    if ((err = ioctl (camera_fd, VIDIOC_S_CTRL, &control_s)) < 0)
        return -1;

    return 0;
}

int LinuxCamera::ReadFrame()
{
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == ioctl (camera_fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            exit (EXIT_FAILURE);
        }
    }

    assert (buf.index < n_buffers);

    //process_image (buffers[buf.index].start);
    for(int i = 0; i < fbuffer->m_YUVFrame->m_ImageSize; i++)
        fbuffer->m_YUVFrame->m_ImageData[i] = ((unsigned char*)buffers[buf.index].start)[i];
    ImgProcess::HFlipYUV(fbuffer->m_YUVFrame);
    ImgProcess::VFlipYUV(fbuffer->m_YUVFrame);
    ImgProcess::YUVtoRGB(fbuffer);
    ImgProcess::RGBtoHSV(fbuffer);

    if (-1 == ioctl (camera_fd, VIDIOC_QBUF, &buf))
        ErrorExit ("VIDIOC_QBUF");

    return 1;
}

void LinuxCamera::CaptureFrame()
{
	if(DEBUG_PRINT == true)
	{
		struct timeval tv;
		static double beforeTime = 0;
		double currentTime;
		double durationTime;
		
		gettimeofday(&tv, NULL);
		currentTime = (double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0;
		durationTime = currentTime - beforeTime;
		fprintf(stderr, "\rCamera: %.1fmsec(%.1ffps)                    ", durationTime, 1000.0 / durationTime);
		beforeTime = currentTime;
	}

    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO (&fds);
        FD_SET (camera_fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select (camera_fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                continue;

            exit (EXIT_FAILURE);
        }

        if (0 == r) {
            fprintf (stderr, "select timeout\n");
            exit (EXIT_FAILURE);
        }

        if (ReadFrame())
            break;

        /* EAGAIN - continue select loop. */
    }
}

// WEBOTS PART //

int LinuxCamera::ReadFrameWb()
{
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == ioctl (camera_fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
            exit (EXIT_FAILURE);
        }
    }

    assert (buf.index < n_buffers);

    // Extract the image from the buffer, flip it (H and V) and convert it in BGRA format (everything in only one loop)
    unsigned char *yuyv = (unsigned char*)buffers[buf.index].start + fbuffer->m_YUVFrame->m_ImageSize/2 - 1;
    unsigned char *bgra  = fbuffer->m_BGRAFrame->m_ImageData;
    int z = 0;

    while(yuyv > (((unsigned char*)buffers[buf.index].start))) {
            int r, g, b;
            int y, u, v;

            if(z)
                y = yuyv[-3] << 8;
            else
                y = yuyv[-1] << 8;
            u = yuyv[-2] - 128;
            v = yuyv[0] - 128;

            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(bgra++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);
            *(bgra++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(bgra++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(bgra++) = 255; // a

            if (z++)
            {
                z = 0;
                yuyv -= 4;
            }
    }


    if (-1 == ioctl (camera_fd, VIDIOC_QBUF, &buf))
        ErrorExit ("VIDIOC_QBUF");

    return 1;
}

void LinuxCamera::CaptureFrameWb()
{
	if(DEBUG_PRINT == true)
	{
		struct timeval tv;
		static double beforeTime = 0;
		double currentTime;
		double durationTime;
		
		gettimeofday(&tv, NULL);
		currentTime = (double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0;
		durationTime = currentTime - beforeTime;
		fprintf(stderr, "\rCamera: %.1fmsec(%.1ffps)                    ", durationTime, 1000.0 / durationTime);
		beforeTime = currentTime;
	}


    for (;;) {

        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO (&fds);
        FD_SET (camera_fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select (camera_fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                continue;

            exit (EXIT_FAILURE);
        }

        if (0 == r) {
            fprintf (stderr, "select timeout\n");
            exit (EXIT_FAILURE);
        }

       if (ReadFrameWb())
            break;

        /* EAGAIN - continue select loop. */
    }
}

