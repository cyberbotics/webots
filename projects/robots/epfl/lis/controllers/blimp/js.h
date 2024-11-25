/*
     PLIB - A Suite of Portable Game Libraries
     Copyright (C) 2001  Steve Baker

     This library is free software; you can redistribute it and/or
     modify it under the terms of the GNU Library General Public
     License as published by the Free Software Foundation; either
     version 2 of the License, or (at your option) any later version.

     This library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
     Library General Public License for more details.

     You should have received a copy of the GNU Library General Public
     License along with this library; if not, write to the Free
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

     For further information visit http://plib.sourceforge.net

     $Id: js.h,v 1.1.1.1 2004/11/19 09:54:32 michel Exp $
*/

#ifndef __INCLUDED_JS_H__
#define __INCLUDED_JS_H__ 1

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // -dw- for memcpy

#ifdef macintosh
#include <InputSprocket.h>
#endif

/*
  FreeBSD port - courtesy of Stephen Montgomery-Smith
  <stephen@math.missouri.edu>

  NetBSD mods - courtesy Rene Hexel.

  The next lines are to define BSD
  see http://www.freebsd.org/handbook/porting.html for why we do this
*/
// lint -save -e620

#if (defined(__unix__) || defined(unix)) && !defined(USG)
#include <sys/param.h>
#endif

#ifdef _WIN32
#include <windows.h>
#if defined(__CYGWIN32__) || defined(__CYGWIN__)
#define NEAR /* */
#define FAR  /* */
#endif
#include <mmsystem.h>
#include <string.h>
#else

#include <fcntl.h>
#include <unistd.h>

#if defined(__FreeBSD__) || defined(__NetBSD__)
#include <machine/joystick.h>
#define JS_DATA_TYPE joystick
#define JS_RETURN (sizeof(struct JS_DATA_TYPE))
#elif defined(__linux__)
#include <errno.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>

/* check the joystick driver version */

#ifdef JS_VERSION
#if JS_VERSION >= 0x010000
#define JS_NEW
#endif
#endif

#else
#ifndef JS_DATA_TYPE

/*
  Not Windoze and no joystick driver...

  Well - we'll put these values in and that should
  allow the code to at least compile. The JS open
  routine should error out and shut off all the code
  downstream anyway
*/

struct JS_DATA_TYPE {
  int buttons;
  int x;
  int y;
};

#define JS_RETURN (sizeof(struct JS_DATA_TYPE))
#endif
#endif
#endif

#define JS_TRUE 1
#define JS_FALSE 0

#ifdef _WIN32
#define _JS_MAX_AXES 6
#elif defined(macintosh)
#define _JS_MAX_AXES 9
#else
#if defined(__FreeBSD__) || defined(__NetBSD__)
#define _JS_MAX_AXES 2
#else
#define _JS_MAX_AXES 9
#endif
#endif

class jsJoystick {
#ifdef macintosh

#define isp_num_axis 9
#define isp_num_needs 41

  ISpElementReference isp_elem[isp_num_needs];
  ISpNeed isp_needs[isp_num_needs];

#endif

#if defined(__FreeBSD__) || defined(__NetBSD__)
  int id;
#endif
#ifdef _WIN32
  JOYINFOEX js;
  UINT js_id;
#else
#ifdef JS_NEW
  js_event js;
  int tmp_buttons;
  float tmp_axes[_JS_MAX_AXES];
#else
  JS_DATA_TYPE js;
#endif
  char fname[128];
  int fd;
#endif

  int error;
  int num_axes;
  int num_buttons;

  float dead_band[_JS_MAX_AXES];
  float saturate[_JS_MAX_AXES];
  float center[_JS_MAX_AXES];
  float max[_JS_MAX_AXES];
  float min[_JS_MAX_AXES];

  void open() {
#ifdef macintosh

    OSStatus err;

    err = ISpStartup();

    if (err == noErr) {
#define ISP_CHECK_ERR(x) \
  if (x != noErr) {      \
    setError();          \
    return;              \
  }

      setError();

      // initialize the needs structure
      ISpNeed temp_isp_needs[isp_num_needs] = {
        {"\pX-Axis", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pY-Axis", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pZ-Axis", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pR-Axis", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pAxis   4", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pAxis   5", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pAxis   6", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pAxis   7", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},
        {"\pAxis   8", 128, 0, 0, kISpElementKind_Axis, kISpElementLabel_None, 0, 0, 0, 0},

        {"\pButton 0", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 1", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 2", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 3", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 4", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 5", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 6", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 7", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 8", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 9", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 10", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 11", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 12", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 13", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 14", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 15", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 16", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 17", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 18", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 19", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 20", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 21", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 22", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 23", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 24", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 25", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 26", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 27", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 28", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 29", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 30", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
        {"\pButton 31", 128, 0, 0, kISpElementKind_Button, kISpElementLabel_Btn_Select, 0, 0, 0, 0},
      };

      memcpy(isp_needs, temp_isp_needs, sizeof(temp_isp_needs));

      // next two calls allow keyboard and mouse to emulate other input devices (gamepads, joysticks, etc)

      /*
      err = ISpDevices_ActivateClass (kISpDeviceClass_Keyboard);
      ISP_CHECK_ERR(err)

      err = ISpDevices_ActivateClass (kISpDeviceClass_Mouse);
      ISP_CHECK_ERR(err)
      */

      err = ISpElement_NewVirtualFromNeeds(isp_num_needs, isp_needs, isp_elem, 0);
      ISP_CHECK_ERR(err)

      err = ISpInit(isp_num_needs, isp_needs, isp_elem, 'PLIB', nil, 0, 128, 0);
      ISP_CHECK_ERR(err)

      num_buttons = isp_num_needs - isp_num_axis;
      num_axes = isp_num_axis;
      if (num_axes > _JS_MAX_AXES)
        num_axes = _JS_MAX_AXES;

      for (int i = 0; i < num_axes; i++) {
        dead_band[i] = 0;
        saturate[i] = 1;
        center[i] = kISpAxisMiddle;
        max[i] = kISpAxisMaximum;
        min[i] = kISpAxisMinimum;
      }

      error = false;
    } else {
      setError();
      num_buttons = num_axes = 0;
    }

#elif defined(_WIN32)

    JOYCAPS jsCaps;

    js.dwFlags = JOY_RETURNALL;
    js.dwSize = sizeof(js);

    memset(&jsCaps, 0, sizeof(jsCaps));

    error = (joyGetDevCaps(js_id, &jsCaps, sizeof(jsCaps)) != JOYERR_NOERROR);
    if (jsCaps.wNumAxes == 0) {
      num_axes = 0;
      setError();
    } else {
      /* accept all the axis */
      num_axes = _JS_MAX_AXES;
      min[5] = (float)jsCaps.wVmin;
      max[5] = (float)jsCaps.wVmax;
      min[4] = (float)jsCaps.wUmin;
      max[4] = (float)jsCaps.wUmax;
      min[3] = (float)jsCaps.wRmin;
      max[3] = (float)jsCaps.wRmax;
      min[2] = (float)jsCaps.wZmin;
      max[2] = (float)jsCaps.wZmax;
      min[1] = (float)jsCaps.wYmin;
      max[1] = (float)jsCaps.wYmax;
      min[0] = (float)jsCaps.wXmin;
      max[0] = (float)jsCaps.wXmax;
    }

    for (int i = 0; i < num_axes; i++) {
      center[i] = (max[i] + min[i]) / 2.0f;
      dead_band[i] = 0.0f;
      saturate[i] = 1.0f;
    }

#else
#if defined(__FreeBSD__) || defined(__NetBSD__)
    FILE *joyfile;
    char joyfname[1024];
    int noargs, in_no_axes;
#endif

    /* Default for older Linux systems. */

    num_axes = 2;
    num_buttons = 32;

#ifdef JS_NEW
    for (int i = 0; i < _JS_MAX_AXES; i++)
      tmp_axes[i] = 0.0f;

    tmp_buttons = 0;
#endif

    // fd = ::open ( fname, O_RDONLY | O_NONBLOCK ) ;
    fd = ::open(fname, O_RDONLY);

    error = (fd < 0);

    if (error)
      return;

#if defined(__FreeBSD__) || defined(__NetBSD__)

    float axes[_JS_MAX_AXES];
    int buttons[_JS_MAX_AXES];
    rawRead(buttons, axes);
    error = axes[0] < -1000000000.0f;
    if (error)
      return;

    snprintf(joyfname, sizeof(joyfname), "%s/.joy%drc", ::getenv("HOME"), id);

    joyfile = fopen(joyfname, "r");
    if (joyfile == NULL)
      return;
    noargs = fscanf(joyfile, "%d%f%f%f%f%f%f", &in_no_axes, &min[0], &center[0], &max[0], &min[1], &center[1], &max[1]);
    error = noargs != 7 || in_no_axes != _JS_MAX_AXES;
    fclose(joyfile);
    if (error)
      return;

    for (int i = 0; i < _JS_MAX_AXES; i++) {
      dead_band[i] = 0.0f;
      saturate[i] = 1.0f;
    }

#else

#ifdef JS_NEW
    /*
      Set the correct number of axes for the linux driver
    */
    ioctl(fd, JSIOCGAXES, &num_axes);
    ioctl(fd, JSIOCGBUTTONS, &num_buttons);
    fcntl(fd, F_SETFL, O_NONBLOCK);

    if (num_axes > _JS_MAX_AXES)
      num_axes = _JS_MAX_AXES;
#endif

#ifndef JS_NEW
    /*
      The Linux driver seems to return 512 for all axes
      when no stick is present - but there is a chance
      that could happen by accident - so it's gotta happen
      on both axes for at least 100 attempts.
    */
    int counter = 0;

    do {
      rawRead(NULL, center);
      counter++;
    } while (!error && counter < 100 && center[0] == 512.0f && center[1] == 512.0f);

    if (counter >= 100)
      setError();
#endif

    for (int i = 0; i < _JS_MAX_AXES; i++) {
#ifdef JS_NEW
      max[i] = 32767.0f;
      center[i] = 0.0f;
      min[i] = -32767.0f;
#else
      max[i] = center[i] * 2.0f;
      min[i] = 0.0f;
#endif
      dead_band[i] = 0.0f;
      saturate[i] = 1.0f;
    }

#endif
#endif
  }

  void close() {
    assert(this);
#if !defined(_WIN32) && !defined(macintosh)
    if (!error)
      ::close(fd);
#endif

#ifdef macintosh

    ISpSuspend();
    ISpStop();
    ISpShutdown();

#endif
  }

  float fudge_axis(float value, int axis) const {
    if (value < center[axis]) {
      float xx = (value - center[axis]) / (center[axis] - min[axis]);

      if (xx < -saturate[axis])
        return -1.0f;

      if (xx > -dead_band[axis])
        return 0.0f;

      xx = (xx + dead_band[axis]) / (saturate[axis] - dead_band[axis]);

      return (xx < -1.0f) ? -1.0f : xx;
    } else {
      float xx = (value - center[axis]) / (max[axis] - center[axis]);

      if (xx > saturate[axis])
        return 1.0f;

      if (xx < dead_band[axis])
        return 0.0f;

      xx = (xx - dead_band[axis]) / (saturate[axis] - dead_band[axis]);

      return (xx > 1.0f) ? 1.0f : xx;
    }
  }

public:
  // cppcheck-suppress uninitMemberVar ; js class member
  explicit jsJoystick(int ident = 0) {
#ifdef _WIN32
    switch (ident) {
      case 0:
        js_id = JOYSTICKID1;
        open();
        break;
      case 1:
        js_id = JOYSTICKID2;
        open();
        break;
      default:
        num_axes = 0;
        num_buttons = 0;
        setError();
        break;
    }

#else
    fd = -1;
#if defined(__FreeBSD__) || defined(__NetBSD__)
    id = ident;
    snprintf(fname, sizeof(fname), "/dev/joy%d", ident);
#else
    snprintf(fname, sizeof(fname), "/dev/js%d", ident);
#endif
    open();
#endif
  }

  ~jsJoystick() {
    close();
  }

  int getNumAxes() const {
    return num_axes;
  }
  int notWorking() const {
    return error;
  }
  void setError() {
    error = JS_TRUE;
  }

  float getDeadBand(int axis) const {
    return dead_band[axis];
  }
  void setDeadBand(int axis, float db) {
    dead_band[axis] = db;
  }

  float getSaturation(int axis) const {
    return saturate[axis];
  }
  void setSaturation(int axis, float st) {
    saturate[axis] = st;
  }

  void setMinRange(const float *axes) {
    memcpy(min, axes, num_axes * sizeof(float));
  }
  void setMaxRange(const float *axes) {
    memcpy(max, axes, num_axes * sizeof(float));
  }
  void setCenter(const float *axes) {
    memcpy(center, axes, num_axes * sizeof(float));
  }

  void getMinRange(float *axes) {
    memcpy(axes, min, num_axes * sizeof(float));
  }
  void getMaxRange(float *axes) {
    memcpy(axes, max, num_axes * sizeof(float));
  }
  void getCenter(float *axes) {
    memcpy(axes, center, num_axes * sizeof(float));
  }

  void read(int *buttons, float *axes) {
    if (error) {
      if (buttons)
        *buttons = 0;

      if (axes)
        for (int i = 0; i < num_axes; i++)
          axes[i] = 0.0f;
    }

    float raw_axes[_JS_MAX_AXES];

    rawRead(buttons, raw_axes);

    if (axes)
      for (int i = 0; i < num_axes; i++)
        axes[i] = fudge_axis(raw_axes[i], i);
  }

  void rawRead(int *buttons, float *axes) {
    if (error) {
      if (buttons)
        *buttons = 0;

      if (axes) {
        for (int i = 0; i < num_axes; i++)
          axes[i] = 1500.0f;
      }

      return;
    }

#ifdef macintosh

    int i;
    int err;
    UInt32 state;

    if (buttons) {
      *buttons = 0;

      for (i = 0; i < num_buttons; i++) {
        err = ISpElement_GetSimpleState(isp_elem[i + isp_num_axis], &state);
        ISP_CHECK_ERR(err)

        *buttons |= state << i;
      }
    }

    if (axes) {
      for (i = 0; i < num_axes; i++) {
        err = ISpElement_GetSimpleState(isp_elem[i], &state);
        ISP_CHECK_ERR(err);

        axes[i] = (float)state;
      }
    }

#elif defined(_WIN32)
    MMRESULT status = joyGetPosEx(js_id, &js);

    if (status != JOYERR_NOERROR) {
      setError();
      return;
    }

    if (buttons)
      *buttons = (int)js.dwButtons;

    if (axes) {
      /* WARNING - Fall through case clauses!! */
      // lint -save -e616

      switch (num_axes) {
        case 6:
          axes[5] = (float)js.dwVpos;
        case 5:
          axes[4] = (float)js.dwUpos;
        case 4:
          axes[3] = (float)js.dwRpos;
        case 3:
          axes[2] = (float)js.dwZpos;
        case 2:
          axes[1] = (float)js.dwYpos;
        case 1:
          axes[0] = (float)js.dwXpos;
          //   break;
          // default:
          //   ulSetError(UL_WARNING, "PLIB_JS: Wrong num_axes. Joystick input is now invalid");
      }
      // lint -restore
    }
#else

#ifdef JS_NEW

    while (1) {
      int status = ::read(fd, &js, sizeof(js_event));

      if (status != sizeof(js_event)) {
        /* use the old values */

        if (buttons)
          *buttons = tmp_buttons;
        if (axes)
          memcpy(axes, tmp_axes, sizeof(float) * num_axes);

        if (errno == EAGAIN)
          return;

        perror(fname);
        setError();
        return;
      }

      switch (js.type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
          if (js.value == 0) /* clear the flag */
            tmp_buttons &= ~(1 << js.number);
          else
            tmp_buttons |= (1 << js.number);
          break;

        case JS_EVENT_AXIS:
          if (js.number < num_axes) {
            tmp_axes[js.number] = (float)js.value;

            if (axes)
              memcpy(axes, tmp_axes, sizeof(float) * num_axes);
          }
          break;

        default:
          printf("PLIB_JS: Unrecognised /dev/js return!?!");

          /* use the old values */

          if (buttons)
            *buttons = tmp_buttons;
          if (axes)
            memcpy(axes, tmp_axes, sizeof(float) * num_axes);

          return;
      }

      if (buttons)
        *buttons = tmp_buttons;
    }

#else

    int status = ::read(fd, &js, JS_RETURN);

    if (status != JS_RETURN) {
      perror(fname);
      setError();
      return;
    }

    if (buttons)
#if defined(__FreeBSD__) || defined(__NetBSD__)
      *buttons = (js.b1 ? 1 : 0) | (js.b2 ? 2 : 0);
#else
      *buttons = js.buttons;
#endif

    if (axes) {
      axes[0] = (float)js.x;
      axes[1] = (float)js.y;
    }
#endif
#endif
  }
};
// lint -restore

#endif
