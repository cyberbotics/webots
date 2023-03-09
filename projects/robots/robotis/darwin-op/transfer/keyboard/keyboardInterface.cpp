// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  Class used to read keyboard input with X11
 * Author:       david.mansolino@epfl.ch
 */

#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xos.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <stdio.h>
#include <stdlib.h>

#include "keyboardInterface.hpp"

Display *display;
Window window;
XEvent Report;
GC gc;
unsigned int width = 500;
unsigned int height = 250;

Window root_return;
int x_return, y_return;
unsigned int border_width_return;
unsigned int depth_return;

const char text1[128] = "This window is used to catch keyboard input by the controller.";
const char text2[128] = "Please do not close it and do not unfocus it.";
const char text3[128] = "It will be closed automatically when the controller terminates.";

int SpecialKey[10] = {0,
                      WB_KEYBOARD_HOME,
                      WB_KEYBOARD_LEFT,
                      WB_KEYBOARD_UP,
                      WB_KEYBOARD_RIGHT,
                      WB_KEYBOARD_DOWN,
                      WB_KEYBOARD_PAGEUP,
                      WB_KEYBOARD_PAGEDOWN,
                      WB_KEYBOARD_END};

KeyboardInterface::KeyboardInterface() {
  initialiseKeyPressed();
}

void KeyboardInterface::initialiseKeyPressed() {
  for (int c = 0; c < NKEYS; c++)
    mKeyPressed[c] = 0;
}

void KeyboardInterface::resetKeyPressed() {
  for (int c = 0; c < NKEYS; c++) {
    if (mKeyPressed[c] == PRESSED)
      mKeyPressed[c] = 1;
    else if (mKeyPressed[c] == VALIDATED_STILL_PRESSED)
      mKeyPressed[c] = 1;
    else
      mKeyPressed[c] = 0;
  }
}

void KeyboardInterface::createWindow() {
  // Open a new Window
  display = XOpenDisplay(NULL);
  window = XCreateSimpleWindow(display, RootWindow(display, 0), 1, 1, width, height, 0, BlackPixel(display, 0),
                               WhitePixel(display, 0));
  XStoreName(display, window, "Webots Cross-Compilation : Keyboard inputs");  // Set window title
  gc = XCreateGC(display, window, 0, NULL);  // this variable will contain the handle to the returned graphics context.

  // Set window max and min size
  XSizeHints *normal_hints = XAllocSizeHints();
  normal_hints->min_width = 400;
  normal_hints->max_width = 1000;
  normal_hints->min_height = 100;
  normal_hints->max_height = 600;
  normal_hints->flags = PMaxSize | PMinSize;
  XSetWMSizeHints(display, window, normal_hints, XInternAtom(display, "WM_NORMAL_HINTS", 0));

  // Show window
  XMapWindow(display, window);
  XFlush(display);

  // Could be used to add an image
  Pixmap bitmap;                             // this variable will contain the ID of the newly created pixmap.
  unsigned int bitmap_width, bitmap_height;  // these variables will contain the dimensions of the loaded bitmap.
  int hotspot_x, hotspot_y;                  // these variables will contain the location of the hot-spot of the loaded bitmap.
  int rc = XReadBitmapFile(display, window, "/home/david/X11-3/keyboard/keyboard.xbm", &bitmap_width, &bitmap_height, &bitmap,
                           &hotspot_x, &hotspot_y);
  if (rc == BitmapSuccess)
    XCopyPlane(display, bitmap, window, gc, 0, 0, bitmap_width, bitmap_height, 0, 0, 1);

  // Set text in the window
  XDrawString(display, window, gc, 10 + (width - 400) / 2, height / 2 - 30, text1, 62);
  XDrawString(display, window, gc, 70 + (width - 400) / 2, height / 2, text2, 45);
  XDrawString(display, window, gc, 10 + (width - 400) / 2, height / 2 + 30, text3, 63);

  /* flush all pending requests to the X server. */
  XFlush(display);
  XSync(display, False);

  // Select Events
  XSelectInput(display, window, ExposureMask | KeyPressMask | KeyReleaseMask);
}

void KeyboardInterface::closeWindow() {
  XDestroyWindow(display, window);
  XCloseDisplay(display);
}

void KeyboardInterface::startListenKeyboard() {
  while (1) {
    XNextEvent(display, &Report);
    switch (Report.type) {
      case Expose: {
        XGetGeometry(display, window, &root_return, &x_return, &y_return, &width, &height, &border_width_return, &depth_return);
        XClearWindow(display, window);
        XDrawString(display, window, gc, 10 + (width - 400) / 2, height / 2 - 30, text1, 63);
        XDrawString(display, window, gc, 70 + (width - 400) / 2, height / 2, text2, 44);
        XDrawString(display, window, gc, 10 + (width - 400) / 2, height / 2 + 30, text3, 62);
        XFlush(display);
        XSync(display, False);
      } break;

      case KeyPress:
        if (XLookupKeysym(&Report.xkey, 0) < 123)
          setKeyPressed(XLookupKeysym(&Report.xkey, 0));
        else {
          switch (XLookupKeysym(&Report.xkey, 0)) {
            case XK_Home:
              setKeyPressed(1);
              break;
            case XK_Left:
              setKeyPressed(2);
              break;
            case XK_Up:
              setKeyPressed(3);
              break;
            case XK_Right:
              setKeyPressed(4);
              break;
            case XK_Down:
              setKeyPressed(5);
              break;
            case XK_Page_Up:
              setKeyPressed(6);
              break;
            case XK_Page_Down:
              setKeyPressed(7);
              break;
            case XK_End:
              setKeyPressed(8);
              break;
          }
        }
        break;

      case KeyRelease:
        if (XLookupKeysym(&Report.xkey, 0) < 123)
          setKeyReleased(XLookupKeysym(&Report.xkey, 0));
        else {
          switch (XLookupKeysym(&Report.xkey, 0)) {
            case XK_Home:
              setKeyReleased(1);
              break;
            case XK_Left:
              setKeyReleased(2);
              break;
            case XK_Up:
              setKeyReleased(3);
              break;
            case XK_Right:
              setKeyReleased(4);
              break;
            case XK_Down:
              setKeyReleased(5);
              break;
            case XK_Page_Up:
              setKeyReleased(6);
              break;
            case XK_Page_Down:
              setKeyReleased(7);
              break;
            case XK_End:
              setKeyReleased(8);
              break;
          }
        }
        break;
    }
  }
}

void KeyboardInterface::setKeyPressed(int key) {
  if (mKeyPressed[key] == VALIDATED)
    mKeyPressed[key] = VALIDATED_STILL_PRESSED;
  else if (mKeyPressed[key] == VALIDATED_STILL_PRESSED)
    mKeyPressed[key] = VALIDATED_STILL_PRESSED;
  else
    mKeyPressed[key] = PRESSED;
}

void KeyboardInterface::setKeyReleased(int key) {
  if (mKeyPressed[key] == PRESSED)
    mKeyPressed[key] = PRESSED_AND_RELEASE;
  else if (mKeyPressed[key] == VALIDATED_STILL_PRESSED)
    mKeyPressed[key] = VALIDATED;
}

int KeyboardInterface::getKeyPressed() {
  for (int c = 0; c < NKEYS; c++) {
    if (mKeyPressed[c] == PRESSED) {
      mKeyPressed[c] = VALIDATED_STILL_PRESSED;
      if (c > 0 && c < 10)  // 1->10 for special caracters
        return SpecialKey[c];
      else if (c > 96 && c < 123)  // Always return ascii caracter of UpperCase
        return (c - 32);
      else
        return c;
    } else if (mKeyPressed[c] == PRESSED_AND_RELEASE) {
      mKeyPressed[c] = VALIDATED;
      if (c > 0 && c < 10)  // 1->10 for special caracters
        return SpecialKey[c];
      else if (c > 96 && c < 123)  // Always return ascii caracter of UpperCase
        return (c - 32);
      else
        return c;
    }
  }
  return -1;
}
