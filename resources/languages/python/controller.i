// Copyright 1996-2018 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************************************/
/* Description:  Swig interface which maps the OO C++ files into a python module called "controller"   */
/*******************************************************************************************************/

%module controller

%begin %{
#define SWIG_PYTHON_2_UNICODE
%}

%{
#include <webots/Accelerometer.hpp>
#include <webots/Brake.hpp>
#include <webots/Camera.hpp>
#include <webots/camera_recognition_object.h>
#include <webots/Compass.hpp>
#include <webots/Connector.hpp>
#include <webots/Device.hpp>
#include <webots/DifferentialWheels.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Lidar.hpp>
#include <webots/lidar_point.h>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Mouse.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Node.hpp>
#include <webots/Pen.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Radar.hpp>
#include <webots/radar_target.h>
#include <webots/RangeFinder.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Skin.hpp>
#include <webots/Speaker.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp>

using namespace std;
%}

//----------------------------------------------------------------------------------------------
//  Miscellaneous - controller module's level
//----------------------------------------------------------------------------------------------

//handling std::string
%include "std_string.i"

// manage double arrays
%typemap(out) const double * {
  int len = 3;
  string test("$name");
  if (test == "getSFVec2f" || test == "getMFVec2f")
    len = 2;
  else if (test == "getSFRotation")
    len = 4;
  else if (test == "getVelocity")
    len = 6;
  else if (test == "getOrientation" || test == "virtualRealityHeadsetGetOrientation")
    len = 9;
  $result = PyList_New(len);
  for (int i = 0; i < len; ++i)
    PyList_SetItem($result, i, PyFloat_FromDouble($1[i]));
}
%typemap(in) const double [ANY] {
  if (!PyList_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList'\n");
    return NULL;
  }
  int len = PyList_Size($input);
  $1 = (double*)malloc(len * sizeof(double));
  for (int i = 0; i < len; ++i)
    $1[i] = PyFloat_AsDouble(PyList_GetItem($input, i));
}
%typemap(in) const int * {
  if (!PyList_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList'\n");
    return NULL;
  }
  int len = PyList_Size($input);
  $1 = (int*)malloc(len * sizeof(int));
  for (int i = 0; i < len; ++i)
    $1[i] = PyInt_AsLong(PyList_GetItem($input, i));
}

//----------------------------------------------------------------------------------------------
//  Device
//----------------------------------------------------------------------------------------------

%include <webots/Device.hpp>

//----------------------------------------------------------------------------------------------
//  Accelerometer
//----------------------------------------------------------------------------------------------

%include <webots/Accelerometer.hpp>

//----------------------------------------------------------------------------------------------
//  Brake
//----------------------------------------------------------------------------------------------

%include <webots/Brake.hpp>

%extend webots::Brake {
  %pythoncode %{
  def getMotor(self):
      try:
          return self.__motor
      except AttributeError:
          self.__motor = Robot.getDevice(self.getMotorTag())
          return self.__motor
  def getPositionSensor(self):
      try:
          return self.instance
      except AttributeError:
          self.__positionSensor = Robot.getDevice(self.getPositionSensorTag())
          return self.__positionSensor
  %}
}

//----------------------------------------------------------------------------------------------
//  Camera
//----------------------------------------------------------------------------------------------

%rename WbCameraRecognitionObject CameraRecognitionObject;



%include <webots/camera_recognition_object.h>

%extend WbCameraRecognitionObject {
  PyObject *get_position() {
    const double *position = $self->position;
    PyObject *ret = PyList_New(3);
    PyList_SetItem(ret, 0, PyFloat_FromDouble(position[0]));
    PyList_SetItem(ret, 1, PyFloat_FromDouble(position[1]));
    PyList_SetItem(ret, 2, PyFloat_FromDouble(position[2]));
    return ret;
  }
  PyObject *get_orientation() {
    const double *orientation = $self->orientation;
    PyObject *ret = PyList_New(4);
    PyList_SetItem(ret, 0, PyFloat_FromDouble(orientation[0]));
    PyList_SetItem(ret, 1, PyFloat_FromDouble(orientation[1]));
    PyList_SetItem(ret, 2, PyFloat_FromDouble(orientation[2]));
    PyList_SetItem(ret, 3, PyFloat_FromDouble(orientation[3]));
    return ret;
  }
  PyObject *get_size() {
    const double *size = $self->size;
    PyObject *ret = PyList_New(2);
    PyList_SetItem(ret, 0, PyInt_FromLong(size[0]));
    PyList_SetItem(ret, 1, PyInt_FromLong(size[1]));
    return ret;
  }
  PyObject *get_position_on_image() {
    const int *position_on_image = $self->position_on_image;
    PyObject *ret = PyList_New(2);
    PyList_SetItem(ret, 0, PyInt_FromLong(position_on_image[0]));
    PyList_SetItem(ret, 1, PyInt_FromLong(position_on_image[1]));
    return ret;
  }
  PyObject *get_size_on_image() {
    const int *size_on_image = $self->size_on_image;
    PyObject *ret = PyList_New(2);
    PyList_SetItem(ret, 0, PyInt_FromLong(size_on_image[0]));
    PyList_SetItem(ret, 1, PyInt_FromLong(size_on_image[1]));
    return ret;
  }
  PyObject *get_colors() {
    const double *colors = $self->colors;
    const int number_of_color = $self->number_of_colors;
    PyObject *ret = PyList_New(3 * number_of_color);
    for (int i = 0; i < 3 * number_of_color; ++i)
      PyList_SetItem(ret, i, PyFloat_FromDouble(colors[i]));
    return ret;
  }
  PyObject *get_id() {
    return PyInt_FromLong($self->id);
  }
  PyObject *get_number_of_colors() {
    return PyInt_FromLong($self->number_of_colors);
  }
  PyObject *get_model() {
    return PyBytes_FromStringAndSize($self->model, strlen($self->model));
  }
};


%typemap(out) unsigned char * {
  int width = arg1->getWidth();
  int height = arg1->getHeight();
  if ($1)
    $result = PyBytes_FromStringAndSize((const char*)$1, 4 * width * height);
  else
    $result = Py_None;

}

%extend webots::Camera {
  PyObject *getImageArray() {
    const unsigned char *im = $self->getImage();
    int width = $self->getWidth();
    int height = $self->getHeight();
    PyObject *ret = Py_None;
    if (im) {
      ret = PyList_New(width);
      for (int x = 0; x < width; ++x) {
        PyObject *dim2 = PyList_New(height);
        PyList_SetItem(ret, x, dim2);
        for (int y = 0; y < height; ++y) {
          PyObject *dim3 = PyList_New(3);
          PyList_SetItem(dim2, y, dim3);
          for (int ch = 0; ch < 3; ++ch)
            PyList_SetItem(dim3, ch, PyInt_FromLong((unsigned int)(im[4 * (x + y * width) + 2 - ch])));
        }
      }
    }
    return ret;
  }

  static PyObject *imageGetRed(PyObject *im, int width, int x, int y) {
    if (!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetRed', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char *s = (unsigned char*)PyString_AsString(im);
    return PyInt_FromLong(s[4 * (y * width + x) + 2]);
  }

  static PyObject *imageGetGreen(PyObject *im, int width, int x, int y) {
    if (!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetGreen', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char *s = (unsigned char*)PyString_AsString(im);
    return PyInt_FromLong(s[4 * (y * width + x) + 1]);
  }

  static PyObject *imageGetBlue(PyObject *im, int width, int x, int y) {
    if (!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetBlue', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char *s = (unsigned char*)PyString_AsString(im);
    return PyInt_FromLong(s[4 * (y * width + x)]);
  }

  static PyObject *imageGetGray(PyObject *im, int width, int x, int y) {
    if (!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetGrey', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char *s = (unsigned char*)PyString_AsString(im);
    return PyInt_FromLong((s[4 * (y * width + x)] + s[4 * (y * width + x) + 1] + s[4 * (y * width + x) + 2]) / 3);
  }

  static PyObject *imageGetGrey(PyObject *im, int width, int x, int y) {
    if (!PyString_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_imageGetGrey', argument 2 of type 'PyString'\n");
      return NULL;
    }
    unsigned char *s = (unsigned char*)PyString_AsString(im);
    return PyInt_FromLong((s[4 * (y * width + x)] + s[4 * (y * width + x) + 1] + s[4 * (y * width + x) + 2]) / 3);
  }

  webots::CameraRecognitionObject getRecognitionObject(int index) const {
    const webots::CameraRecognitionObject *objects = $self->getRecognitionObjects();
    return objects[index];
  }

  %pythoncode %{
  def getRecognitionObjects(self):
     ret = []
     for i in range(self.getRecognitionNumberOfObjects()):
       ret.append(self.getRecognitionObject(i))
     return ret
  %}
};

%include <webots/Camera.hpp>

%typemap(out) unsigned char *;

//----------------------------------------------------------------------------------------------
//  Compass
//----------------------------------------------------------------------------------------------

%include <webots/Compass.hpp>

//----------------------------------------------------------------------------------------------
//  Connector
//----------------------------------------------------------------------------------------------

%include <webots/Connector.hpp>

//----------------------------------------------------------------------------------------------
//  Display
//----------------------------------------------------------------------------------------------

%typemap(in) const void * {
  if (!PyList_Check($input) && !PyString_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList' or 'PyString'\n");
    return NULL;
  }
  if (PyList_Check($input)) {
    int len1 = PyList_Size($input);
    PyObject *l2 = PyList_GetItem($input, 0);
    if (!PyList_Check(l2)) {
      PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList' of 'PyList'\n");
      return NULL;
    }
    int len2 = PyList_Size(l2);
    PyObject *l3 = PyList_GetItem(l2, 0);
    if (!PyList_Check(l3)) {
      PyErr_SetString(PyExc_TypeError, "in method '$name', expected 'PyList' of 'PyList' of 'PyList'\n");
      return NULL;
    }
    int len3 = PyList_Size(l3);
    $1 = (void *)malloc(len1 * len2 * len3 * sizeof(unsigned char));
    for (int i = 0; i < len1; ++i)
      for (int j = 0; j < len2; ++j)
        for (int k = 0; k < len3; ++k)
          ((unsigned char *)$1)[(i * len2 * len3) + (j * len3) + k] = (unsigned char) PyInt_AsLong(PyList_GetItem(PyList_GetItem(PyList_GetItem($input, i), j), k));
  } else // PyString case
    $1 = PyString_AsString($input);
}

%rename (__internalImageNew) imageNew(int width, int height, const void *data, int format) const;
%rename (__internalDrawPolygon) drawPolygon(const int *x, const int *y, int size);
%rename (__internalFillPolygon) fillPolygon(const int *x, const int *y, int size);

%extend webots::Display {
  %pythoncode %{
    def imageNew(self, data, format, width=None, height=None):
      if isinstance(data, list):
        return self.__internalImageNew(len(data), len(data[0]), data, format)
      elif width is None or height is None:
        raise TypeError('imageNew : width and height must be given if data is not a list')
      else:
        return self.__internalImageNew(width, height, data, format)
    def drawPolygon(self, x, y):
      self.__internalDrawPolygon(x, y, min(len(x), len(y)))
    def fillPolygon(self, x, y):
      self.__internalFillPolygon(x, y, min(len(x), len(y)))
  %}
}

%include <webots/ImageRef.hpp>
%include <webots/Display.hpp>

//----------------------------------------------------------------------------------------------
//  Distance sensor
//----------------------------------------------------------------------------------------------

%include <webots/DistanceSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Emitter
//----------------------------------------------------------------------------------------------

%typemap(in) (const void *data, int size) {
  $1 = PyString_AsString($input);
  $2 = PyString_Size($input);
}

%include <webots/Emitter.hpp>

%typemap(in) (const void *data, int size);

//----------------------------------------------------------------------------------------------
//  Field
//----------------------------------------------------------------------------------------------

%ignore webots::Field::findField(WbFieldRef ref);
%ignore webots::Field::cleanup();

%include <webots/Field.hpp>

//----------------------------------------------------------------------------------------------
//  GPS
//----------------------------------------------------------------------------------------------

%include <webots/GPS.hpp>

//----------------------------------------------------------------------------------------------
//  Gyro
//----------------------------------------------------------------------------------------------

%include <webots/Gyro.hpp>

//----------------------------------------------------------------------------------------------
//  InertialUnit
//----------------------------------------------------------------------------------------------

%include <webots/InertialUnit.hpp>

//----------------------------------------------------------------------------------------------
//  Joystick
//----------------------------------------------------------------------------------------------

%include <webots/Joystick.hpp>

//----------------------------------------------------------------------------------------------
//  Keyboard
//----------------------------------------------------------------------------------------------

%include <webots/Keyboard.hpp>

//----------------------------------------------------------------------------------------------
//  LED
//----------------------------------------------------------------------------------------------

%include <webots/LED.hpp>

//----------------------------------------------------------------------------------------------
//  Lidar
//----------------------------------------------------------------------------------------------

%rename WbLidarPoint LidarPoint;

%include <webots/lidar_point.h>

%typemap(out) float * {
  int width = arg1->getHorizontalResolution();
  int height = arg1->getNumberOfLayers();
  int len = width * height;
  if ($1) {
    $result = PyList_New(len);
    for (int x = 0; x < len; ++x)
      PyList_SetItem($result, x, PyFloat_FromDouble($1[x]));
  } else
    $result = Py_None;
}

%ignore webots::Lidar::getPointCloud();
%ignore webots::Lidar::getLayerPointCloud();

%extend webots::Lidar {

  webots::LidarPoint getPoint(int index) const {
    const webots::LidarPoint *point = $self->getPointCloud();
    return point[index];
  }

  webots::LidarPoint getLayerPoint(int layer, int index) const {
    const webots::LidarPoint *point = $self->getLayerPointCloud(layer);
    return point[index];
  }

  %pythoncode %{
  def getPointCloud(self):
     ret = []
     for i in range(self.getNumberOfPoints()):
       ret.append(self.getPoint(i))
     return ret

  def getLayerPointCloud(self, layer):
     ret = []
     for i in range(self.getHorizontalResolution()):
       ret.append(self.getLayerPoint(layer, i))
     return ret
  %}

  PyObject *getRangeImageArray() {
    const float *im = $self->getRangeImage();
    int width = $self->getHorizontalResolution();
    int height = $self->getNumberOfLayers();
    PyObject *ret = Py_None;
    if (im) {
      ret = PyList_New(width);
      for (int x = 0; x < width; ++x) {
        PyObject *dim2 = PyList_New(height);
        PyList_SetItem(ret, x, dim2);
        for (int y = 0; y < height; ++y) {
          PyObject *v = PyFloat_FromDouble(im[x + y * width]);
          PyList_SetItem(dim2, y, v);
        }
      }
    }
    return ret;
  }
};

%include <webots/Lidar.hpp>

%typemap(out) float *;

//----------------------------------------------------------------------------------------------
//  LightSensor
//----------------------------------------------------------------------------------------------

%include <webots/LightSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Motion
//----------------------------------------------------------------------------------------------

%include <webots/utils/Motion.hpp>

//----------------------------------------------------------------------------------------------
//  Motor
//----------------------------------------------------------------------------------------------

%include <webots/Motor.hpp>

%extend webots::Motor {
  %pythoncode %{
  def getBrake(self):
      try:
          return self.__brake
      except AttributeError:
          self.__brake = Robot.getDevice(self.getBrakeTag())
          return self.__brake
  def getPositionSensor(self):
      try:
          return self.__positionSensor
      except AttributeError:
          self.__positionSensor = Robot.getDevice(self.getPositionSensorTag())
          return self.__positionSensor
  %}
}

//----------------------------------------------------------------------------------------------
//  Mouse
//----------------------------------------------------------------------------------------------

%rename WbMouseState MouseState;

%include <webots/mouse_state.h>

%include <webots/Mouse.hpp>

//----------------------------------------------------------------------------------------------
//  Node
//----------------------------------------------------------------------------------------------

%ignore webots::Node::findNode(WbNodeRef ref);
%ignore webots::Node::cleanup();

%extend webots::Node {
  %pythoncode %{
  def __eq__(self, other):
      if self is None and other is None:
          return True
      elif self is None or other is None:
          return False
      elif self.getId() == other.getId():
          return True
      return False

  def __ne__(self, other):
      return not self.__eq__(other)
  %}
};

%include <webots/Node.hpp>

//----------------------------------------------------------------------------------------------
//  Pen
//----------------------------------------------------------------------------------------------

%include <webots/Pen.hpp>

//----------------------------------------------------------------------------------------------
//  PositionSensor
//----------------------------------------------------------------------------------------------

%include <webots/PositionSensor.hpp>

%extend webots::PositionSensor {
  %pythoncode %{
  def getBrake(self):
      try:
          return self.__brake
      except AttributeError:
          self.__brake = Robot.getDevice(self.getBrakeTag())
          return self.__brake
  def getMotor(self):
      try:
          return self.__motor
      except AttributeError:
          self.__motor = Robot.getDevice(self.getMotorTag())
          return self.__motor
  %}
}

//----------------------------------------------------------------------------------------------
//  Radar
//----------------------------------------------------------------------------------------------

%rename WbRadarTarget RadarTarget;

%include <webots/radar_target.h>

%ignore webots::Lidar::getTargets();

%extend webots::Radar {
  webots::RadarTarget getTarget(int index) const {
    const webots::RadarTarget *targets = $self->getTargets();
    return targets[index];
  }

  %pythoncode %{
  def getTargets(self):
     ret = []
     for i in range(self.getNumberOfTargets()):
       ret.append(self.getTarget(i))
     return ret
  %}
};

%include <webots/Radar.hpp>

//----------------------------------------------------------------------------------------------
//  RangeFinder
//----------------------------------------------------------------------------------------------

%typemap(out) float * {
  int width = arg1->getWidth();
  int height = arg1->getHeight();
  int len = width * height;
  if ($1) {
    $result = PyList_New(len);
    for (int x = 0; x < len; ++x)
      PyList_SetItem($result, x, PyFloat_FromDouble($1[x]));
  } else
    $result = Py_None;
}

%extend webots::RangeFinder {

  PyObject *getRangeImageArray() {
    const float *im = $self->getRangeImage();
    int width = $self->getWidth();
    int height = $self->getHeight();
    PyObject *ret = Py_None;
    if (im) {
      PyObject *ret = PyList_New(width);
      for (int x = 0; x < width; ++x) {
        PyObject *dim2 = PyList_New(height);
        PyList_SetItem(ret, x, dim2);
        for (int y = 0; y < height; ++y) {
          PyObject *v = PyFloat_FromDouble(im[x + y * width]);
          PyList_SetItem(dim2, y, v);
        }
      }
    }
    return ret;
  }

  static PyObject *rangeImageGetValue(PyObject *im, double minRange, double maxRange, int width, int x, int y) {
    if (!PyList_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList'\n");
      return NULL;
    }
    PyObject *value = PyList_GetItem(im, y * width + x);
    if (!PyFloat_Check(value)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList' of 'PyFloat'\n");
      return NULL;
    }
    fprintf(stderr, "Warning: Camera.rangeImageGetValue is deprecated, please use Camera.rangeImageGetDepth instead\n");
    return value;
  }

  static PyObject *rangeImageGetDepth(PyObject *im, int width, int x, int y) {
    if (!PyList_Check(im)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList'\n");
      return NULL;
    }
    PyObject *value = PyList_GetItem(im, y * width + x);
    if (!PyFloat_Check(value)) {
      PyErr_SetString(PyExc_TypeError, "in method 'Camera_rangeImageGetValue', argument 2 of type 'PyList' of 'PyFloat'\n");
      return NULL;
    }
    return value;
  }
};

%include <webots/RangeFinder.hpp>

%typemap(out) float *;

//----------------------------------------------------------------------------------------------
//  Receiver
//----------------------------------------------------------------------------------------------

%typemap(out) const void * {
  $result = PyBytes_FromStringAndSize((const char*) $1, arg1->getDataSize());
}

%include <webots/Receiver.hpp>

%typemap(out) const void *;

//----------------------------------------------------------------------------------------------
//  Skin
//----------------------------------------------------------------------------------------------

%include <webots/Skin.hpp>

//----------------------------------------------------------------------------------------------
//  Speaker
//----------------------------------------------------------------------------------------------

%include <webots/Speaker.hpp>

//----------------------------------------------------------------------------------------------
//  TouchSensor
//----------------------------------------------------------------------------------------------

%include <webots/TouchSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Robot
//----------------------------------------------------------------------------------------------

%ignore webots::Robot::getAccelerometer(const std::string &name);
%ignore webots::Robot::getBrake(const std::string &name);
%ignore webots::Robot::getCamera(const std::string &name);
%ignore webots::Robot::getCompass(const std::string &name);
%ignore webots::Robot::getConnector(const std::string &name);
%ignore webots::Robot::getDisplay(const std::string &name);
%ignore webots::Robot::getDistanceSensor(const std::string &name);
%ignore webots::Robot::getEmitter(const std::string &name);
%ignore webots::Robot::getGPS(const std::string &name);
%ignore webots::Robot::getGyro(const std::string &name);
%ignore webots::Robot::getInertialUnit(const std::string &name);
%ignore webots::Robot::getJoystick();
%ignore webots::Robot::getKeyboard();
%ignore webots::Robot::getLED(const std::string &name);
%ignore webots::Robot::getLidar(const std::string &name);
%ignore webots::Robot::getLightSensor(const std::string &name);
%ignore webots::Robot::getMotor(const std::string &name);
%ignore webots::Robot::getMouse();
%ignore webots::Robot::getPen(const std::string &name);
%ignore webots::Robot::getPositionSensor(const std::string &name);
%ignore webots::Robot::getRadar(const std::string &name);
%ignore webots::Robot::getRangeFinder(const std::string &name);
%ignore webots::Robot::getReceiver(const std::string &name);
%ignore webots::Robot::getSkin(const std::string &name);
%ignore webots::Robot::getSpeaker(const std::string &name);
%ignore webots::Robot::getTouchSensor(const std::string &name);
%ignore webots::Robot::windowCustomFunction(void *arg);
%ignore webots::Robot::wwiSend(const char *data, int size);
%ignore webots::Robot::wwiReceive(int *size);

%rename ("__internalGetDevice") webots::Robot::getDevice;
%rename ("__internalGetDeviceByIndex") webots::Robot::getDeviceByIndex;
%rename ("__internalGetDeviceTypeFromTag") webots::Robot::getDeviceTypeFromTag;
%rename ("__internalGetDeviceNameFromTag") webots::Robot::getDeviceNameFromTag;
%rename ("__internalGetDeviceTagFromIndex") webots::Robot::getDeviceTagFromIndex;
%rename ("__internalGetDeviceTagFromName") webots::Robot::getDeviceTagFromName;

%extend webots::Robot {
  %pythoncode %{
    __devices = []
    joystick = Joystick()
    keyboard = Keyboard()
    mouse = Mouse()
    def createAccelerometer(self, name):
      return Accelerometer(name)
    def getAccelerometer(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.ACCELEROMETER):
        return None
      return self.__getOrCreateDevice(tag)
    def createBrake(self, name):
      return Brake(name)
    def getBrake(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.BRAKE):
        return None
      return self.__getOrCreateDevice(tag)
    def createCamera(self, name):
      return Camera(name)
    def getCamera(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.CAMERA):
        return None
      return self.__getOrCreateDevice(tag)
    def createCompass(self, name):
      return Compass(name)
    def getCompass(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.COMPASS):
        return None
      return self.__getOrCreateDevice(tag)
    def createConnector(self, name):
      return Connector(name)
    def getConnector(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.CONNECTOR):
        return None
      return self.__getOrCreateDevice(tag)
    def createDisplay(self, name):
      return Display(name)
    def getDisplay(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.DISPLAY):
        return None
      return self.__getOrCreateDevice(tag)
    def createDistanceSensor(self, name):
      return DistanceSensor(name)
    def getDistanceSensor(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.DISTANCE_SENSOR):
        return None
      return self.__getOrCreateDevice(tag)
    def createEmitter(self, name):
      return Emitter(name)
    def getEmitter(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.EMITTER):
        return None
      return self.__getOrCreateDevice(tag)
    def createGPS(self, name):
      return GPS(name)
    def getGPS(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.GPS):
        return None
      return self.__getOrCreateDevice(tag)
    def createGyro(self, name):
      return Gyro(name)
    def getGyro(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.GYRO):
        return None
      return self.__getOrCreateDevice(tag)
    def createInertialUnit(self, name):
      return InertialUnit(name)
    def getInertialUnit(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.INERTIAL_UNIT):
        return None
      return self.__getOrCreateDevice(tag)
    def getJoystick(self):
      return self.joystick
    def getKeyboard(self):
      return self.keyboard
    def createLED(self, name):
      return LED(name)
    def getLED(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.LED):
        return None
      return self.__getOrCreateDevice(tag)
    def createLidar(self, name):
      return Lidar(name)
    def getLidar(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.LIDAR):
        return None
      return self.__getOrCreateDevice(tag)
    def createLightSensor(self, name):
      return LightSensor(name)
    def getLightSensor(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.LIGHT_SENSOR):
        return None
      return self.__getOrCreateDevice(tag)
    def createMotor(self, name):
      return Motor(name)
    def getMotor(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.LINEAR_MOTOR) and not Device.hasType(tag, Node.ROTATIONAL_MOTOR):
        return None
      return self.__getOrCreateDevice(tag)
    def getMouse(self):
      return self.mouse
    def createPen(self, name):
      return Pen(name)
    def getPen(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.PEN):
        return None
      return self.__getOrCreateDevice(tag)
    def createPositionSensor(self, name):
      return PositionSensor(name)
    def getPositionSensor(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.POSITION_SENSOR):
        return None
      return self.__getOrCreateDevice(tag)
    def createRadar(self, name):
      return Radar(name)
    def getRadar(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.RADAR):
        return None
      return self.__getOrCreateDevice(tag)
    def createRangeFinder(self, name):
      return RangeFinder(name)
    def getRangeFinder(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.RANGE_FINDER):
        return None
      return self.__getOrCreateDevice(tag)
    def createReceiver(self, name):
      return Receiver(name)
    def getReceiver(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.RECEIVER):
        return None
      return self.__getOrCreateDevice(tag)
    def createSkin(self, name):
      return Skin(name)
    def getSkin(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.SKIN):
        return None
      return self.__getOrCreateDevice(tag)
    def createSpeaker(self, name):
      return Speaker(name)
    def getSpeaker(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.SPEAKER):
        return None
      return self.__getOrCreateDevice(tag)
    def createTouchSensor(self, name):
      return TouchSensor(name)
    def getTouchSensor(self, name):
      tag = self.__internalGetDeviceTagFromName(name)
      if not Device.hasType(tag, Node.TOUCH_SENSOR):
        return None
      return self.__getOrCreateDevice(tag)
    def getDeviceByIndex(self, index):
      tag = self.__internalGetDeviceTagFromIndex(index)
      return self.__getOrCreateDevice(tag)
    @staticmethod
    def getDevice(tag):
      if tag == 0:
          return None
      size = len(Robot.__devices)
      if size == 0 or tag >= size:
          return None
      return Robot.__devices[tag]
    def __getOrCreateDevice(self, tag):
      if tag == 0:
          return None
      size = len(Robot.__devices)
      if size > 0:
          if tag >= size:
              return None
          return Robot.__devices[tag]

      # initialize Robot.__devices list
      count = self.getNumberOfDevices()
      Robot.__devices = [None] * (count + 1)
      for i in range(0, count):
          otherTag = self.__internalGetDeviceTagFromIndex(i)
          name = self.__internalGetDeviceNameFromTag(otherTag)
          nodeType = self.__internalGetDeviceTypeFromTag(otherTag)
          if nodeType == Node.ACCELEROMETER:
              Robot.__devices[otherTag] = self.createAccelerometer(name)
          elif nodeType == Node.BRAKE:
              Robot.__devices[otherTag] = self.createBrake(name)
          elif nodeType == Node.CAMERA:
              Robot.__devices[otherTag] = self.createCamera(name)
          elif nodeType == Node.COMPASS:
              Robot.__devices[otherTag] = self.createCompass(name)
          elif nodeType == Node.CONNECTOR:
              Robot.__devices[otherTag] = self.createConnector(name)
          elif nodeType == Node.DISPLAY:
              Robot.__devices[otherTag] = self.createDisplay(name)
          elif nodeType == Node.DISTANCE_SENSOR:
              Robot.__devices[otherTag] = self.createDistanceSensor(name)
          elif nodeType == Node.EMITTER:
              Robot.__devices[otherTag] = self.createEmitter(name)
          elif nodeType == Node.GPS:
              Robot.__devices[otherTag] = self.createGPS(name)
          elif nodeType == Node.GYRO:
              Robot.__devices[otherTag] = self.createGyro(name)
          elif nodeType == Node.INERTIAL_UNIT:
              Robot.__devices[otherTag] = self.createInertialUnit(name)
          elif nodeType == Node.LED:
              Robot.__devices[otherTag] = self.createLED(name)
          elif nodeType == Node.LIDAR:
              Robot.__devices[otherTag] = self.createLidar(name)
          elif nodeType == Node.LIGHT_SENSOR:
              Robot.__devices[otherTag] = self.createLightSensor(name)
          elif nodeType == Node.LINEAR_MOTOR or nodeType == Node.ROTATIONAL_MOTOR:
              Robot.__devices[otherTag] = self.createMotor(name)
          elif nodeType == Node.PEN:
              Robot.__devices[otherTag] = self.createPen(name)
          elif nodeType == Node.POSITION_SENSOR:
              Robot.__devices[otherTag] = self.createPositionSensor(name)
          elif nodeType == Node.RADAR:
              Robot.__devices[otherTag] = self.createRadar(name)
          elif nodeType == Node.RANGE_FINDER:
              Robot.__devices[otherTag] = self.createRangeFinder(name)
          elif nodeType == Node.RECEIVER:
              Robot.__devices[otherTag] = self.createReceiver(name)
          elif nodeType == Node.SPEAKER:
              Robot.__devices[otherTag] = self.createSpeaker(name)
          elif nodeType == Node.TOUCH_SENSOR:
              Robot.__devices[otherTag] = self.createTouchSensor(name)
      return Robot.__devices[tag]
  %}
}

%include <webots/Robot.hpp>

//----------------------------------------------------------------------------------------------
//  DifferentialWheels
//----------------------------------------------------------------------------------------------

%include <webots/DifferentialWheels.hpp>

//----------------------------------------------------------------------------------------------
//  Supervisor
//----------------------------------------------------------------------------------------------

%include <webots/Supervisor.hpp>
