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

/*******************************************************************************************************/
/* Description:  Swig interface which maps the OO C++ files into a java package called "controller"    */
/*******************************************************************************************************/

%module wrapper

%{
#include <webots/Accelerometer.hpp>
#include <webots/Altimeter.hpp>
#include <webots/Brake.hpp>
#include <webots/Camera.hpp>
#include <webots/camera_recognition_object.h>
#include <webots/contact_point.h>
#include <webots/Connector.hpp>
#include <webots/Compass.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/ImageRef.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Lidar.hpp>
#include <webots/lidar_point.h>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Mouse.hpp>
#include <webots/Node.hpp>
#include <webots/Pen.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/radar_target.h>
#include <webots/Radar.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Skin.hpp>
#include <webots/Speaker.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/VacuumGripper.hpp>
#include <webots/utils/Motion.hpp>

using namespace std;
%}

//----------------------------------------------------------------------------------------------
//  Miscellaneous - controller module's level
//----------------------------------------------------------------------------------------------

//for the conversion between array and pointer
%include "typemaps.i"
%include "arrays_java.i"

%javamethodmodifiers getLookupTableSize "private"

// general typemap for determining the size of output double array
%typemap(out) double [] {
  const string test("$name");
  if (test == "getSFVec2f" || test == "getMFVec2f")
    $result = SWIG_JavaArrayOutDouble(jenv, $1, 2);
  else if (test == "getSFRotation" || test == "getQuaternion")
    $result = SWIG_JavaArrayOutDouble(jenv, $1, 4);
  else if (test == "getVelocity")
    $result = SWIG_JavaArrayOutDouble(jenv, $1, 6);
  else if (test == "getOrientation" || test == "virtualRealityHeadsetGetOrientation")
    $result = SWIG_JavaArrayOutDouble(jenv, $1, 9);
  else if (test == "getPose")
    $result = SWIG_JavaArrayOutDouble(jenv, $1, 16);
  else if (test != "colors" && test != "getLookupTable")
    $result = SWIG_JavaArrayOutDouble(jenv, $1, 3);
}
%apply double[] {double *};

%typemap(out) double *colors {
  $result = SWIG_JavaArrayOutDouble(jenv, (double *) $1, arg1->number_of_colors*3);
}

%typemap(out) const double *getLookupTable {
  $result = SWIG_JavaArrayOutDouble(jenv, (double *) $1, arg1->getLookupTableSize()*3);
}

// for loading the shared library
%pragma(java) jniclasscode=%{
  static {
    try {
      System.loadLibrary("JavaController");
    } catch (UnsatisfiedLinkError e) {
      System.err.println("Native code library failed to load. See the chapter on Dynamic Linking Problems in the SWIG Java documentation for help.\n" + e);
      System.exit(1);
    }
  }
%}

//for generating simple enums...
//%include "enums.swg"
%include "enumsimple.swg"
%javaconst(1);

//handling std::string
%include "std_string.i"

//----------------------------------------------------------------------------------------------
//  ANSI Support
//----------------------------------------------------------------------------------------------

%typemap(javacode) webots::AnsiCodes %{
  public final static String RESET = "\u001b[0m";

  public final static String BOLD = "\u001b[1m";
  public final static String UNDERLINE = "\u001b[4m";

  public final static String BLACK_BACKGROUND = "\u001b[40m";
  public final static String RED_BACKGROUND = "\u001b[41m";
  public final static String GREEN_BACKGROUND = "\u001b[42m";
  public final static String YELLOW_BACKGROUND = "\u001b[43m";
  public final static String BLUE_BACKGROUND = "\u001b[44m";
  public final static String MAGENTA_BACKGROUND = "\u001b[45m";
  public final static String CYAN_BACKGROUND = "\u001b[46m";
  public final static String WHITE_BACKGROUND = "\u001b[47m";

  public final static String BLACK_FOREGROUND = "\u001b[30m";
  public final static String RED_FOREGROUND = "\u001b[31m";
  public final static String GREEN_FOREGROUND = "\u001b[32m";
  public final static String YELLOW_FOREGROUND = "\u001b[33m";
  public final static String BLUE_FOREGROUND = "\u001b[34m";
  public final static String MAGENTA_FOREGROUND = "\u001b[35m";
  public final static String CYAN_FOREGROUND = "\u001b[36m";
  public final static String WHITE_FOREGROUND = "\u001b[37m";

  public final static String CLEAR_SCREEN = "\u001b[2J";
%}


// we need to create an empty class for SWIG to create a Java module
%inline %{
namespace webots {
  class AnsiCodes {
  };
}
%}

//----------------------------------------------------------------------------------------------
//  Device
//----------------------------------------------------------------------------------------------

%include <webots/Device.hpp>

//----------------------------------------------------------------------------------------------
//  Accelerometer
//----------------------------------------------------------------------------------------------

%include <webots/Accelerometer.hpp>

//----------------------------------------------------------------------------------------------
//  Altimeter
//----------------------------------------------------------------------------------------------

%include <webots/Altimeter.hpp>

//----------------------------------------------------------------------------------------------
//  Brake
//----------------------------------------------------------------------------------------------

%rename("getMotorPrivate") getMotor();
%javamethodmodifiers getMotor() "private"
%rename("getPositionSensorPrivate") getPositionSensor();
%javamethodmodifiers getPositionSensor() "private"

%typemap(javacode) webots::Brake %{
  private Motor motor;
  private PositionSensor positionSensor;

  public Motor getMotor() {
    if (motor == null)
      motor = (Motor)Robot.getDevice(getMotorTag());
    return motor;
  }
  public PositionSensor getPositionSensor() {
    if (positionSensor == null)
      positionSensor = (PositionSensor)Robot.getDevice(getPositionSensorTag());
    return positionSensor;
  }
%}
%include <webots/Brake.hpp>

//----------------------------------------------------------------------------------------------
//  Camera
//----------------------------------------------------------------------------------------------

%rename WbCameraRecognitionObject CameraRecognitionObject;

%javamethodmodifiers position_on_image "private"
%javamethodmodifiers size_on_image "private"
%javamethodmodifiers number_of_colors "private"

%typemap(out) int [] {
  $result = SWIG_JavaArrayOutInt(jenv, $1, 2);
}
%apply int[] {const int *};

%include <webots/camera_recognition_object.h>

%extend WbCameraRecognitionObject {
  const int *getPositionOnImage() const {
    return $self->position_on_image;
  }
  const int *getSizeOnImage() const {
    return $self->size_on_image;
  }
  int getNumberOfColors() const {
    return $self->number_of_colors;
  }
};

%javamethodmodifiers getRecognitionObjects() const "private"
%rename("getRecognitionObjectsPrivate") getRecognitionObjects() const;
%javamethodmodifiers getRecognitionObject(int index) const "private"

%ignore webots::Camera::imageGetRed(const unsigned char *image, int width, int x,int y);
%ignore webots::Camera::imageGetGreen(const unsigned char *image, int width, int x,int y);
%ignore webots::Camera::imageGetBlue(const unsigned char *image, int width, int x,int y);
%ignore webots::Camera::imageGetGrey(const unsigned char *image, int width, int x,int y);
%ignore webots::Camera::imageGetGray(const unsigned char *image, int width, int x,int y);

%typemap(out) const unsigned char * {
  $result = SWIG_JavaArrayOutInt(jenv, (int *) $1, arg1->getWidth()*arg1->getHeight());
}

%typemap(jni) const unsigned char * "jintArray"
%typemap(jtype) const unsigned char * "int[]"
%typemap(jstype) const unsigned char * "int[]"

%typemap(javain) const unsigned char * "$javainput"
%typemap(javaout) const unsigned char * {
  return $jnicall;
}

%apply int[] {const int *};

%extend webots::Camera {
  webots::CameraRecognitionObject getRecognitionObject(int index) const {
    const webots::CameraRecognitionObject *objects = $self->getRecognitionObjects();
    return objects[index];
  }
};

%typemap(javacode) webots::Camera %{
  public static int imageGetRed(int[] image, int width, int x, int y){
    return ((image[y * width + x] & 0xff0000) >> 16) & 0xff;
  }

  public static int imageGetGreen(int[] image, int width, int x, int y){
    return ((image[y * width + x] & 0x00ff00) >> 8) & 0xff;
  }

  public static int imageGetBlue(int[] image, int width, int x, int y){
    return image[y * width + x] & 0x0000ff;
  }

  public static int imageGetGray(int[] image, int width, int x, int y){
    int pixel = (image[y * width + x] & 0xffffff);
    return (
            ((pixel >> 16) & 0xff) +
            ((pixel >>  8) & 0xff) +
            (pixel         & 0xff)
           )/3;
  }

  public static int imageGetGrey(int[] image, int width, int x, int y){
    return imageGetGray(image, width, x, y);
  }

  public static int pixelGetRed(int pixel) {
    return ((pixel & 0xff0000) >> 16) & 0xff;
  }

  public static int pixelGetGreen(int pixel) {
    return ((pixel & 0x00ff00) >> 8) & 0xff;
  }

  public static int pixelGetBlue(int pixel) {
    return pixel & 0x0000ff;
  }

  public static int pixelGetGray(int pixel) {
    return (
            ((pixel >> 16) & 0xff) +
            ((pixel >>  8) & 0xff) +
            (pixel         & 0xff)
           )/3;
  }

  public static int pixelGetGrey(int pixel) {
    return pixelGetGray(pixel);
  }

  public CameraRecognitionObject[] getRecognitionObjects() {
    int numberOfObjects = wrapperJNI.Camera_getRecognitionNumberOfObjects(swigCPtr, this);
    CameraRecognitionObject ret[] = new CameraRecognitionObject[numberOfObjects];
    for (int i = 0; i < numberOfObjects; ++i)
      ret[i] = this.getRecognitionObject(i);
    return ret;
  }
%}

%include <webots/Camera.hpp>

%clear const int*;
%clear const unsigned char *;

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

%apply int[] {int *};

%ignore webots::ImageRef::ImageRef(const WbDeviceTag &deviceTag,const WbImageRef &imageRef);
%ignore webots::ImageRef::getImageRef();

%typemap(in) const void * {
  if (!$input)
    SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null array");
  jsize sz = jenv->GetArrayLength($input);
  jint *jarr = jenv->GetIntArrayElements($input, 0);

  // big endian -> little endian
  unsigned int *v = (unsigned int *)jarr;
  for(int i = 0; i < sz; i++)
    v[i] = ((v[i] << 24) | ((v[i] & 0xff00) << 8) | ((v[i] >> 8) & 0xff00) | (v[i] >> 24));

  $1 = (void *) jarr;
}

%typemap(freearg) const void * {
  jenv->ReleaseIntArrayElements(jarg4, (jint*)$1, 0);
}

%typemap(jni) (const void *) "jintArray"
%typemap(jtype) (const void *) "int[]"
%typemap(jstype) (const void *) "int[]"

%typemap(javain) const void * "$javainput"
%typemap(javaout) const void * {
  return $jnicall;
}

%typemap(javacode) webots::Display %{
   public void drawPolygon(int[] x, int[] y) {
     drawPolygon(x,y,Math.min(x.length,y.length));
   }
   public void fillPolygon(int[] x, int[] y) {
     fillPolygon(x,y,Math.min(x.length,y.length));
   }
%}

%include <webots/ImageRef.hpp>
%include <webots/Display.hpp>

%clear int*;
%clear const void *;

//----------------------------------------------------------------------------------------------
//  Distance sensor
//----------------------------------------------------------------------------------------------

%include <webots/DistanceSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Emitter
//----------------------------------------------------------------------------------------------

%typemap(in) void * {
  if (!$input)
    SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null array");
  jbyte *jarr = jenv->GetByteArrayElements($input, 0);
  $1 = (void *) jarr;
}

%typemap(jni) void * "jbyteArray"
%typemap(jtype) void * "byte[]"
%typemap(jstype) void * "byte[]"

%typemap(javain) void * "$javainput"
%typemap(javaout) void * {
  return $jnicall;
}

%typemap(javacode) webots::Emitter %{
   public int send(byte[] data){
     return send(data,data.length);
   }
%}

%include <webots/Emitter.hpp>

%clear void *;

//----------------------------------------------------------------------------------------------
//  Field
//----------------------------------------------------------------------------------------------

%ignore webots::Field::findField(WbFieldRef ref);
%ignore webots::Field::cleanup();

%rename("getSFNodePrivate") getSFNode() const;
%rename("getMFNodePrivate") getMFNode(int index) const;
%javamethodmodifiers getSFNode() const "private"
%javamethodmodifiers getMFNode(int index) const "private"

%typemap(javacode) webots::Field %{
  public Node getSFNode() {
    long cPtr = wrapperJNI.Field_getSFNodePrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Node getMFNode(int index) {
    long cPtr = wrapperJNI.Field_getMFNodePrivate(swigCPtr, this, index);
    return Node.findNode(cPtr);
  }

  private static java.util.HashMap<Long,Field> fields = new java.util.HashMap<Long,Field>();

  // DO NOT USE THIS FUNCTION: IT IS RESERVED FOR INTERNAL USE !
  public static Field findField(long cPtr) {
    if (cPtr == 0)
      return null;

    Field field = fields.get(new Long(cPtr));
    if (field != null)
      return field;

    field = new Field(cPtr, false);
    fields.put(new Long(cPtr), field);
    return field;
  }
%}

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

%extend WbLidarPoint {
  double getLayerId() {
    return $self->layer_id;
  }
};

%typemap(out) float [] {
  int size = arg1->getHorizontalResolution();
  const string functionName("$name");
  if (functionName != "getLayerRangeImage")
    size *= arg1->getNumberOfLayers();
  $result = SWIG_JavaArrayOutFloat(jenv, $1, size);
}

%apply float[] {const float *};

%javamethodmodifiers getPointCloud() const "private"
%javamethodmodifiers getLayerPointCloud(int layer) const "private"
%rename("getPointCloudPrivate") getPointCloud() const;
%rename("getLayerPointCloudPrivate") getLayerPointCloud(int layer) const;
%javamethodmodifiers getPoint(int index) const "private"
%javamethodmodifiers getLayerPoint(int layer, int index) const "private"

%extend webots::Lidar {
  webots::LidarPoint getPoint(int index) const {
    const webots::LidarPoint *point = $self->getPointCloud();
    return point[index];
  }

  webots::LidarPoint getLayerPoint(int layer, int index) const {
    const webots::LidarPoint *point = $self->getLayerPointCloud(layer);
    return point[index];
  }
};

%typemap(javacode) webots::Lidar %{
  public LidarPoint[] getPointCloud() {
    int numberOfPoints = wrapperJNI.Lidar_getNumberOfPoints(swigCPtr, this);
    LidarPoint ret[] = new LidarPoint[numberOfPoints];
    for (int i = 0; i < numberOfPoints; ++i)
      ret[i] = this.getPoint(i);
    return ret;
  }

  public LidarPoint[] getLayerPointCloud(int layer) {
    int numberOfPoints = wrapperJNI.Lidar_getHorizontalResolution(swigCPtr, this);
    LidarPoint ret[] = new LidarPoint[numberOfPoints];
    for (int i = 0; i < numberOfPoints; ++i)
      ret[i] = this.getLayerPoint(layer, i);
    return ret;
  }
%}

%include <webots/Lidar.hpp>

%clear const float *;

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

%rename("getBrakePrivate") getBrake();
%javamethodmodifiers getBrake() "private"
%rename("getPositionSensorPrivate") getPositionSensor();
%javamethodmodifiers getPositionSensor() "private"

%typemap(javacode) webots::Motor %{
  private Brake brake;
  private PositionSensor positionSensor;

  public Brake getBrake() {
    if (brake == null)
      brake = (Brake)Robot.getDevice(getBrakeTag());
    return brake;
  }
  public PositionSensor getPositionSensor() {
    if (positionSensor == null)
      positionSensor = (PositionSensor)Robot.getDevice(getPositionSensorTag());
    return positionSensor;
  }
%}
%include <webots/Motor.hpp>

//----------------------------------------------------------------------------------------------
//  Mouse
//----------------------------------------------------------------------------------------------

%rename WbMouseState MouseState;

%include <webots/mouse_state.h>

%include <webots/Mouse.hpp>

//----------------------------------------------------------------------------------------------
//  Node
//----------------------------------------------------------------------------------------------

%rename WbContactPoint ContactPoint;

%ignore webots::Node::findNode(WbNodeRef ref);
%ignore webots::Node::cleanup();

%rename("getParentNodePrivate") getParentNode() const;
%javamethodmodifiers getParentNode() const "private"

%rename("getFromProtoDefPrivate") getFromProtoDef(const std::string &name) const;
%javamethodmodifiers getFromProtoDef(const std::string &name) const "private"

%rename("getFieldPrivate") getField(const std::string &fieldName) const;
%javamethodmodifiers getField(const std::string &fieldName) const "private"

%apply int *OUTPUT { int *size };
%rename(getContactPointsPrivate) getContactPoints;

%include <webots/contact_point.h>
%extend WbContactPoint {
  int getNodeId() const {
    return $self->node_id;
  }
};

%extend webots::Node {
  ContactPoint getContactPointFromPointer(long long points, int index) const {
    return *((webots::ContactPoint *)(points + index));
  }
};

%typemap(javacode) webots::Node %{
// ----- begin hand written section ----
  public ContactPoint[] getContactPoints(Boolean includeDescendants) {
    int sizePointer[] = {0};
    long result = wrapperJNI.Node_getContactPointsPrivate(swigCPtr, this, includeDescendants, sizePointer);
    int size = sizePointer[0];
    ContactPoint ret[] = new ContactPoint[size];

    for (int i = 0; i < size; ++i)
      ret[i] = getContactPointFromPointer(result, 0);
    return ret;
  }

  public Node getParentNode() {
    long cPtr = wrapperJNI.Node_getParentNodePrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Node getFromProtoDef(String name) {
    long cPtr = wrapperJNI.Node_getFromProtoDefPrivate(swigCPtr, this, name);
    return Node.findNode(cPtr);
  }

  public Field getField(String fieldName) {
    long cPtr = wrapperJNI.Node_getFieldPrivate(swigCPtr, this, fieldName);
    return Field.findField(cPtr);
  }

  private static java.util.HashMap<Long,Node> nodes = new java.util.HashMap<Long,Node>();

  // DO NOT USE THIS FUNCTION: IT IS RESERVED FOR INTERNAL USE !
  public static Node findNode(long cPtr) {
    if (cPtr == 0)
      return null;

    Node node = nodes.get(new Long(cPtr));
    if (node != null)
      return node;

    node = new Node(cPtr, false);
    nodes.put(new Long(cPtr), node);
    return node;
  }
%}

%include <webots/Node.hpp>

//----------------------------------------------------------------------------------------------
//  Pen
//----------------------------------------------------------------------------------------------

%include <webots/Pen.hpp>

//----------------------------------------------------------------------------------------------
//  PositionSensor
//----------------------------------------------------------------------------------------------

%rename("getBrakePrivate") getBrake();
%javamethodmodifiers getBrake() "private"
%rename("getMotorPrivate") getMotor();
%javamethodmodifiers getPositionSensor() "private"

%typemap(javacode) webots::PositionSensor %{
  private Brake brake;
  private Motor motor;

  public Brake getBrake() {
    if (brake == null)
      brake = (Brake)Robot.getDevice(getBrakeTag());
    return brake;
  }
  public Motor getMotor() {
    if (motor == null)
      motor = (Motor)Robot.getDevice(getMotorTag());
    return motor;
  }
%}
%include <webots/PositionSensor.hpp>

//----------------------------------------------------------------------------------------------
//  Radar
//----------------------------------------------------------------------------------------------

%rename WbRadarTarget RadarTarget;

%include <webots/radar_target.h>

%extend WbRadarTarget {
  double getReceivedPower() {
    return $self->received_power;
  }
};

%javamethodmodifiers getTargets() const "private"
%rename("getTargetsPrivate") getTargets() const;
%javamethodmodifiers getTarget(int index) const "private"

%extend webots::Radar {
  webots::RadarTarget getTarget(int index) const {
    const webots::RadarTarget *targets = $self->getTargets();
    return targets[index];
  }
};

%typemap(javacode) webots::Radar %{
  public RadarTarget[] getTargets() {
    int numberOfTargets = wrapperJNI.Radar_getNumberOfTargets(swigCPtr, this);
    RadarTarget ret[] = new RadarTarget[numberOfTargets];
    for (int i = 0; i < numberOfTargets; ++i)
      ret[i] = this.getTarget(i);
    return ret;
  }
%}

%include <webots/Radar.hpp>

//----------------------------------------------------------------------------------------------
//  RangeFinder
//----------------------------------------------------------------------------------------------

%typemap(out) float [] {
  $result = SWIG_JavaArrayOutFloat(jenv, $1, arg1->getWidth()*arg1->getHeight());
}

%apply float[] {const float *};

%include <webots/RangeFinder.hpp>

%clear const float *;

//----------------------------------------------------------------------------------------------
//  Receiver
//----------------------------------------------------------------------------------------------

%typemap(out) void * {
  $result = SWIG_JavaArrayOutSchar(jenv, (signed char *) $1, arg1->getDataSize());
}

%typemap(jni) void * "jbyteArray"
%typemap(jtype) void * "byte[]"
%typemap(jstype) void * "byte[]"

%typemap(javain) void * "$javainput"
%typemap(javaout) void * {
  return $jnicall;
}

%include <webots/Receiver.hpp>

%clear void *;

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
//  VacuumGripper
//----------------------------------------------------------------------------------------------

%include <webots/VacuumGripper.hpp>

//----------------------------------------------------------------------------------------------
//  Robot
//----------------------------------------------------------------------------------------------

%ignore webots::Robot::getAccelerometer(const std::string &name);
%ignore webots::Robot::getAltimeter(const std::string &name);
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
%ignore webots::Robot::getVacuumGripper(const std::string &name);
%ignore webots::Robot::windowCustomFunction(void *arg);
%ignore webots::Robot::wwiSend(const char *data, int size);
%ignore webots::Robot::wwiReceive(int *size);

%rename("getDevicePrivate") getDevice(int tag);
%javamethodmodifiers getDevice(int tag) "private"
%rename("getDeviceByIndexPrivate") getDeviceByIndex(int index);
%javamethodmodifiers getDeviceByIndex(int index) "private"
%javamethodmodifiers getDeviceTypeFromTag(int tag) "private"
%javamethodmodifiers getDeviceNameFromTag(int tag) "private"
%javamethodmodifiers getDeviceTagFromIndex(int index) "private"
%javamethodmodifiers getDeviceTagFromName(String name) "private"

%typemap(javacode) webots::Robot %{

  static private Device[] devices = null;
  private Joystick joystick = new Joystick();
  private Keyboard keyboard = new Keyboard();
  private Mouse mouse = new Mouse();

  protected Accelerometer createAccelerometer(String name) {
    return new Accelerometer(name);
  }

  public Accelerometer getAccelerometer(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.ACCELEROMETER))
      return null;
    return (Accelerometer)getOrCreateDevice(tag);
  }

  protected Altimeter createAltimeter(String name) {
    return new Altimeter(name);
  }

  public Altimeter getAltimeter(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.ALTIMETER))
      return null;
    return (Altimeter)getOrCreateDevice(tag);
  }

  protected Brake createBrake(String name) {
    return new Brake(name);
  }

  public Brake getBrake(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.BRAKE))
      return null;
    return (Brake)getOrCreateDevice(tag);
  }

  protected Camera createCamera(String name) {
    return new Camera(name);
  }

  public Camera getCamera(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.CAMERA))
      return null;
    return (Camera)getOrCreateDevice(tag);
  }

  protected Compass createCompass(String name) {
    return new Compass(name);
  }

  public Compass getCompass(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.COMPASS))
      return null;
    return (Compass)getOrCreateDevice(tag);
  }

  protected Connector createConnector(String name) {
    return new Connector(name);
  }

  public Connector getConnector(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.CONNECTOR))
      return null;
    return (Connector)getOrCreateDevice(tag);
  }

  protected Display createDisplay(String name) {
    return new Display(name);
  }

  public Display getDisplay(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.DISPLAY))
      return null;
    return (Display)getOrCreateDevice(tag);
  }

  protected DistanceSensor createDistanceSensor(String name) {
    return new DistanceSensor(name);
  }

  public DistanceSensor getDistanceSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.DISTANCE_SENSOR))
      return null;
    return (DistanceSensor)getOrCreateDevice(tag);
  }

  protected Emitter createEmitter(String name) {
    return new Emitter(name);
  }

  public Emitter getEmitter(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.EMITTER))
      return null;
    return (Emitter)getOrCreateDevice(tag);
  }

  protected GPS createGPS(String name) {
    return new GPS(name);
  }

  public GPS getGPS(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.GPS))
      return null;
    return (GPS)getOrCreateDevice(tag);
  }

  protected Gyro createGyro(String name) {
    return new Gyro(name);
  }

  public Gyro getGyro(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.GYRO))
      return null;
    return (Gyro)getOrCreateDevice(tag);
  }

  protected InertialUnit createInertialUnit(String name) {
    return new InertialUnit(name);
  }

  public InertialUnit getInertialUnit(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.INERTIAL_UNIT))
      return null;
    return (InertialUnit)getOrCreateDevice(tag);
  }

  public Joystick getJoystick() {
    return joystick;
  }

  public Keyboard getKeyboard() {
    return keyboard;
  }

  protected LED createLED(String name) {
    return new LED(name);
  }

  public LED getLED(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LED))
      return null;
    return (LED)getOrCreateDevice(tag);
  }

  protected Lidar createLidar(String name) {
    return new Lidar(name);
  }

  public Lidar getLidar(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LIDAR))
      return null;
    return (Lidar)getOrCreateDevice(tag);
  }

  protected LightSensor createLightSensor(String name) {
    return new LightSensor(name);
  }

  public LightSensor getLightSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LIGHT_SENSOR))
      return null;
    return (LightSensor)getOrCreateDevice(tag);
  }

  protected Motor createMotor(String name) {
    return new Motor(name);
  }

  public Motor getMotor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LINEAR_MOTOR) && !Device.hasType(tag, Node.ROTATIONAL_MOTOR))
      return null;
    return (Motor)getOrCreateDevice(tag);
  }

  public Mouse getMouse() {
    return mouse;
  }

  protected Pen createPen(String name) {
    return new Pen(name);
  }

  public Pen getPen(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.PEN))
      return null;
    return (Pen)getOrCreateDevice(tag);
  }

  protected PositionSensor createPositionSensor(String name) {
    return new PositionSensor(name);
  }

  public PositionSensor getPositionSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.POSITION_SENSOR))
      return null;
    return (PositionSensor)getOrCreateDevice(tag);
  }

  protected Radar createRadar(String name) {
    return new Radar(name);
  }

  public Radar getRadar(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.RADAR))
      return null;
    return (Radar)getOrCreateDevice(tag);
  }

  protected RangeFinder createRangeFinder(String name) {
    return new RangeFinder(name);
  }

  public RangeFinder getRangeFinder(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.RANGE_FINDER))
      return null;
    return (RangeFinder)getOrCreateDevice(tag);
  }

  protected Receiver createReceiver(String name) {
    return new Receiver(name);
  }

  public Receiver getReceiver(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.RECEIVER))
      return null;
    return (Receiver)getOrCreateDevice(tag);
  }

  protected Skin createSkin(String name) {
    return new Skin(name);
  }

  public Skin getSkin(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.SKIN))
      return null;
    return (Skin)getOrCreateDevice(tag);
  }

  protected Speaker createSpeaker(String name) {
    return new Speaker(name);
  }

  public Speaker getSpeaker(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.SPEAKER))
      return null;
    return (Speaker)getOrCreateDevice(tag);
  }

  protected TouchSensor createTouchSensor(String name) {
    return new TouchSensor(name);
  }

  public TouchSensor getTouchSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.TOUCH_SENSOR))
      return null;
    return (TouchSensor)getOrCreateDevice(tag);
  }

  protected VacuumGripper createVacuumGripper(String name) {
    return new VacuumGripper(name);
  }

  public VacuumGripper getVacuumGripper(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.VACUUM_GRIPPER))
      return null;
    return (VacuumGripper)getOrCreateDevice(tag);
  }

  public Device getDeviceByIndex(int index) {
    return getOrCreateDevice(getDeviceTagFromIndex(index));
  }

  static public Device getDevice(int tag) {
    if (tag == 0 || devices == null || tag >= devices.length)
      return null;
    return devices[tag];
  }

  private Device getOrCreateDevice(int tag) {
    if (tag == 0)
      return null;

    int count = getNumberOfDevices();
    // if new devices have been added, then count is greater than devices.length
    // deleted devices are not removed from the C API list and don't affect the number of devices
    if (devices != null && devices.length == count + 1 && tag < devices.length)
        return devices[tag];

    // (re-)initialize devices list
    if (tag > count)
        return null;
    devices = new Device[count + 1];
    for (int i = 0; i < count; i++) {
      int otherTag = getDeviceTagFromIndex(i);
      String name = getDeviceNameFromTag(otherTag);
      switch(getDeviceTypeFromTag(otherTag)) {
        case Node.ACCELEROMETER:    devices[otherTag] = createAccelerometer(name); break;
        case Node.ALTIMETER:        devices[otherTag] = createAltimeter(name); break;
        case Node.BRAKE:            devices[otherTag] = createBrake(name); break;
        case Node.CAMERA:           devices[otherTag] = createCamera(name); break;
        case Node.COMPASS:          devices[otherTag] = createCompass(name); break;
        case Node.CONNECTOR:        devices[otherTag] = createConnector(name); break;
        case Node.DISPLAY:          devices[otherTag] = createDisplay(name); break;
        case Node.DISTANCE_SENSOR:  devices[otherTag] = createDistanceSensor(name); break;
        case Node.EMITTER:          devices[otherTag] = createEmitter(name); break;
        case Node.GPS:              devices[otherTag] = createGPS(name); break;
        case Node.GYRO:             devices[otherTag] = createGyro(name); break;
        case Node.INERTIAL_UNIT:    devices[otherTag] = createInertialUnit(name); break;
        case Node.LED:              devices[otherTag] = createLED(name); break;
        case Node.LIDAR:            devices[otherTag] = createLidar(name); break;
        case Node.LIGHT_SENSOR:     devices[otherTag] = createLightSensor(name); break;
        case Node.LINEAR_MOTOR:
        case Node.ROTATIONAL_MOTOR: devices[otherTag] = createMotor(name); break;
        case Node.PEN:              devices[otherTag] = createPen(name); break;
        case Node.POSITION_SENSOR:  devices[otherTag] = createPositionSensor(name); break;
        case Node.RADAR:            devices[otherTag] = createRadar(name); break;
        case Node.RANGE_FINDER:     devices[otherTag] = createRangeFinder(name); break;
        case Node.RECEIVER:         devices[otherTag] = createReceiver(name); break;
        case Node.SKIN:             devices[otherTag] = createSkin(name); break;
        case Node.SPEAKER:          devices[otherTag] = createSpeaker(name); break;
        case Node.TOUCH_SENSOR:     devices[otherTag] = createTouchSensor(name); break;
        case Node.VACUUM_GRIPPER:   devices[otherTag] = createVacuumGripper(name); break;
        default:                    devices[otherTag] = null; break;
      }
    }
    return devices[tag];
  }
%}

%include <webots/Robot.hpp>

//----------------------------------------------------------------------------------------------
//  Supervisor
//----------------------------------------------------------------------------------------------

%typemap(javacode) webots::Supervisor %{
  public Node getRoot() {
    long cPtr = wrapperJNI.Supervisor_getRootPrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Node getSelf() {
    long cPtr = wrapperJNI.Supervisor_getSelfPrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Node getFromDef(String name) {
    long cPtr = wrapperJNI.Supervisor_getFromDefPrivate(swigCPtr, this, name);
    return Node.findNode(cPtr);
  }

  public Node getFromId(int id) {
    long cPtr = wrapperJNI.Supervisor_getFromIdPrivate(swigCPtr, this, id);
    return Node.findNode(cPtr);
  }

  public Node getFromDevice(Device device) {
    return getFromDeviceTag(device.getTag());
  }

  private Node getFromDeviceTag(int tag) {
    long cPtr = wrapperJNI.Supervisor_getFromDeviceTagPrivate(swigCPtr, this, tag);
    return Node.findNode(cPtr);
  }

  public Node getSelected() {
    long cPtr = wrapperJNI.Supervisor_getSelectedPrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }
%}

%rename("getRootPrivate") getRoot() const;
%rename("getSelfPrivate") getSelf() const;
%rename("getFromDefPrivate") getFromDef(const std::string &name) const;
%rename("getFromIdPrivate") getFromId(int id) const;
%rename("getFromDevicePrivate") getFromDevice(const Device *device) const;
%rename("getFromDeviceTagPrivate") getFromDeviceTag(int tag) const;
%rename("getSelectedPrivate") getSelected() const;

%javamethodmodifiers getRoot() const "private"
%javamethodmodifiers getSelf() const "private"
%javamethodmodifiers getFromDef(const std::string &name) const "private"
%javamethodmodifiers getFromId(int id) const "private"
%javamethodmodifiers getFromDevice(const Device *device) const "private"
%javamethodmodifiers getFromDeviceTag(int tag) const "private"
%javamethodmodifiers getSelected() const "private"

%include <webots/Supervisor.hpp>
