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

#ifndef ROS_HPP
#define ROS_HPP

#include <ros/node_handle.h>
#include <webots/Robot.hpp>

#include <webots_ros/get_bool.h>
#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/get_urdf.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>

#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/robot_set_mode.h>
#include <webots_ros/robot_wait_for_user_input_event.h>

#include <highlevel/RosControl.hpp>

using namespace webots;

class RosSensor;
class RosDevice;
class RosJoystick;
class RosKeyboard;
class RosSupervisor;

class Ros {
public:
  Ros();
  virtual ~Ros();

  void run(int argc, char **argv);
  ros::NodeHandle *nodeHandle() { return mNodeHandle; }
  int stepSize() const { return mStepSize; }
  const std::string &name() const { return mRobotName; }
  const std::string &rosNameSpace() const { return mRosNameSpace; }
  Device *getDevice(const std::string &name);

  static std::string fixedNameString(const std::string &name);

protected:
  virtual void setupRobot();
  virtual void setRosDevices(const char **hiddenDevices, int numberHiddenDevices);
  virtual void launchRos(int argc, char **argv);
  virtual int step(int duration) { return mRobot->step(duration); }
  Robot *mRobot;

private:
  void fixName();
  void publishClockIfNeeded();
  bool timeStepCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool waitForUserInputEventCallback(webots_ros::robot_wait_for_user_input_event::Request &req,
                                     webots_ros::robot_wait_for_user_input_event::Response &res);
  bool getDeviceListCallback(webots_ros::robot_get_device_list::Request &req, webots_ros::robot_get_device_list::Response &res);
  bool getTimeCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getModelCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getDataCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getUrdfCallback(webots_ros::get_urdf::Request &req, webots_ros::get_urdf::Response &res);
  bool setDataCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool getCustomDataCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool setCustomDataCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool getModeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getSupervisorCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getSynchronizationCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getProjectPathCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getWorldPathCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getBasicTimeStepCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getNumberOfDevicesCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool setModeCallback(webots_ros::robot_set_mode::Request &req, webots_ros::robot_set_mode::Response &res);
  bool wwiReceiveTextCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool wwiSendTextCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);

  std::string mRobotName;
  std::vector<RosDevice *> mDeviceList;
  std::vector<RosSensor *> mSensorList;
  RosSupervisor *mRosSupervisor;
  RosJoystick *mRosJoystick;
  RosKeyboard *mRosKeyboard;
  ros::NodeHandle *mNodeHandle;
  ros::Publisher mNamePublisher;
  ros::ServiceServer mTimeStepService;
  ros::ServiceServer mWaitForUserInputEventService;
  ros::ServiceServer mDeviceListService;
  ros::ServiceServer mGetTimeService;
  ros::ServiceServer mGetModelService;
  ros::ServiceServer mGetUrdfService;
  ros::ServiceServer mGetDataService;
  ros::ServiceServer mSetDataService;
  ros::ServiceServer mGetCustomDataService;
  ros::ServiceServer mSetCustomDataService;
  ros::ServiceServer mGetModeService;
  ros::ServiceServer mGetSupervisorService;
  ros::ServiceServer mGetSynchronizationService;
  ros::ServiceServer mGetProjectPathService;
  ros::ServiceServer mGetWorldPathService;
  ros::ServiceServer mGetBasicTimeStepService;
  ros::ServiceServer mGetNumberOfDevicesService;
  ros::ServiceServer mSetModeService;
  ros::ServiceServer mWwiReceiveTextService;
  ros::ServiceServer mWwiSendTextService;
  ros::Publisher mClockPublisher;
  unsigned int mStepSize;
  int mStep;
  bool mEnd;
  bool mShouldPublishClock;
  bool mIsSynchronized;
  bool mUseWebotsSimTime;
  bool mAutoPublish;
  bool mUseRosControl;
  std::string mRosNameSpace;
  std::string mRobotDescriptionPrefix;
  bool mSetRobotDescription;
  highlevel::RosControl *mRosControl;
};

#endif  // ROS_HPP
