// Copyright 1996-2020 Cyberbotics Ltd.
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

/*
 * Description:   controller for the PR2 to grab a can in a fridge and serve it
 */

#include <signal.h>
#include <webots_ros/sensor_set.h>
#include "ros/ros.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#define TIME_STEP 32
#define ACCELERATION 10
#define FWD_SPEED 3.
#define ROTATE_SPEED 6

static int controllerCount;
static std::vector<std::string> controllerList;
static std::string controllerName;
static double wheelPosition = 0;
static double torsoPosition = 0;
static double rotationPosition = 0;
static double lShoulderPosition = 0;
static double rShoulderPosition = 0;
static double llFingerPosition = 0;
static double rrFingerPosition = 0;
static unsigned int lBumper = 0;
static unsigned int rBumper = 0;
static double imu_rpy[3] = {0, 0, 0};

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

ros::ServiceClient fllWheelPositionClient;
ros::ServiceClient flrWheelPositionClient;
ros::ServiceClient frlWheelPositionClient;
ros::ServiceClient frrWheelPositionClient;
ros::ServiceClient bllWheelPositionClient;
ros::ServiceClient blrWheelPositionClient;
ros::ServiceClient brlWheelPositionClient;
ros::ServiceClient brrWheelPositionClient;

webots_ros::set_float fllWheelPositionSrv;
webots_ros::set_float flrWheelPositionSrv;
webots_ros::set_float frlWheelPositionSrv;
webots_ros::set_float frrWheelPositionSrv;
webots_ros::set_float bllWheelPositionSrv;
webots_ros::set_float blrWheelPositionSrv;
webots_ros::set_float brlWheelPositionSrv;
webots_ros::set_float brrWheelPositionSrv;

ros::ServiceClient fllWheelSpeedClient;
ros::ServiceClient flrWheelSpeedClient;
ros::ServiceClient frlWheelSpeedClient;
ros::ServiceClient frrWheelSpeedClient;
ros::ServiceClient bllWheelSpeedClient;
ros::ServiceClient blrWheelSpeedClient;
ros::ServiceClient brlWheelSpeedClient;
ros::ServiceClient brrWheelSpeedClient;

webots_ros::set_float fllWheelSpeedSrv;
webots_ros::set_float flrWheelSpeedSrv;
webots_ros::set_float frlWheelSpeedSrv;
webots_ros::set_float frrWheelSpeedSrv;
webots_ros::set_float bllWheelSpeedSrv;
webots_ros::set_float blrWheelSpeedSrv;
webots_ros::set_float brlWheelSpeedSrv;
webots_ros::set_float brrWheelSpeedSrv;

ros::ServiceClient flRotationClient;
ros::ServiceClient frRotationClient;
ros::ServiceClient blRotationClient;
ros::ServiceClient brRotationClient;

webots_ros::set_float flRotationSrv;
webots_ros::set_float frRotationSrv;
webots_ros::set_float blRotationSrv;
webots_ros::set_float brRotationSrv;

void fllWheelCallback(const std_msgs::Float64::ConstPtr &value) {
  wheelPosition = value->data;
}
void torsoCallback(const std_msgs::Float64::ConstPtr &value) {
  torsoPosition = value->data;
}
void flRotationCallback(const std_msgs::Float64::ConstPtr &value) {
  rotationPosition = value->data;
}
void lShoulderCallback(const std_msgs::Float64::ConstPtr &value) {
  lShoulderPosition = value->data;
}
void rShoulderCallback(const std_msgs::Float64::ConstPtr &value) {
  rShoulderPosition = value->data;
}
void llFingerCallback(const std_msgs::Float64::ConstPtr &value) {
  llFingerPosition = value->data;
}
void rrFingerCallback(const std_msgs::Float64::ConstPtr &value) {
  rrFingerPosition = value->data;
}
void lBumperCallback(const std_msgs::Float64::ConstPtr &value) {
  lBumper = value->data;
}
void rBumperCallback(const std_msgs::Float64::ConstPtr &value) {
  rBumper = value->data;
}
void imuCallback(const std_msgs::Float64MultiArray::ConstPtr &values) {
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it) {
    imu_rpy[i] = *it;
    i++;
  }
}

void move(double speed) {
  fllWheelPositionSrv.request.position = INFINITY;
  flrWheelPositionSrv.request.position = INFINITY;
  frlWheelPositionSrv.request.position = INFINITY;
  frrWheelPositionSrv.request.position = INFINITY;
  bllWheelPositionSrv.request.position = INFINITY;
  blrWheelPositionSrv.request.position = INFINITY;
  brlWheelPositionSrv.request.position = INFINITY;
  brrWheelPositionSrv.request.position = INFINITY;

  fllWheelPositionClient.call(fllWheelPositionSrv);
  flrWheelPositionClient.call(flrWheelPositionSrv);
  frlWheelPositionClient.call(frlWheelPositionSrv);
  frrWheelPositionClient.call(frrWheelPositionSrv);
  bllWheelPositionClient.call(bllWheelPositionSrv);
  blrWheelPositionClient.call(blrWheelPositionSrv);
  brlWheelPositionClient.call(brlWheelPositionSrv);
  brrWheelPositionClient.call(brrWheelPositionSrv);

  fllWheelSpeedSrv.request.value = speed;
  flrWheelSpeedSrv.request.value = speed;
  frlWheelSpeedSrv.request.value = speed;
  frrWheelSpeedSrv.request.value = speed;
  bllWheelSpeedSrv.request.value = speed;
  blrWheelSpeedSrv.request.value = speed;
  brlWheelSpeedSrv.request.value = speed;
  brrWheelSpeedSrv.request.value = speed;

  fllWheelSpeedClient.call(fllWheelSpeedSrv);
  flrWheelSpeedClient.call(flrWheelSpeedSrv);
  frlWheelSpeedClient.call(frlWheelSpeedSrv);
  frrWheelSpeedClient.call(frrWheelSpeedSrv);
  bllWheelSpeedClient.call(bllWheelSpeedSrv);
  blrWheelSpeedClient.call(blrWheelSpeedSrv);
  brlWheelSpeedClient.call(brlWheelSpeedSrv);
  brrWheelSpeedClient.call(brrWheelSpeedSrv);
}

void rotate(double speed) {
  fllWheelPositionSrv.request.position = INFINITY;
  flrWheelPositionSrv.request.position = INFINITY;
  frlWheelPositionSrv.request.position = INFINITY;
  frrWheelPositionSrv.request.position = INFINITY;
  bllWheelPositionSrv.request.position = INFINITY;
  blrWheelPositionSrv.request.position = INFINITY;
  brlWheelPositionSrv.request.position = INFINITY;
  brrWheelPositionSrv.request.position = INFINITY;

  fllWheelPositionClient.call(fllWheelPositionSrv);
  flrWheelPositionClient.call(flrWheelPositionSrv);
  frlWheelPositionClient.call(frlWheelPositionSrv);
  frrWheelPositionClient.call(frrWheelPositionSrv);
  bllWheelPositionClient.call(bllWheelPositionSrv);
  blrWheelPositionClient.call(blrWheelPositionSrv);
  brlWheelPositionClient.call(brlWheelPositionSrv);
  brrWheelPositionClient.call(brrWheelPositionSrv);

  fllWheelSpeedSrv.request.value = -speed;
  flrWheelSpeedSrv.request.value = -speed;
  frlWheelSpeedSrv.request.value = speed;
  frrWheelSpeedSrv.request.value = speed;
  bllWheelSpeedSrv.request.value = -speed;
  blrWheelSpeedSrv.request.value = -speed;
  brlWheelSpeedSrv.request.value = speed;
  brrWheelSpeedSrv.request.value = speed;

  fllWheelSpeedClient.call(fllWheelSpeedSrv);
  flrWheelSpeedClient.call(flrWheelSpeedSrv);
  frlWheelSpeedClient.call(frlWheelSpeedSrv);
  frrWheelSpeedClient.call(frrWheelSpeedSrv);
  bllWheelSpeedClient.call(bllWheelSpeedSrv);
  blrWheelSpeedClient.call(blrWheelSpeedSrv);
  brlWheelSpeedClient.call(brlWheelSpeedSrv);
  brrWheelSpeedClient.call(brrWheelSpeedSrv);
}

void rotate_wheels(double angle) {
  int speed = -ROTATE_SPEED;

  if (angle > rotationPosition)
    speed = ROTATE_SPEED;

  flRotationSrv.request.position = angle;
  frRotationSrv.request.position = -angle;
  blRotationSrv.request.position = -angle;
  brRotationSrv.request.position = angle;

  flRotationClient.call(flRotationSrv);
  frRotationClient.call(frRotationSrv);
  blRotationClient.call(blRotationSrv);
  brRotationClient.call(brRotationSrv);

  fllWheelPositionSrv.request.position = INFINITY;
  flrWheelPositionSrv.request.position = INFINITY;
  frlWheelPositionSrv.request.position = INFINITY;
  frrWheelPositionSrv.request.position = INFINITY;
  bllWheelPositionSrv.request.position = INFINITY;
  blrWheelPositionSrv.request.position = INFINITY;
  brlWheelPositionSrv.request.position = INFINITY;
  brrWheelPositionSrv.request.position = INFINITY;

  fllWheelPositionClient.call(fllWheelPositionSrv);
  flrWheelPositionClient.call(flrWheelPositionSrv);
  frlWheelPositionClient.call(frlWheelPositionSrv);
  frrWheelPositionClient.call(frrWheelPositionSrv);
  bllWheelPositionClient.call(bllWheelPositionSrv);
  blrWheelPositionClient.call(blrWheelPositionSrv);
  brlWheelPositionClient.call(brlWheelPositionSrv);
  brrWheelPositionClient.call(brrWheelPositionSrv);

  fllWheelSpeedSrv.request.value = -speed;
  flrWheelSpeedSrv.request.value = speed;
  frlWheelSpeedSrv.request.value = speed;
  frrWheelSpeedSrv.request.value = -speed;
  bllWheelSpeedSrv.request.value = speed;
  blrWheelSpeedSrv.request.value = -speed;
  brlWheelSpeedSrv.request.value = -speed;
  brrWheelSpeedSrv.request.value = speed;

  fllWheelSpeedClient.call(fllWheelSpeedSrv);
  flrWheelSpeedClient.call(flrWheelSpeedSrv);
  frlWheelSpeedClient.call(frlWheelSpeedSrv);
  frrWheelSpeedClient.call(frrWheelSpeedSrv);
  bllWheelSpeedClient.call(bllWheelSpeedSrv);
  blrWheelSpeedClient.call(blrWheelSpeedSrv);
  brlWheelSpeedClient.call(brlWheelSpeedSrv);
  brrWheelSpeedClient.call(brrWheelSpeedSrv);
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the 'pr2_beer' node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  // create a node named 'pr2_beer' on ROS network
  ros::init(argc, argv, "pr2_beer", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of available controllers
  ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    // the spinOnce() function is called multiple times to be sure to not skip any controller
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  // if there is more than one controller available, let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  // leave topic once it's not necessary anymore
  nameSub.shutdown();

  ros::ServiceClient fllWheelAccelerationClient;
  ros::ServiceClient flrWheelAccelerationClient;
  ros::ServiceClient frlWheelAccelerationClient;
  ros::ServiceClient frrWheelAccelerationClient;
  ros::ServiceClient bllWheelAccelerationClient;
  ros::ServiceClient blrWheelAccelerationClient;
  ros::ServiceClient brlWheelAccelerationClient;
  ros::ServiceClient brrWheelAccelerationClient;

  webots_ros::set_float fllWheelAccelerationSrv;
  webots_ros::set_float flrWheelAccelerationSrv;
  webots_ros::set_float frlWheelAccelerationSrv;
  webots_ros::set_float frrWheelAccelerationSrv;
  webots_ros::set_float bllWheelAccelerationSrv;
  webots_ros::set_float blrWheelAccelerationSrv;
  webots_ros::set_float brlWheelAccelerationSrv;
  webots_ros::set_float brrWheelAccelerationSrv;

  ros::ServiceClient headTiltClient;
  webots_ros::set_float headTiltSrv;
  ros::ServiceClient torsoClient;
  webots_ros::set_float torsoSrv;
  ros::ServiceClient lShoulderRollClient;
  webots_ros::set_float lShoulderRollSrv;
  ros::ServiceClient lShoulderLiftClient;
  webots_ros::set_float lShoulderLiftSrv;
  ros::ServiceClient rShoulderRollClient;
  webots_ros::set_float rShoulderRollSrv;
  ros::ServiceClient rShoulderLiftClient;
  webots_ros::set_float rShoulderLiftSrv;
  ros::ServiceClient ruArmRollClient;
  webots_ros::set_float ruArmRollSrv;
  ros::ServiceClient rElbowLiftClient;
  webots_ros::set_float rElbowLiftSrv;
  ros::ServiceClient lElbowLiftClient;
  webots_ros::set_float lElbowLiftSrv;
  ros::ServiceClient rWristRollClient;
  webots_ros::set_float rWristRollSrv;
  ros::ServiceClient llFingerClient;
  webots_ros::set_float llFingerSrv;
  ros::ServiceClient lrFingerClient;
  webots_ros::set_float lrFingerSrv;
  ros::ServiceClient lrFingerTipClient;
  webots_ros::set_float lrFingerTipSrv;
  ros::ServiceClient llFingerTipClient;
  webots_ros::set_float llFingerTipSrv;
  ros::ServiceClient rlFingerClient;
  webots_ros::set_float rlFingerSrv;
  ros::ServiceClient rrFingerClient;
  webots_ros::set_float rrFingerSrv;

  ros::ServiceClient rlFingerSpeedClient;
  webots_ros::set_float rlFingerSpeedSrv;
  ros::ServiceClient rrFingerSpeedClient;
  webots_ros::set_float rrFingerSpeedSrv;
  ros::ServiceClient llFingerSpeedClient;
  webots_ros::set_float llFingerSpeedSrv;
  ros::ServiceClient lrFingerSpeedClient;
  webots_ros::set_float lrFingerSpeedSrv;
  ros::ServiceClient lShoulderRollSpeedClient;
  webots_ros::set_float lShoulderRollSpeedSrv;
  ros::ServiceClient rShoulderRollSpeedClient;
  webots_ros::set_float rShoulderRollSpeedSrv;

  ros::ServiceClient fllWheelSetClient;
  ros::ServiceClient flRotationSetClient;
  ros::ServiceClient torsoSetClient;
  ros::ServiceClient llFingerSetClient;
  ros::ServiceClient rrFingerSetClient;
  ros::ServiceClient lShoulderSetClient;
  ros::ServiceClient rShoulderSetClient;
  ros::ServiceClient lBumperSetClient;
  ros::ServiceClient rBumperSetClient;
  ros::ServiceClient imuSetClient;
  ros::ServiceClient cameraSetClient;
  webots_ros::sensor_set setSensorSrv;
  setSensorSrv.request.period = TIME_STEP;

  timeStepClient = n.serviceClient<webots_ros::set_int>(controllerName + "/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  fllWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_l_wheel_joint/set_position");
  flrWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_r_wheel_joint/set_position");
  frlWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_l_wheel_joint/set_position");
  frrWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_r_wheel_joint/set_position");
  bllWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_l_wheel_joint/set_position");
  blrWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_r_wheel_joint/set_position");
  brlWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_l_wheel_joint/set_position");
  brrWheelPositionClient = n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_r_wheel_joint/set_position");

  fllWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_l_wheel_joint/set_velocity");
  flrWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_r_wheel_joint/set_velocity");
  frlWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_l_wheel_joint/set_velocity");
  frrWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_r_wheel_joint/set_velocity");
  bllWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_l_wheel_joint/set_velocity");
  blrWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_r_wheel_joint/set_velocity");
  brlWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_l_wheel_joint/set_velocity");
  brrWheelSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_r_wheel_joint/set_velocity");

  fllWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_l_wheel_joint/set_acceleration");
  flrWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_r_wheel_joint/set_acceleration");
  frlWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_l_wheel_joint/set_acceleration");
  frrWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_r_wheel_joint/set_acceleration");
  bllWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_l_wheel_joint/set_acceleration");
  blrWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_r_wheel_joint/set_acceleration");
  brlWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_l_wheel_joint/set_acceleration");
  brrWheelAccelerationClient =
    n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_r_wheel_joint/set_acceleration");

  flRotationClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fl_caster_rotation_joint/set_position");
  frRotationClient = n.serviceClient<webots_ros::set_float>(controllerName + "/fr_caster_rotation_joint/set_position");
  blRotationClient = n.serviceClient<webots_ros::set_float>(controllerName + "/bl_caster_rotation_joint/set_position");
  brRotationClient = n.serviceClient<webots_ros::set_float>(controllerName + "/br_caster_rotation_joint/set_position");

  headTiltClient = n.serviceClient<webots_ros::set_float>(controllerName + "/head_tilt_joint/set_position");
  torsoClient = n.serviceClient<webots_ros::set_float>(controllerName + "/torso_lift_joint/set_position");
  lShoulderRollClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_shoulder_pan_joint/set_position");
  rShoulderRollClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_shoulder_pan_joint/set_position");
  lShoulderLiftClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_shoulder_lift_joint/set_position");
  rShoulderLiftClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_shoulder_lift_joint/set_position");
  ruArmRollClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_upper_arm_roll_joint/set_position");
  lElbowLiftClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_elbow_flex_joint/set_position");
  rElbowLiftClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_elbow_flex_joint/set_position");
  rWristRollClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_wrist_roll_joint/set_position");
  llFingerClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_gripper_l_finger_joint/set_position");
  lrFingerClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_gripper_r_finger_joint/set_position");
  rlFingerClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_gripper_l_finger_joint/set_position");
  rrFingerClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_gripper_r_finger_joint/set_position");
  llFingerTipClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_gripper_l_finger_tip_joint/set_position");
  lrFingerTipClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_gripper_r_finger_tip_joint/set_position");

  llFingerSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_gripper_l_finger_joint/set_velocity");
  lrFingerSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_gripper_r_finger_joint/set_velocity");
  rlFingerSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_gripper_l_finger_joint/set_velocity");
  rrFingerSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_gripper_r_finger_joint/set_velocity");
  lShoulderRollSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/l_shoulder_pan_joint/set_velocity");
  rShoulderRollSpeedClient = n.serviceClient<webots_ros::set_float>(controllerName + "/r_shoulder_pan_joint/set_velocity");

  fllWheelSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/fl_caster_l_wheel_joint/position_sensor/set_sensor");
  flRotationSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/fl_caster_rotation_joint/position_sensor/set_sensor");
  torsoSetClient = n.serviceClient<webots_ros::sensor_set>(controllerName + "/torso_lift_joint/position_sensor/set_sensor");
  llFingerSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/l_gripper_l_finger_joint/position_sensor/set_sensor");
  rrFingerSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/r_gripper_r_finger_joint/position_sensor/set_sensor");
  lShoulderSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/l_shoulder_pan_joint/position_sensor/set_sensor");
  rShoulderSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/r_shoulder_pan_joint/position_sensor/set_sensor");
  lBumperSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/r_gripper_l_finger_tip_contact_sensor/set_sensor");
  rBumperSetClient =
    n.serviceClient<webots_ros::sensor_set>(controllerName + "/l_gripper_l_finger_tip_contact_sensor/set_sensor");
  imuSetClient = n.serviceClient<webots_ros::sensor_set>(controllerName + "/imu_sensor/set_sensor");
  cameraSetClient = n.serviceClient<webots_ros::sensor_set>(controllerName + "/wide_stereo_l_stereo_camera_sensor/set_sensor");

  fllWheelAccelerationSrv.request.value = ACCELERATION;
  fllWheelAccelerationClient.call(fllWheelAccelerationSrv);
  flrWheelAccelerationSrv.request.value = ACCELERATION;
  flrWheelAccelerationClient.call(flrWheelAccelerationSrv);
  frlWheelAccelerationSrv.request.value = ACCELERATION;
  frlWheelAccelerationClient.call(frlWheelAccelerationSrv);
  frrWheelAccelerationSrv.request.value = ACCELERATION;
  frrWheelAccelerationClient.call(frrWheelAccelerationSrv);
  bllWheelAccelerationSrv.request.value = ACCELERATION;
  bllWheelAccelerationClient.call(bllWheelAccelerationSrv);
  blrWheelAccelerationSrv.request.value = ACCELERATION;
  blrWheelAccelerationClient.call(blrWheelAccelerationSrv);
  brlWheelAccelerationSrv.request.value = ACCELERATION;
  brlWheelAccelerationClient.call(brlWheelAccelerationSrv);
  brrWheelAccelerationSrv.request.value = ACCELERATION;
  brrWheelAccelerationClient.call(brrWheelAccelerationSrv);

  fllWheelSetClient.call(setSensorSrv);
  flRotationSetClient.call(setSensorSrv);
  torsoSetClient.call(setSensorSrv);
  llFingerSetClient.call(setSensorSrv);
  rrFingerSetClient.call(setSensorSrv);
  lShoulderSetClient.call(setSensorSrv);
  rShoulderSetClient.call(setSensorSrv);
  lBumperSetClient.call(setSensorSrv);
  rBumperSetClient.call(setSensorSrv);
  imuSetClient.call(setSensorSrv);
  cameraSetClient.call(setSensorSrv);

  ros::Subscriber fllWheelSubscriber =
    n.subscribe(controllerName + "/fl_caster_l_wheel_joint/position_sensor/32", 1, fllWheelCallback);
  ros::Subscriber flRotationSubscriber =
    n.subscribe(controllerName + "/fl_caster_rotation_joint/position_sensor/32", 1, flRotationCallback);
  ros::Subscriber torsoSubscriber;
  ros::Subscriber lBumperSubscriber;
  ros::Subscriber rBumperSubscriber;
  ros::Subscriber rrFingerSubscriber;
  ros::Subscriber lShoulderSubscriber;
  ros::Subscriber llFingerSubscriber;
  ros::Subscriber rShoulderSubscriber;
  ros::Subscriber imuSubscriber;

  int i = 0;
  int step = 0;
  int sequence = 0;
  double finger_pos = 0;

  // idle state
  lShoulderLiftSrv.request.value = 1.35;
  lShoulderLiftClient.call(lShoulderLiftSrv);
  rShoulderLiftSrv.request.value = 1.35;
  rShoulderLiftClient.call(rShoulderLiftSrv);
  lElbowLiftSrv.request.value = -2.2;
  lElbowLiftClient.call(lElbowLiftSrv);
  rElbowLiftSrv.request.value = -2.2;
  rElbowLiftClient.call(rElbowLiftSrv);
  llFingerSrv.request.value = 0.4;
  llFingerClient.call(llFingerSrv);
  lrFingerSrv.request.value = 0.4;
  lrFingerClient.call(lrFingerSrv);
  rlFingerSrv.request.value = 0.05;
  rlFingerClient.call(rlFingerSrv);
  rrFingerSrv.request.value = 0.2;
  rrFingerClient.call(rrFingerSrv);
  torsoSrv.request.value = 0.33;
  torsoClient.call(torsoSrv);

  while (ros::ok()) {
    step++;
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
      ROS_ERROR("Failed to call service time_step for next step.");

    ros::spinOnce();
    ros::spinOnce();

    // turn wheels to be able to rotate on itself
    if (rotationPosition > -0.78 && sequence < 2) {
      if (sequence == 0) {
        rotate_wheels(-0.785);
        imuSubscriber = n.subscribe(controllerName + "/imu_sensor/32", 1, imuCallback);
      }
      sequence = 1;
    }
    // turn 180° on itself to face fridge
    else if (imu_rpy[2] > -2.75 && sequence < 3) {
      if (sequence == 1) {
        ROS_INFO("PR2 rotates to face the fridge.");
        rotate(-ROTATE_SPEED);
      }
      sequence = 2;
    }
    // turn wheels to be able to go forward
    else if (rotationPosition < 0 && sequence < 4) {
      if (sequence == 2) {
        rotate_wheels(0);
        imuSubscriber.shutdown();
      }
      sequence = 3;
    }
    // move in front of the fridge
    else if (wheelPosition < 45 && sequence < 5) {
      if (sequence == 3) {
        ROS_INFO("PR2 moves to the fridge.");
        flRotationSubscriber.shutdown();
        torsoSubscriber = n.subscribe(controllerName + "/torso_lift_joint/position_sensor/32", 1, torsoCallback);
        move(FWD_SPEED);
      }
      sequence = 4;
    }
    // move right hand in front of the door's handle
    else if (torsoPosition < 0.325 && sequence < 6) {
      if (sequence == 4) {
        ROS_INFO("PR2 places its right hand in front of the door's handle.");
        move(0);
        rShoulderRollSrv.request.value = 0.41;
        rShoulderRollClient.call(rShoulderRollSrv);
        rShoulderRollSpeedSrv.request.value = 0.21;
        rShoulderRollSpeedClient.call(rShoulderRollSpeedSrv);
        rShoulderLiftSrv.request.value = -0.3;
        rShoulderLiftClient.call(rShoulderLiftSrv);
        ruArmRollSrv.request.value = -3.14;
        ruArmRollClient.call(ruArmRollSrv);
        rElbowLiftSrv.request.value = -0.3;
        rElbowLiftClient.call(rElbowLiftSrv);
        rWristRollSrv.request.value = 1.57;
        rWristRollClient.call(rWristRollSrv);
      }
      sequence = 5;
    }
    // move the hand on the handle
    else if (wheelPosition < 46.5 && sequence < 7) {
      if (sequence == 5) {
        ROS_INFO("PR2 grabs the handle.");
        torsoSubscriber.shutdown();
        rBumperSubscriber =
          n.subscribe(controllerName + "/r_gripper_l_finger_tip_contact_sensor/32_bumper", 1, rBumperCallback);
        move(FWD_SPEED / 3);
      }
      sequence = 6;
    }
    // grab the handle of the door
    else if (!rBumper && sequence < 8) {
      if (sequence == 6) {
        move(0);
        rlFingerSrv.request.value = 0;
        rlFingerClient.call(rlFingerSrv);
        rrFingerSrv.request.value = 0;
        rrFingerClient.call(rrFingerSrv);
      }
      sequence = 7;
    }
    // open the door
    else if (wheelPosition > 39 && sequence < 9) {
      if (sequence == 7) {
        ROS_INFO("PR2 opens the door.");
        rBumperSubscriber.shutdown();
        rrFingerSubscriber = n.subscribe(controllerName + "/r_gripper_r_finger_joint/position_sensor/32", 1, rrFingerCallback);
        rlFingerSpeedSrv.request.value = 0;
        rlFingerSpeedClient.call(rlFingerSpeedSrv);
        rrFingerSpeedSrv.request.value = 0;
        rrFingerSpeedClient.call(rrFingerSpeedSrv);
        rShoulderRollSrv.request.value = -0.25;
        rShoulderRollClient.call(rShoulderRollSrv);
        move(-FWD_SPEED / 2);
      }
      sequence = 8;
    }
    // release the door's handle
    else if (rrFingerPosition < 0.39 && sequence < 10) {
      if (sequence == 8) {
        move(0);
        rlFingerSrv.request.value = 0.4;
        rlFingerClient.call(rlFingerSrv);
        rrFingerSrv.request.value = 0.4;
        rrFingerClient.call(rrFingerSrv);
        rlFingerSpeedSrv.request.value = 0.2;
        rlFingerSpeedClient.call(rlFingerSpeedSrv);
        rrFingerSpeedSrv.request.value = 0.2;
        rrFingerSpeedClient.call(rrFingerSpeedSrv);
        rShoulderRollSrv.request.value = -0.5;
        rShoulderRollClient.call(rShoulderRollSrv);
        rShoulderRollSpeedSrv.request.value = 1;
        rShoulderRollSpeedClient.call(rShoulderRollSpeedSrv);
      }
      sequence = 9;
    }
    // pull back right arm, extend left one and move a little forward
    else if (wheelPosition < 42 && sequence < 11) {
      if (sequence == 9) {
        rrFingerSubscriber.shutdown();
        lShoulderSubscriber = n.subscribe(controllerName + "/l_shoulder_pan_joint/position_sensor/32", 1, lShoulderCallback);
        lShoulderLiftSrv.request.value = 0;
        lShoulderLiftClient.call(lShoulderLiftSrv);
        rShoulderLiftSrv.request.value = 1.2;
        rShoulderLiftClient.call(rShoulderLiftSrv);
        ruArmRollSrv.request.value = 0;
        ruArmRollClient.call(ruArmRollSrv);
        lElbowLiftSrv.request.value = 0;
        lElbowLiftClient.call(lElbowLiftSrv);
        move(FWD_SPEED / 3);
      }
      sequence = 10;
    }
    // push the door to stay wide open
    else if (lShoulderPosition > -0.55 && sequence < 12) {
      if (sequence == 10) {
        move(0);
        rElbowLiftSrv.request.value = -2.3;
        rElbowLiftClient.call(rElbowLiftSrv);
        lShoulderRollSrv.request.value = -0.57;
        lShoulderRollClient.call(lShoulderRollSrv);
        lShoulderRollSpeedSrv.request.value = 0.6;
        lShoulderRollSpeedClient.call(lShoulderRollSpeedSrv);
      }
      sequence = 11;
    } else if (step < 1260 && sequence < 13) {
      if (sequence == 11) {
        headTiltSrv.request.value = 0.4;
        headTiltClient.call(headTiltSrv);
      }
      sequence = 12;
    }
    // move forward next to the fridge
    else if (wheelPosition < 48 && sequence < 14) {
      if (sequence == 12) {
        ROS_INFO("PR2 moves closer to the fridge.");
        move(FWD_SPEED / 2);
        rShoulderRollSrv.request.value = -0.1;
        rShoulderRollClient.call(rShoulderRollSrv);
        lShoulderLiftSrv.request.value = 0.65;
        lShoulderLiftClient.call(lShoulderLiftSrv);
        lElbowLiftSrv.request.value = -0.65;
        lElbowLiftClient.call(lElbowLiftSrv);
        lShoulderRollSrv.request.value = -0.48;
        lShoulderRollClient.call(lShoulderRollSrv);
        lShoulderRollSpeedSrv.request.value = 0.01;
        lShoulderRollSpeedClient.call(lShoulderRollSpeedSrv);
      }
      sequence = 13;
    }
    // place the left hand in front of the can
    else if (lShoulderPosition < -0.236 && sequence < 15) {
      if (sequence == 13) {
        ROS_INFO("PR2 moves its left-hand around the can.");
        move(0);
        lShoulderRollSrv.request.value = -0.235;
        lShoulderRollClient.call(lShoulderRollSrv);
        lShoulderRollSpeedSrv.request.value = 1;
        lShoulderRollSpeedClient.call(lShoulderRollSpeedSrv);
      }
      sequence = 14;
    }
    // place left hand around the can
    else if (wheelPosition < 50.5 && sequence < 16) {
      if (sequence == 14) {
        lShoulderSubscriber.shutdown();
        llFingerSubscriber = n.subscribe(controllerName + "/l_gripper_l_finger_joint/position_sensor/32", 1, llFingerCallback);
        lBumperSubscriber =
          n.subscribe(controllerName + "/l_gripper_l_finger_tip_contact_sensor/32_bumper", 1, lBumperCallback);
        move(FWD_SPEED / 3);
      }
      sequence = 15;
    }
    // grab the can
    else if (llFingerPosition > (finger_pos - 0.04) && sequence < 17) {
      if (sequence == 15) {
        ROS_INFO("PR2 grabs the can.");
        move(0);
        llFingerSrv.request.value = 0;
        llFingerClient.call(llFingerSrv);
        lrFingerSrv.request.value = 0;
        lrFingerClient.call(lrFingerSrv);
        llFingerSpeedSrv.request.value = 0.015;
        llFingerSpeedClient.call(llFingerSpeedSrv);
        lrFingerSpeedSrv.request.value = 0.015;
        lrFingerSpeedClient.call(lrFingerSpeedSrv);
        llFingerTipSrv.request.value = 0.4;
        llFingerTipClient.call(llFingerTipSrv);
        lrFingerTipSrv.request.value = 0.4;
        lrFingerTipClient.call(lrFingerTipSrv);
      }
      if (i == 1) {
        finger_pos = llFingerPosition;
        i++;
      }
      if (lBumper)
        i++;
      sequence = 16;
    }
    // move backward out of the fridge
    else if (wheelPosition > 40 && sequence < 18) {
      if (sequence == 16) {
        llFingerSubscriber.shutdown();
        lBumperSubscriber.shutdown();
        rShoulderSubscriber = n.subscribe(controllerName + "/r_shoulder_pan_joint/position_sensor/32", 1, rShoulderCallback);
        move(-FWD_SPEED);
        llFingerSpeedSrv.request.value = 0;
        llFingerSpeedClient.call(llFingerSpeedSrv);
        lrFingerSpeedSrv.request.value = 0;
        lrFingerSpeedClient.call(lrFingerSpeedSrv);
      }
      sequence = 17;
    }
    // stop and open right shoulder
    else if (rShoulderPosition > -0.59 && sequence < 19) {
      if (sequence == 17) {
        move(0);
        rShoulderRollSrv.request.value = -0.6;
        rShoulderRollClient.call(rShoulderRollSrv);
      }
      sequence = 18;
    }
    // extend right arm along the door
    else if (wheelPosition < 40.55 && sequence < 20) {
      if (sequence == 18) {
        move(FWD_SPEED / 2);
        rShoulderLiftSrv.request.value = 0;
        rShoulderLiftClient.call(rShoulderLiftSrv);
        rElbowLiftSrv.request.value = 0;
        rElbowLiftClient.call(rElbowLiftSrv);
      }
      sequence = 19;
    }
    // close the door
    else if (rShoulderPosition < 0.5 && sequence < 21) {
      if (sequence == 19) {
        ROS_INFO("PR2 closes the door.");
        move(0);
        rShoulderRollSrv.request.value = 0.5;
        rShoulderRollClient.call(rShoulderRollSrv);
        rShoulderRollSpeedSrv.request.value = 0.18;
        rShoulderRollSpeedClient.call(rShoulderRollSpeedSrv);
        lShoulderRollSrv.request.value = 0;
        lShoulderRollClient.call(lShoulderRollSrv);
        lShoulderLiftSrv.request.value = 1;
        lShoulderLiftClient.call(lShoulderLiftSrv);
        lElbowLiftSrv.request.value = -1;
        lElbowLiftClient.call(lElbowLiftSrv);
        flRotationSetClient.call(setSensorSrv);
        flRotationSubscriber =
          n.subscribe(controllerName + "/fl_caster_rotation_joint/position_sensor/32", 1, flRotationCallback);
      }
      sequence = 20;
    } else if (wheelPosition < 46 && sequence < 22) {
      if (sequence == 20) {
        rShoulderRollSrv.request.value = 0.2;
        rShoulderRollClient.call(rShoulderRollSrv);
        move(FWD_SPEED / 4);
      }
      sequence = 21;
    }
    // rotate wheels to be able to turn on itself
    else if (rotationPosition > -0.78 && sequence < 23) {
      if (sequence == 21) {
        rotate_wheels(-0.785);
        imuSetClient.call(setSensorSrv);
        imuSubscriber = n.subscribe(controllerName + "/imu_sensor/32", 1, imuCallback);
        rShoulderSubscriber.shutdown();
      }
      sequence = 22;
    }
    // rotate 180° on itself
    else if (imu_rpy[2] < -0.35 && sequence < 24) {
      if (sequence == 22) {
        ROS_INFO("PR2 rotates back to go to the lounge.");
        rotate(ROTATE_SPEED);
        rShoulderRollSrv.request.value = 0;
        rShoulderRollClient.call(rShoulderRollSrv);
        rShoulderLiftSrv.request.value = 1.35;
        rShoulderLiftClient.call(rShoulderLiftSrv);
        rElbowLiftSrv.request.value = -2.2;
        rElbowLiftClient.call(rElbowLiftSrv);
        rWristRollSrv.request.value = 0;
        rWristRollClient.call(rWristRollSrv);
      }
      sequence = 23;
    }
    // rotate wheels to be able to move forward
    else if (rotationPosition < 0 && sequence < 25) {
      if (sequence == 23) {
        rotate_wheels(0);
        imuSubscriber.shutdown();
      }
      sequence = 24;
    }
    // go back to the table
    else if (wheelPosition < 80 && sequence < 26) {
      if (sequence == 24) {
        move(FWD_SPEED);
      }
      sequence = 25;
    }
    // rotate wheels to be able to turn on itself
    else if (rotationPosition > -0.78 && sequence < 27) {
      if (sequence == 25) {
        rotate_wheels(-0.785);
        imuSetClient.call(setSensorSrv);
        imuSubscriber = n.subscribe(controllerName + "/imu_sensor/32", 1, imuCallback);
      }
      sequence = 26;
    }
    // rotate 90° on itself to face table
    else if (imu_rpy[2] > -1.15 && sequence < 28) {
      if (sequence == 26) {
        ROS_INFO("PR2 faces the table.");
        torsoSetClient.call(setSensorSrv);
        torsoSubscriber = n.subscribe(controllerName + "/torso_lift_joint/position_sensor/32", 1, torsoCallback);
        rotate(-ROTATE_SPEED);
      }
      sequence = 27;
    }
    // lower the can to lay on the table
    else if (torsoPosition > 0.17 && sequence < 29) {
      if (sequence == 27) {
        ROS_INFO("PR2 slowly lay the can on the table.");
        move(0);
        torsoSrv.request.value = 0.165;
        torsoClient.call(torsoSrv);
        imuSubscriber.shutdown();
        llFingerSetClient.call(setSensorSrv);
        llFingerSubscriber = n.subscribe(controllerName + "/l_gripper_l_finger_joint/position_sensor/32", 1, llFingerCallback);
      }
      sequence = 28;
    }
    // release the can
    else if (llFingerPosition < 0.39 && sequence < 30) {
      if (sequence == 28) {
        ROS_INFO("PR2 release the can and go back to stand-by position.");
        llFingerSrv.request.value = 0.4;
        llFingerClient.call(llFingerSrv);
        lrFingerSrv.request.value = 0.4;
        lrFingerClient.call(lrFingerSrv);
        llFingerTipSrv.request.value = 0;
        llFingerTipClient.call(llFingerTipSrv);
        lrFingerTipSrv.request.value = 0;
        lrFingerTipClient.call(lrFingerTipSrv);
        llFingerSpeedSrv.request.value = 0.1;
        llFingerSpeedClient.call(llFingerSpeedSrv);
        lrFingerSpeedSrv.request.value = 0.1;
        lrFingerSpeedClient.call(lrFingerSpeedSrv);
      }
      sequence = 29;
    }
    // bring back arm to initial position
    else if (sequence == 29) {
      lElbowLiftSrv.request.value = -2.2;
      lElbowLiftClient.call(lElbowLiftSrv);
      lShoulderLiftSrv.request.value = 1.35;
      lShoulderLiftClient.call(lShoulderLiftSrv);
      break;
    }
    ros::spinOnce();
  }
  ros::spinOnce();
  setSensorSrv.request.period = 0;
  fllWheelSubscriber.shutdown();
  flRotationSubscriber.shutdown();
  torsoSubscriber.shutdown();

  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  return (0);
}
