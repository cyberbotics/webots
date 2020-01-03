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

#include <signal.h>
#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/save_image.h>

#define TIME_STEP 32;

static int controllerCount;
static std::vector<std::string> controllerList;
static std::vector<float> imageRangeFinder;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

// get range image from the range-finder
void rangeFinderCallback(const sensor_msgs::Image::ConstPtr &image) {
  int size = image->width * image->height;
  imageRangeFinder.resize(size);

  const float *depth_data = reinterpret_cast<const float *>(&image->data[0]);
  for (int i = 0; i < size; ++i)
    imageRangeFinder[i] = depth_data[i];
}

void quit(int sig) {
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the 'catch_the_bird' node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  std::string controllerName, motorName, rangeFinderName;
  std::vector<std::string> deviceList;
  int width, height;
  float i, step;
  bool birdCatched = false;

  // create a node named 'catch_the_bird' on ROS network
  ros::init(argc, argv, "catch_the_bird", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
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
      ROS_ERROR("Invalid number for  controller choice.");
      return 1;
    }
  }
  // leave topic once it's not necessary anymore
  nameSub.shutdown();

  // call device_list service to get the list of the devices available on the controller and print it the device_list_srv object
  // contains 2 members request and response. Their fields are described in the corresponding .srv file
  ros::ServiceClient deviceListClient =
    n.serviceClient<webots_ros::robot_get_device_list>(controllerName + "/robot/get_device_list");
  webots_ros::robot_get_device_list deviceListSrv;

  if (deviceListClient.call(deviceListSrv))
    deviceList = deviceListSrv.response.list;
  else
    ROS_ERROR("Failed to call service device_list.");
  motorName = deviceList[0];
  rangeFinderName = deviceList[1];

  ros::ServiceClient rangeFinderGetInfoClient =
    n.serviceClient<webots_ros::range_finder_get_info>(controllerName + '/' + rangeFinderName + "/get_info");
  webots_ros::range_finder_get_info rangeFinderGetInfoSrv;
  if (rangeFinderGetInfoClient.call(rangeFinderGetInfoSrv)) {
    width = rangeFinderGetInfoSrv.response.width;
    height = rangeFinderGetInfoSrv.response.height;
    ROS_INFO("Range-finder size is %d x %d.", width, height);
  } else
    ROS_ERROR("Failed to call service range_finder_get_info.");

  // enable the range-finder
  ros::ServiceClient enableRangeFinderClient =
    n.serviceClient<webots_ros::set_int>(controllerName + '/' + rangeFinderName + "/enable");
  webots_ros::set_int enableRangeFinderSrv;
  ros::Subscriber subRangeFinderRangeFinder;

  enableRangeFinderSrv.request.value = 2 * TIME_STEP;
  if (enableRangeFinderClient.call(enableRangeFinderSrv) && enableRangeFinderSrv.response.success) {
    ROS_INFO("Range-finder enabled with sampling period %d.", enableRangeFinderSrv.request.value);
    subRangeFinderRangeFinder = n.subscribe(controllerName + '/' + rangeFinderName + "/range_image", 1, rangeFinderCallback);

    // wait for  the topics to be initialized
    while (subRangeFinderRangeFinder.getNumPublishers() == 0) {
    }
  } else
    ROS_ERROR("Failed to call service enable for %s.", rangeFinderName.c_str());

  ros::ServiceClient rangeFinderSaveImageClient =
    n.serviceClient<webots_ros::save_image>(controllerName + '/' + rangeFinderName + "/save_image");
  webots_ros::save_image rangeFinderSaveImageSrv;
  rangeFinderSaveImageSrv.request.filename = std::string(getenv("HOME")) + std::string("/bird_catched.png");
  rangeFinderSaveImageSrv.request.quality = 100;

  // enable motor
  ros::ServiceClient motorSetPositionClient =
    n.serviceClient<webots_ros::set_float>(controllerName + '/' + motorName + "/set_position");
  webots_ros::set_float motorSetPositionSrv;
  motorSetPositionSrv.request.value = 0;
  i = 0.2;
  step = 0.025;

  ros::ServiceClient motorGetTargetPositionClient =
    n.serviceClient<webots_ros::get_float>(controllerName + '/' + motorName + "/get_target_position");
  webots_ros::get_float motorGetTargetPositionSrv;

  // enable time_step
  timeStepClient = n.serviceClient<webots_ros::set_int>(controllerName + "/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;

  // main loop
  while (!birdCatched && ros::ok()) {
    motorSetPositionSrv.request.value = i;
    motorSetPositionClient.call(motorSetPositionSrv);
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call next step with time_step service.");
      exit(1);
    }
    motorGetTargetPositionClient.call(motorGetTargetPositionSrv);
    if (i >= 3.14)
      step = -0.025;
    if (i <= -3.14)
      step = 0.025;
    i += step;
    ros::spinOnce();
    while (imageRangeFinder.size() < (width * height))
      ros::spinOnce();
    // check if it sees the bird and take a picture of the bird
    if (imageRangeFinder[12 + (width * height / 4)] < 0.5) {
      birdCatched = true;
      if (rangeFinderSaveImageClient.call(rangeFinderSaveImageSrv) && rangeFinderSaveImageSrv.response.success == 1)
        ROS_INFO("What a beautifull bird we found here!");
      else
        ROS_INFO("Failed to call service save_image to take a picture of the bird.");
    }
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  n.shutdown();
}
