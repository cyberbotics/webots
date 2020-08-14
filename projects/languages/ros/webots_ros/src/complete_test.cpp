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

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <signal.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <webots_ros/BoolStamped.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Int8Stamped.h>
#include <webots_ros/RadarTarget.h>
#include <webots_ros/RecognitionObject.h>
#include <webots_ros/StringStamped.h>
#include <cstdlib>
#include "ros/ros.h"

#include <webots_ros/get_bool.h>
#include <webots_ros/get_float.h>
#include <webots_ros/get_float_array.h>
#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/get_urdf.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_float_array.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>

#include <webots_ros/camera_get_focus_info.h>
#include <webots_ros/camera_get_info.h>
#include <webots_ros/camera_get_zoom_info.h>
#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_draw_oval.h>
#include <webots_ros/display_draw_pixel.h>
#include <webots_ros/display_draw_polygon.h>
#include <webots_ros/display_draw_rectangle.h>
#include <webots_ros/display_draw_text.h>
#include <webots_ros/display_get_info.h>
#include <webots_ros/display_image_copy.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>
#include <webots_ros/display_image_save.h>
#include <webots_ros/display_set_font.h>
#include <webots_ros/field_get_bool.h>
#include <webots_ros/field_get_color.h>
#include <webots_ros/field_get_count.h>
#include <webots_ros/field_get_float.h>
#include <webots_ros/field_get_int32.h>
#include <webots_ros/field_get_node.h>
#include <webots_ros/field_get_rotation.h>
#include <webots_ros/field_get_string.h>
#include <webots_ros/field_get_type.h>
#include <webots_ros/field_get_type_name.h>
#include <webots_ros/field_get_vec2f.h>
#include <webots_ros/field_get_vec3f.h>
#include <webots_ros/field_import_node.h>
#include <webots_ros/field_remove.h>
#include <webots_ros/field_set_bool.h>
#include <webots_ros/field_set_color.h>
#include <webots_ros/field_set_float.h>
#include <webots_ros/field_set_int32.h>
#include <webots_ros/field_set_rotation.h>
#include <webots_ros/field_set_string.h>
#include <webots_ros/field_set_vec2f.h>
#include <webots_ros/field_set_vec3f.h>
#include <webots_ros/lidar_get_frequency_info.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/motor_set_control_pid.h>
#include <webots_ros/node_add_force_or_torque.h>
#include <webots_ros/node_add_force_with_offset.h>
#include <webots_ros/node_get_center_of_mass.h>
#include <webots_ros/node_get_contact_point.h>
#include <webots_ros/node_get_field.h>
#include <webots_ros/node_get_id.h>
#include <webots_ros/node_get_name.h>
#include <webots_ros/node_get_number_of_contact_points.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/node_get_parent_node.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_static_balance.h>
#include <webots_ros/node_get_status.h>
#include <webots_ros/node_get_type.h>
#include <webots_ros/node_get_velocity.h>
#include <webots_ros/node_remove.h>
#include <webots_ros/node_reset_functions.h>
#include <webots_ros/node_set_velocity.h>
#include <webots_ros/node_set_visibility.h>
#include <webots_ros/pen_set_ink_color.h>
#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/receiver_get_emitter_direction.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/robot_set_mode.h>
#include <webots_ros/robot_wait_for_user_input_event.h>
#include <webots_ros/save_image.h>
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/supervisor_get_from_id.h>
#include <webots_ros/supervisor_movie_start_recording.h>
#include <webots_ros/supervisor_set_label.h>
#include <webots_ros/supervisor_virtual_reality_headset_get_orientation.h>
#include <webots_ros/supervisor_virtual_reality_headset_get_position.h>

#define TIME_STEP 32

using namespace std;

static int model_count;
static vector<string> model_list;
static vector<unsigned char> imageColor;
static vector<float> imageRangeFinder;
static int connectorPresence = 0;
static double accelerometerValues[3] = {0, 0, 0};
static double compassValues[3] = {0, 0, 0};
static double GPSValues[3] = {0, 0, 0};
static double GyroValues[3] = {0, 0, 0};
static double inertialUnitValues[4] = {0, 0, 0, 0};
static double touchSensorValues[3] = {0, 0, 0};
static bool callbackCalled = false;

ros::ServiceClient time_step_client;
webots_ros::set_int time_step_srv;

void modelNameCallback(const std_msgs::String::ConstPtr &name) {
  model_count++;
  model_list.push_back(name->data);
  ROS_INFO("Model #%d: %s.", model_count, model_list.back().c_str());
  callbackCalled = true;
}

void cameraCallback(const sensor_msgs::Image::ConstPtr &values) {
  int i = 0;
  imageColor.resize(values->step * values->height);
  for (std::vector<unsigned char>::const_iterator it = values->data.begin(); it != values->data.end(); ++it) {
    imageColor[i] = *it;
    i++;
  }
  callbackCalled = true;
}

void cameraRecognitionCallback(const webots_ros::RecognitionObject::ConstPtr &object) {
  ROS_INFO("Camera recognition saw a '%s' (time: %d:%d).", object->model.c_str(), object->header.stamp.sec,
           object->header.stamp.nsec);
  callbackCalled = true;
}

void joystickCallback(const webots_ros::Int8Stamped::ConstPtr &value) {
  ROS_INFO("Joystick button pressed: %d (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value) {
  ROS_INFO("Keyboard key pressed: %d (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void radarTargetsCallback(const webots_ros::RadarTarget::ConstPtr &target) {
  ROS_INFO("Received a radar target with distance=%lf received power=%lf speed=%lf azimuth=%lf (time: %d:%d).",
           target->distance, target->receivedPower, target->speed, target->azimuth, target->header.stamp.sec,
           target->header.stamp.nsec);
  callbackCalled = true;
}

void radarTargetsNumberCallback(const webots_ros::Int8Stamped::ConstPtr &value) {
  ROS_INFO("Number of target seen by the radar: %d (time: %d:%d).", value->data, value->header.stamp.sec,
           value->header.stamp.nsec);
  callbackCalled = true;
}

void rangeFinderCallback(const sensor_msgs::Image::ConstPtr &image) {
  int size = image->width * image->height;
  imageRangeFinder.resize(size);

  const float *depth_data = reinterpret_cast<const float *>(&image->data[0]);
  for (int i = 0; i < size; ++i)
    imageRangeFinder[i] = depth_data[i];
  callbackCalled = true;
}

void lidarCallback(const sensor_msgs::Image::ConstPtr &image) {
  callbackCalled = true;
}

void connectorCallback(const webots_ros::Int8Stamped::ConstPtr &value) {
  connectorPresence = value->data;

  ROS_INFO("Connector presence: %d (time: %d:%d).", connectorPresence, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void accelerometerCallback(const sensor_msgs::Imu::ConstPtr &values) {
  accelerometerValues[0] = values->linear_acceleration.x;
  accelerometerValues[1] = values->linear_acceleration.y;
  accelerometerValues[2] = values->linear_acceleration.z;

  ROS_INFO("Accelerometer values are x=%f y=%f z=%f (time: %d:%d).", accelerometerValues[0], accelerometerValues[1],
           accelerometerValues[2], values->header.stamp.sec, values->header.stamp.nsec);
  callbackCalled = true;
}

void battery_sensorCallback(const webots_ros::Float64Stamped::ConstPtr &value) {
  ROS_INFO("Battery level is %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void compassCallback(const sensor_msgs::MagneticField::ConstPtr &values) {
  compassValues[0] = values->magnetic_field.x;
  compassValues[1] = values->magnetic_field.y;
  compassValues[2] = values->magnetic_field.z;

  ROS_INFO("Compass values are x=%f y=%f z=%f (time: %d:%d).", compassValues[0], compassValues[1], compassValues[2],
           values->header.stamp.sec, values->header.stamp.nsec);
  callbackCalled = true;
}

void distance_sensorCallback(const sensor_msgs::Range::ConstPtr &value) {
  ROS_INFO("Distance from object is %f (time: %d:%d).", value->range, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
  GPSValues[0] = values->latitude;
  GPSValues[1] = values->altitude;
  GPSValues[2] = values->longitude;

  ROS_INFO("GPS values are x=%f y=%f z=%f (time: %d:%d).", GPSValues[0], GPSValues[1], GPSValues[2], values->header.stamp.sec,
           values->header.stamp.nsec);
  callbackCalled = true;
}

void GPSSpeedCallback(const webots_ros::Float64Stamped::ConstPtr &value) {
  ROS_INFO("GPS speed is: %fkm/h (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void gyroCallback(const sensor_msgs::Imu::ConstPtr &values) {
  GyroValues[0] = values->angular_velocity.x;
  GyroValues[1] = values->angular_velocity.y;
  GyroValues[2] = values->angular_velocity.z;

  ROS_INFO("Gyro values are x=%f y=%f z=%f (time: %d:%d).", GyroValues[0], GyroValues[1], GyroValues[2],
           values->header.stamp.sec, values->header.stamp.nsec);
  callbackCalled = true;
}

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[0] = values->orientation.x;
  inertialUnitValues[1] = values->orientation.y;
  inertialUnitValues[2] = values->orientation.z;
  inertialUnitValues[3] = values->orientation.w;

  ROS_INFO("Inertial unit values (quaternions) are x=%f y=%f z=%f w=%f (time: %d:%d).", inertialUnitValues[0],
           inertialUnitValues[1], inertialUnitValues[2], inertialUnitValues[2], values->header.stamp.sec,
           values->header.stamp.nsec);
  callbackCalled = true;
}

void lightSensorCallback(const sensor_msgs::Illuminance::ConstPtr &value) {
  ROS_INFO("Light intensity is %f.", value->illuminance);
  callbackCalled = true;
}

void motorSensorCallback(const webots_ros::Float64Stamped::ConstPtr &value) {
  ROS_INFO("Motor sensor sent value %f.", value->data);
  callbackCalled = true;
}

void positionSensorCallback(const webots_ros::Float64Stamped::ConstPtr &value) {
  ROS_INFO("Position sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void touchSensorCallback(const webots_ros::Float64Stamped::ConstPtr &value) {
  ROS_INFO("Touch sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void touchSensorBumperCallback(const webots_ros::BoolStamped::ConstPtr &value) {
  ROS_INFO("Touch sensor sent value %d (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  callbackCalled = true;
}

void touchSensor3DCallback(const geometry_msgs::WrenchStamped::ConstPtr &values) {
  touchSensorValues[0] = values->wrench.force.x;
  touchSensorValues[1] = values->wrench.force.y;
  touchSensorValues[2] = values->wrench.force.z;

  ROS_INFO("Touch sensor values are x = %f, y = %f and z = %f (time: %d:%d).", touchSensorValues[0], touchSensorValues[1],
           touchSensorValues[2], values->header.stamp.sec, values->header.stamp.nsec);
  callbackCalled = true;
}

void receiverCallback(const webots_ros::StringStamped::ConstPtr &value) {
  char *message = const_cast<char *>(value->data.c_str());
  ROS_INFO("Received a message %s.", message);
  callbackCalled = true;
}

void quit(int sig) {
  time_step_srv.request.value = 0;
  time_step_client.call(time_step_srv);
  ROS_INFO("User stopped the 'complete_test' node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  string model_name = "my_robot";

  ros::init(argc, argv, "complete_test", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);

  ros::Subscriber name_sub = n.subscribe("model_name", 100, modelNameCallback);
  while (model_count == 0 || model_count < name_sub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();
  name_sub.shutdown();

  ////////////////////////
  // ROBOT METHODS TEST //
  ////////////////////////

  int mode = 0;
  string model;
  string path;
  string data;
  vector<string> device_list;

  time_step_client = n.serviceClient<webots_ros::set_int>(model_name + "/robot/time_step");
  time_step_srv.request.value = TIME_STEP;

  if (time_step_client.call(time_step_srv) && time_step_srv.response.success)
    ROS_INFO("time_step service works.");
  else
    ROS_ERROR("Failed to call service time_step to update robot's time step.");

  ros::ServiceClient get_number_of_devices_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/robot/get_number_of_devices");
  webots_ros::get_int get_number_of_devices_srv;

  if (get_number_of_devices_client.call(get_number_of_devices_srv)) {
    int number_of_devices = get_number_of_devices_srv.response.value;
    ROS_INFO("%s has %d devices.", model_name.c_str(), number_of_devices);
  } else
    ROS_ERROR("Failed to call service get_number_of_devices.");

  get_number_of_devices_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient device_list_client =
    n.serviceClient<webots_ros::robot_get_device_list>(model_name + "/robot/get_device_list");
  webots_ros::robot_get_device_list device_list_srv;

  if (device_list_client.call(device_list_srv)) {
    device_list = device_list_srv.response.list;
    for (unsigned int i = 0; i < device_list.size(); i++)
      ROS_INFO("Device [%d]: %s.", i, device_list[i].c_str());
  } else
    ROS_ERROR("Failed to call service device_list.");

  device_list_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient urdf_client = n.serviceClient<webots_ros::get_urdf>(model_name + "/robot/get_urdf");
  webots_ros::get_urdf urdf_srv;
  urdf_srv.request.prefix = "unique_robot_prefix_name_";

  if (urdf_client.call(urdf_srv)) {
    std::string urdf = urdf_srv.response.value;
    if (urdf.find(urdf_srv.request.prefix) == std::string::npos)
      ROS_ERROR("Invalid response from get_urdf.");
    else
      ROS_INFO("URDF has been successfully obtained.");
  } else
    ROS_ERROR("Failed to call service get_urdf.");

  urdf_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_basic_time_step_client =
    n.serviceClient<webots_ros::get_float>(model_name + "/robot/get_basic_time_step");
  webots_ros::get_float get_basic_time_step_srv;

  if (get_basic_time_step_client.call(get_basic_time_step_srv)) {
    double basic_time_step = get_basic_time_step_srv.response.value;
    ROS_INFO("%s has a basic time step of %f.", model_name.c_str(), basic_time_step);
  } else
    ROS_ERROR("Failed to call service get_basic_time_step.");

  get_basic_time_step_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient robot_get_custom_data_client =
    n.serviceClient<webots_ros::get_string>(model_name + "/robot/get_custom_data");
  webots_ros::get_string robot_get_custom_data_srv;

  if (robot_get_custom_data_client.call(robot_get_custom_data_srv)) {
    data = robot_get_custom_data_srv.response.value;
    ROS_INFO("Data of %s is %s.", model_name.c_str(), data.c_str());
  } else
    ROS_ERROR("Failed to call service robot_get_custom_data.");

  robot_get_custom_data_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_mode_client = n.serviceClient<webots_ros::get_int>(model_name + "/robot/get_mode");
  webots_ros::get_int get_mode_srv;

  if (get_mode_client.call(get_mode_srv)) {
    mode = get_mode_srv.response.value;
    ROS_INFO("Mode of %s is %d.", model_name.c_str(), mode);
  } else
    ROS_ERROR("Failed to call service get_mode.");

  get_mode_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_model_client = n.serviceClient<webots_ros::get_string>(model_name + "/robot/get_model");
  webots_ros::get_string get_model_srv;

  if (get_model_client.call(get_model_srv)) {
    model = get_model_srv.response.value;
    ROS_INFO("Model of %s is %s.", model_name.c_str(), model.c_str());
  } else
    ROS_ERROR("Failed to call service get_model.");

  get_model_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_project_path_client = n.serviceClient<webots_ros::get_string>(model_name + "/robot/get_project_path");
  webots_ros::get_string get_project_path_srv;

  if (get_project_path_client.call(get_project_path_srv)) {
    path = get_project_path_srv.response.value;
    ROS_INFO("World path of %s is %s.", model_name.c_str(), path.c_str());
  } else
    ROS_ERROR("Failed to call service get_project_path.");

  get_project_path_client.shutdown();
  time_step_client.call(time_step_srv);

  // robot_get_world_path
  ros::ServiceClient get_world_path_client = n.serviceClient<webots_ros::get_string>(model_name + "/robot/get_world_path");
  webots_ros::get_string get_world_path_srv;

  if (get_world_path_client.call(get_world_path_srv)) {
    path = get_world_path_srv.response.value;
    ROS_INFO("Project path of %s is %s.", model_name.c_str(), path.c_str());
  } else
    ROS_ERROR("Failed to call service get_project_path.");

  get_world_path_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_supervisor_client = n.serviceClient<webots_ros::get_bool>(model_name + "/robot/get_supervisor");
  webots_ros::get_bool get_supervisor_srv;

  if (get_supervisor_client.call(get_supervisor_srv)) {
    if (get_supervisor_srv.response.value)
      ROS_INFO("%s is a supervisor.", model_name.c_str());
    else
      ROS_ERROR("%s isn't a supervisor.", model_name.c_str());
  } else
    ROS_ERROR("Failed to call service get_synchronization.");

  get_supervisor_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_synchronization_client =
    n.serviceClient<webots_ros::get_bool>(model_name + "/robot/get_synchronization");
  webots_ros::get_bool get_synchronization_srv;

  if (get_synchronization_client.call(get_synchronization_srv)) {
    bool synchronization = get_synchronization_srv.response.value;
    if (synchronization)
      ROS_INFO("%s is sync.", model_name.c_str());
    else
      ROS_INFO("%s isn't sync.", model_name.c_str());
  } else
    ROS_ERROR("Failed to call service get_synchronization.");

  get_synchronization_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_time_client = n.serviceClient<webots_ros::get_float>(model_name + "/robot/get_time");
  webots_ros::get_float get_time_srv;

  if (get_time_client.call(get_time_srv)) {
    double time = get_time_srv.response.value;
    ROS_INFO("Time for %s is %f.", model_name.c_str(), time);
  } else
    ROS_ERROR("Failed to call service get_time.");

  get_time_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_type_client = n.serviceClient<webots_ros::get_int>(model_name + "/robot/get_type");
  webots_ros::get_int get_type_srv;

  if (get_type_client.call(get_type_srv)) {
    int type = get_type_srv.response.value;
    ROS_INFO("Type of %s is %d.", model_name.c_str(), type);
  } else
    ROS_ERROR("Failed to call service get_type.");

  get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient robot_set_custom_data_client =
    n.serviceClient<webots_ros::set_string>(model_name + "/robot/set_custom_data");
  webots_ros::set_string robot_set_custom_data_srv;

  robot_set_custom_data_srv.request.value = "OVERWRITTEN";
  if (robot_set_custom_data_client.call(robot_set_custom_data_srv)) {
    if (robot_set_custom_data_srv.response.success)
      ROS_INFO("Data of %s has been set to %s.", model_name.c_str(), data.c_str());
  } else
    ROS_ERROR("Failed to call service robot_set_custom_data.");

  robot_set_custom_data_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_mode_client = n.serviceClient<webots_ros::robot_set_mode>(model_name + "/robot/set_mode");
  webots_ros::robot_set_mode set_mode_srv;

  set_mode_srv.request.mode = mode;
  if (set_mode_client.call(set_mode_srv)) {
    if (set_mode_srv.response.success == 1)
      ROS_INFO("Mode of %s has been set to %d.", model_name.c_str(), mode);
  } else
    ROS_ERROR("Failed to call service set_mode.");

  set_mode_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient enable_keyboard_client = n.serviceClient<webots_ros::set_int>(model_name + "/keyboard/enable");
  webots_ros::set_int enable_keyboard_srv;
  ros::Subscriber sub_keyboard;

  enable_keyboard_srv.request.value = 32;
  if (enable_keyboard_client.call(enable_keyboard_srv) && enable_keyboard_srv.response.success) {
    ROS_INFO("Keyboard of %s has been enabled.", model_name.c_str());
    sub_keyboard = n.subscribe(model_name + "/keyboard/key", 1, keyboardCallback);
    ROS_INFO("Topics for keyboard initialized.");
    callbackCalled = false;
    while (sub_keyboard.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topics for keyboard connected.");
  } else
    ROS_ERROR("Failed to enable keyboard.");

  ros::ServiceClient wait_for_user_input_event_client =
    n.serviceClient<webots_ros::robot_wait_for_user_input_event>(model_name + "/robot/wait_for_user_input_event");
  webots_ros::robot_wait_for_user_input_event wait_for_user_input_event_srv;

  wait_for_user_input_event_srv.request.eventType = 4;  // WB_EVENT_KEYBOARD
  wait_for_user_input_event_srv.request.timeout = 20;
  if (wait_for_user_input_event_client.call(wait_for_user_input_event_srv))
    ROS_INFO("Detected user input event: %d.", wait_for_user_input_event_srv.response.event);
  else
    ROS_ERROR("Failed to call service wait_for_user_input_event.");

  wait_for_user_input_event_client.shutdown();
  time_step_client.call(time_step_srv);

  sub_keyboard.shutdown();
  enable_keyboard_client.shutdown();
  time_step_client.call(time_step_srv);

  /////////////////////////
  // BREAK METHODS TEST //
  /////////////////////////

  // brake_set_damping_constant
  ros::ServiceClient brake_set_client = n.serviceClient<webots_ros::set_float>(model_name + "/my_brake/set_damping_constant");
  webots_ros::set_float brake_set_srv;
  brake_set_srv.request.value = 0.55;
  if (brake_set_client.call(brake_set_srv) && brake_set_srv.response.success)
    ROS_INFO("Brake damping constant set to 0.55.");
  else
    ROS_ERROR("Failed to call service brake_set_damping_constant.");

  brake_set_client.shutdown();
  time_step_client.call(time_step_srv);

  // brake_get_motor_name
  ros::ServiceClient brake_get_motor_name_client =
    n.serviceClient<webots_ros::get_string>(model_name + "/my_brake/get_motor_name");
  webots_ros::get_string brake_get_motor_name_srv;
  if (brake_get_motor_name_client.call(brake_get_motor_name_srv)) {
    ROS_INFO("Linear motor name returned from Brake API %s.", brake_get_motor_name_srv.response.value.c_str());
    if (brake_get_motor_name_srv.response.value.compare("linear_motor") != 0)
      ROS_ERROR("Failed to call service brake_get_motor_name: received '%s' instead of 'linear_motor'",
                brake_get_motor_name_srv.response.value.c_str());
  } else
    ROS_ERROR("Failed to call service brake_get_motor_name.");

  brake_get_motor_name_client.shutdown();
  time_step_client.call(time_step_srv);

  /////////////////////////
  // CAMERA METHODS TEST //
  /////////////////////////

  // camera_set_focal_distance
  double focal_distance = 0.33;
  ros::ServiceClient camera_set_client = n.serviceClient<webots_ros::set_float>(model_name + "/camera/set_focal_distance");
  webots_ros::set_float camera_set_focal_distance_srv;
  camera_set_focal_distance_srv.request.value = focal_distance;
  if (camera_set_client.call(camera_set_focal_distance_srv) && camera_set_focal_distance_srv.response.success)
    ROS_INFO("Camera focal distance set to %f.", focal_distance);
  else
    ROS_ERROR("Failed to call service camera_set_focal_distance.");

  camera_set_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera_set_fov
  double fov = 1.33;
  camera_set_client = n.serviceClient<webots_ros::set_float>(model_name + "/camera/set_fov");
  webots_ros::set_float camera_set_fov_srv;
  camera_set_fov_srv.request.value = fov;
  if (camera_set_client.call(camera_set_fov_srv) && camera_set_fov_srv.response.success)
    ROS_INFO("Camera fov set to %f.", fov);
  else
    ROS_ERROR("Failed to call service camera_set_fov.");

  camera_set_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera enable
  ros::ServiceClient enable_camera_client;
  webots_ros::set_int camera_srv;
  ros::Subscriber sub_camera_color;

  enable_camera_client = n.serviceClient<webots_ros::set_int>(model_name + "/camera/enable");
  camera_srv.request.value = TIME_STEP;
  if (enable_camera_client.call(camera_srv) && camera_srv.response.success) {
    ROS_INFO("Camera enabled.");
    sub_camera_color = n.subscribe(model_name + "/camera/image", 1, cameraCallback);
    ROS_INFO("Topic for camera color initialized.");
    callbackCalled = false;
    while (sub_camera_color.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topic for camera color connected.");
  } else {
    if (camera_srv.response.success == -1)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable camera.");
    return 1;
  }

  sub_camera_color.shutdown();
  enable_camera_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera_get_info
  ros::ServiceClient get_info_client = n.serviceClient<webots_ros::camera_get_info>(model_name + "/camera/get_info");
  webots_ros::camera_get_info get_info_srv;
  if (get_info_client.call(get_info_srv))
    ROS_INFO("Camera of %s has a width of %d, a height of %d, a field of view of %f, a near range of %f.", model_name.c_str(),
             get_info_srv.response.width, get_info_srv.response.height, get_info_srv.response.Fov,
             get_info_srv.response.nearRange);
  else
    ROS_ERROR("Failed to call service camera_get_info.");
  if (get_info_srv.response.Fov != fov)
    ROS_ERROR("Failed to set camera fov.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera_get_focus_info
  get_info_client = n.serviceClient<webots_ros::camera_get_focus_info>(model_name + "/camera/get_focus_info");
  webots_ros::camera_get_focus_info camera_get_focus_info_srv;
  if (get_info_client.call(camera_get_focus_info_srv))
    ROS_INFO("Camera of %s has focalLength %f, focalDistance %f, maxFocalDistance %f, and minFocalDistance %f.",
             model_name.c_str(), camera_get_focus_info_srv.response.focalLength,
             camera_get_focus_info_srv.response.focalDistance, camera_get_focus_info_srv.response.maxFocalDistance,
             camera_get_focus_info_srv.response.minFocalDistance);
  else
    ROS_ERROR("Failed to call service camera_get_focus_info.");
  if (camera_get_focus_info_srv.response.focalDistance != focal_distance)
    ROS_ERROR("Failed to set camera focal distance.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera_get_zoom_info
  get_info_client = n.serviceClient<webots_ros::camera_get_zoom_info>(model_name + "/camera/get_zoom_info");
  webots_ros::camera_get_zoom_info camera_get_zoom_info_srv;
  if (get_info_client.call(camera_get_zoom_info_srv))
    ROS_INFO("Camera of %s has min fov %f, anf max fov %f.", model_name.c_str(), camera_get_zoom_info_srv.response.minFov,
             camera_get_zoom_info_srv.response.maxFov);
  else
    ROS_ERROR("Failed to call service camera_get_zoom_info.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  // check presence of recognition capability
  get_info_client = n.serviceClient<webots_ros::get_bool>(model_name + "/camera/has_recognition");
  webots_ros::get_bool camera_has_recognition_srv;
  if (get_info_client.call(camera_has_recognition_srv))
    if (camera_has_recognition_srv.response.value)
      ROS_INFO("Recognition capability of camera of %s found.", model_name.c_str());
    else
      ROS_ERROR("Recognition capability of camera of %s not found.", model_name.c_str());
  else
    ROS_ERROR("Failed to call service camera_get_zoom_info.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera recognition enable
  ros::ServiceClient enable_camera_recognition_client;
  webots_ros::set_int camera_recognition_srv;
  ros::Subscriber sub_camera_recognition;

  enable_camera_recognition_client = n.serviceClient<webots_ros::set_int>(model_name + "/camera/recognition_enable");
  camera_recognition_srv.request.value = TIME_STEP;
  if (enable_camera_recognition_client.call(camera_recognition_srv) && camera_recognition_srv.response.success) {
    ROS_INFO("Camera recognition enabled.");
    sub_camera_recognition = n.subscribe(model_name + "/camera/recognition_objects", 1, cameraRecognitionCallback);
    ROS_INFO("Topic for camera recognition initialized.");
    callbackCalled = false;
    while (sub_camera_recognition.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topic for camera recognition connected.");
  } else {
    if (camera_recognition_srv.response.success == -1)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable camera recognition.");
    return 1;
  }

  sub_camera_recognition.shutdown();
  enable_camera_recognition_client.shutdown();
  time_step_client.call(time_step_srv);

  // camera_save_image
  ros::ServiceClient save_image_client = n.serviceClient<webots_ros::save_image>(model_name + "/camera/save_image");
  webots_ros::save_image save_image_srv;
  save_image_srv.request.filename = std::string(getenv("HOME")) + std::string("/test_image_camera.png");
  save_image_srv.request.quality = 100;

  if (save_image_client.call(save_image_srv) && save_image_srv.response.success == 1)
    ROS_INFO("Image saved.");
  else
    ROS_ERROR("Failed to call service save_image.");

  save_image_client.shutdown();
  time_step_client.call(time_step_srv);

  ROS_INFO("Camera disabled.");

  /////////////////////////
  // DEVICE METHODS TEST //
  /////////////////////////

  // device_get_name
  ros::ServiceClient device_get_name_client = n.serviceClient<webots_ros::get_string>(model_name + "/camera/get_name");
  webots_ros::get_string device_get_name_srv;
  if (device_get_name_client.call(device_get_name_srv))
    ROS_INFO("Camera device name: %s.", device_get_name_srv.response.value.c_str());
  else
    ROS_ERROR("Failed to call service get_name.");

  device_get_name_client.shutdown();
  time_step_client.call(time_step_srv);

  // device_get_name
  ros::ServiceClient device_get_model_client = n.serviceClient<webots_ros::get_string>(model_name + "/camera/get_model");
  webots_ros::get_string device_get_model_srv;
  if (device_get_model_client.call(device_get_model_srv))
    ROS_INFO("Camera device model: %s.", device_get_model_srv.response.value.c_str());
  else
    ROS_ERROR("Failed to call service get_model.");

  device_get_model_client.shutdown();
  time_step_client.call(time_step_srv);

  ////////////////////////////////
  // ACCELEROMETER METHODS TEST //
  ////////////////////////////////

  ros::ServiceClient set_accelerometer_client;
  webots_ros::set_int accelerometer_srv;
  ros::Subscriber sub_accelerometer_32;
  ros::Subscriber sub_accelerometer_64;

  set_accelerometer_client = n.serviceClient<webots_ros::set_int>(model_name + "/accelerometer/enable");

  accelerometer_srv.request.value = 64;
  if (set_accelerometer_client.call(accelerometer_srv) && accelerometer_srv.response.success) {
    sub_accelerometer_64 = n.subscribe(model_name + "/accelerometer/values", 1, accelerometerCallback);
    callbackCalled = false;
    while (sub_accelerometer_64.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (accelerometer_srv.response.success == -1)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable accelerometer.");
    return 1;
  }

  sub_accelerometer_64.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient sampling_period_accelerometer_client;
  webots_ros::get_int sampling_period_accelerometer_srv;

  sampling_period_accelerometer_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/accelerometer/get_sampling_period");
  sampling_period_accelerometer_client.call(sampling_period_accelerometer_srv);
  ROS_INFO("Accelerometer is enabled with a sampling period of %d.", sampling_period_accelerometer_srv.response.value);

  accelerometer_srv.request.value = 32;
  if (set_accelerometer_client.call(accelerometer_srv) && accelerometer_srv.response.success) {
    sub_accelerometer_32 = n.subscribe(model_name + "/accelerometer/values", 1, accelerometerCallback);
    callbackCalled = false;
    while (sub_accelerometer_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (accelerometer_srv.response.success == -1)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable accelerometer.");
    return 1;
  }

  ros::ServiceClient lookup_table_accelerometer_client;
  webots_ros::get_float_array lookup_table_accelerometer_srv;
  lookup_table_accelerometer_client =
    n.serviceClient<webots_ros::get_float_array>(model_name + "/accelerometer/get_lookup_table");
  if (lookup_table_accelerometer_client.call(lookup_table_accelerometer_srv))
    ROS_INFO("Accelerometer lookup table size = %lu.", lookup_table_accelerometer_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'accelerometer'.");
  if (lookup_table_accelerometer_srv.response.value.size() != 0)
    ROS_ERROR("Size of lookup table of 'accelerometer' is wrong.");
  lookup_table_accelerometer_client.shutdown();

  sub_accelerometer_32.shutdown();
  set_accelerometer_client.shutdown();
  time_step_client.call(time_step_srv);

  sampling_period_accelerometer_client.call(sampling_period_accelerometer_srv);
  ROS_INFO("Accelerometer is enabled with a sampling period of %d.", sampling_period_accelerometer_srv.response.value);

  // wait for webots to detect shutdown of topics and to disable the sensor afterwards
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  sampling_period_accelerometer_client.call(sampling_period_accelerometer_srv);
  ROS_INFO("Accelerometer is disabled (sampling period is %d).", sampling_period_accelerometer_srv.response.value);

  sampling_period_accelerometer_client.shutdown();
  time_step_client.call(time_step_srv);

  /////////////////////////////////
  // BATTERY SENSOR METHODS TEST //
  /////////////////////////////////

  ros::ServiceClient set_battery_sensor_client;
  webots_ros::set_int battery_sensor_srv;
  ros::Subscriber sub_battery_sensor_32;
  set_battery_sensor_client = n.serviceClient<webots_ros::set_int>(model_name + "/battery_sensor/enable");

  ros::ServiceClient sampling_period_battery_sensor_client;
  webots_ros::get_int sampling_period_battery_sensor_srv;
  sampling_period_battery_sensor_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/battery_sensor/get_sampling_period");

  battery_sensor_srv.request.value = 32;
  if (set_battery_sensor_client.call(battery_sensor_srv) && battery_sensor_srv.response.success) {
    ROS_INFO("Battery_sensor enabled.");
    sub_battery_sensor_32 = n.subscribe(model_name + "/battery_sensor/value", 1, battery_sensorCallback);
    callbackCalled = false;
    while (sub_battery_sensor_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!battery_sensor_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable battery_sensor.");
    return 1;
  }

  sub_battery_sensor_32.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_battery_sensor_client.call(sampling_period_battery_sensor_srv);
  ROS_INFO("Battery_sensor is enabled with a sampling period of %d.", sampling_period_battery_sensor_srv.response.value);

  time_step_client.call(time_step_srv);

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  sampling_period_battery_sensor_client.call(sampling_period_battery_sensor_srv);
  ROS_INFO("Battery_sensor is disabled (sampling period is %d).", sampling_period_battery_sensor_srv.response.value);

  set_battery_sensor_client.shutdown();
  sampling_period_battery_sensor_client.shutdown();
  time_step_client.call(time_step_srv);

  //////////////////////////
  // COMPASS METHODS TEST //
  //////////////////////////

  ros::ServiceClient set_compass_client;
  webots_ros::set_int compass_srv;
  ros::Subscriber sub_compass_32;
  set_compass_client = n.serviceClient<webots_ros::set_int>(model_name + "/compass/enable");

  ros::ServiceClient sampling_period_compass_client;
  webots_ros::get_int sampling_period_compass_srv;
  sampling_period_compass_client = n.serviceClient<webots_ros::get_int>(model_name + "/compass/get_sampling_period");

  compass_srv.request.value = 32;
  if (set_compass_client.call(compass_srv) && compass_srv.response.success == 1) {
    ROS_INFO("Compass enabled.");
    sub_compass_32 = n.subscribe(model_name + "/compass/values", 1, compassCallback);
    callbackCalled = false;
    while (sub_compass_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (compass_srv.response.success == -1)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable compass.");
    return 1;
  }

  ros::ServiceClient lookup_table_compass_client;
  webots_ros::get_float_array lookup_table_compass_srv;
  lookup_table_compass_client = n.serviceClient<webots_ros::get_float_array>(model_name + "/compass/get_lookup_table");
  if (lookup_table_compass_client.call(lookup_table_compass_srv))
    ROS_INFO("Compass lookup table size = %lu.", lookup_table_compass_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'compass'.");
  if (lookup_table_compass_srv.response.value.size() != 0)
    ROS_ERROR("Size of lookup table of 'compass' is wrong.");
  lookup_table_compass_client.shutdown();

  sub_compass_32.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_compass_client.call(sampling_period_compass_srv);
  ROS_INFO("Compass is enabled with a sampling period of %d.", sampling_period_compass_srv.response.value);

  time_step_client.call(time_step_srv);

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  sampling_period_compass_client.call(sampling_period_compass_srv);
  ROS_INFO("Compass is disabled (sampling period is %d).", sampling_period_compass_srv.response.value);

  set_compass_client.shutdown();
  sampling_period_compass_client.shutdown();
  time_step_client.call(time_step_srv);

  ////////////////////////////
  // CONNECTOR METHODS TEST //
  ////////////////////////////

  ros::ServiceClient connector_enable_presence_client;
  webots_ros::set_int connector_srv;
  ros::Subscriber sub_connector;
  connector_enable_presence_client = n.serviceClient<webots_ros::set_int>(model_name + "/connector/presence_sensor/enable");

  connector_srv.request.value = 32;
  if (connector_enable_presence_client.call(connector_srv) && connector_srv.response.success) {
    ROS_INFO("Connector's presence sensor enabled.");
    sub_connector = n.subscribe(model_name + "/connector/presence", 1, connectorCallback);
    callbackCalled = false;
    while (sub_connector.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!connector_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable connector's presence sensor.");
    return 1;
  }

  sub_connector.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  connector_srv.request.value = 0;
  if (connector_enable_presence_client.call(connector_srv) && connector_srv.response.success)
    ROS_INFO("Connector's presence sensor disabled.");
  else {
    if (!connector_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to disable connector's presence sensor.");
    return 1;
  }

  ros::ServiceClient connector_lock_client;
  webots_ros::set_bool connector_lock_srv;
  connector_lock_client = n.serviceClient<webots_ros::set_bool>(model_name + "/connector/lock");

  connector_lock_srv.request.value = true;
  if (connector_lock_client.call(connector_lock_srv) && connector_lock_srv.response.success)
    ROS_INFO("Connector has been locked.");
  else
    ROS_INFO("Failed to lock connector.");

  connector_lock_client.shutdown();
  connector_enable_presence_client.shutdown();
  time_step_client.call(time_step_srv);

  //////////////////////////
  // DISPLAY METHODS TEST //
  //////////////////////////

  ros::ServiceClient display_get_info_client;
  webots_ros::display_get_info display_get_info_srv;
  display_get_info_client = n.serviceClient<webots_ros::display_get_info>(model_name + "/display/get_info");

  display_get_info_client.call(display_get_info_srv);
  ROS_INFO("Display's width is %d and its height is %d.", display_get_info_srv.response.width,
           display_get_info_srv.response.height);

  display_get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_set_color_client;
  webots_ros::set_int display_set_color_srv;
  display_set_color_client = n.serviceClient<webots_ros::set_int>(model_name + "/display/set_color");

  display_set_color_srv.request.value = 0xFF0000;
  if (display_set_color_client.call(display_set_color_srv) && display_set_color_srv.response.success)
    ROS_INFO("Display's color has been updated.");
  else
    ROS_ERROR("Failed to call service display_set_color. Success = %d.", display_set_color_srv.response.success);

  display_set_color_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_set_alpha_client;
  webots_ros::set_float display_set_alpha_srv;
  display_set_alpha_client = n.serviceClient<webots_ros::set_float>(model_name + "/display/set_alpha");

  display_set_alpha_srv.request.value = 1.0;
  if (display_set_alpha_client.call(display_set_alpha_srv) && display_set_alpha_srv.response.success)
    ROS_INFO("Display's alpha has been updated.");
  else
    ROS_ERROR("Failed to call service display_set_alpha.");

  display_set_alpha_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_set_opacity_client;
  webots_ros::set_float display_set_opacity_srv;
  display_set_opacity_client = n.serviceClient<webots_ros::set_float>(model_name + "/display/set_opacity");

  display_set_opacity_srv.request.value = 1.0;
  if (display_set_opacity_client.call(display_set_opacity_srv) && display_set_opacity_srv.response.success)
    ROS_INFO("Display's opacity has been updated.");
  else
    ROS_ERROR("Failed to call service display_set_opacity.");

  display_set_opacity_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_set_font_client;
  webots_ros::display_set_font display_set_font_srv;
  display_set_font_client = n.serviceClient<webots_ros::display_set_font>(model_name + "/display/set_font");

  display_set_font_srv.request.font = "Arial";
  display_set_font_srv.request.size = 8;
  display_set_font_srv.request.antiAliasing = 0;
  if (display_set_font_client.call(display_set_font_srv) && display_set_font_srv.response.success == 1)
    ROS_INFO("Display's font has been updated.");
  else
    ROS_ERROR("Failed to call service display_set_font. Success = %d.", display_set_font_srv.response.success);

  display_set_font_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_draw_pixel_client;
  webots_ros::display_draw_pixel display_draw_pixel_srv;
  display_draw_pixel_client = n.serviceClient<webots_ros::display_draw_pixel>(model_name + "/display/draw_pixel");

  display_draw_pixel_srv.request.x1 = 10;
  display_draw_pixel_srv.request.y1 = 10;
  if (display_draw_pixel_client.call(display_draw_pixel_srv) && display_draw_pixel_srv.response.success == 1)
    ROS_INFO("Pixel drawn at x =32 and y = 32 on the display.");
  else
    ROS_ERROR("Failed to call service display_draw_pixel. Success = %d.", display_draw_pixel_srv.response.success);

  display_draw_pixel_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_draw_line_client;
  webots_ros::display_draw_line display_draw_line_srv;
  display_draw_line_client = n.serviceClient<webots_ros::display_draw_line>(model_name + "/display/draw_line");

  display_draw_line_srv.request.x1 = 32;
  display_draw_line_srv.request.x2 = 63;
  display_draw_line_srv.request.y1 = 32;
  display_draw_line_srv.request.y2 = 42;
  if (display_draw_line_client.call(display_draw_line_srv) && display_draw_line_srv.response.success == 1)
    ROS_INFO("Line drawn at x =32 and y = 32 on the display.");
  else
    ROS_ERROR("Failed to call service display_draw_line. Success = %d.", display_draw_line_srv.response.success);

  display_draw_line_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_draw_rectangle_client;
  webots_ros::display_draw_rectangle display_draw_rectangle_srv;
  display_draw_rectangle_client = n.serviceClient<webots_ros::display_draw_rectangle>(model_name + "/display/draw_rectangle");

  display_draw_rectangle_srv.request.x = 2;
  display_draw_rectangle_srv.request.y = 32;
  display_draw_rectangle_srv.request.width = 10;
  display_draw_rectangle_srv.request.height = 5;
  if (display_draw_rectangle_client.call(display_draw_rectangle_srv) && display_draw_rectangle_srv.response.success == 1)
    ROS_INFO("Rectangle drawn at x =32 and y = 32 with width = 10 and height = 5 on the display.");
  else
    ROS_ERROR("Failed to call service display_draw_rectangle. Success = %d.", display_draw_rectangle_srv.response.success);

  display_draw_rectangle_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_draw_oval_client;
  webots_ros::display_draw_oval display_draw_oval_srv;
  display_draw_oval_client = n.serviceClient<webots_ros::display_draw_oval>(model_name + "/display/draw_oval");

  display_draw_oval_srv.request.cx = 32;
  display_draw_oval_srv.request.cy = 6;
  display_draw_oval_srv.request.a = 10;
  display_draw_oval_srv.request.b = 5;

  if (display_draw_oval_client.call(display_draw_oval_srv) && display_draw_oval_srv.response.success == 1)
    ROS_INFO("Oval drawn at x =32 and y = 6 and axes a = 10 and b = 5 on the display.");
  else
    ROS_ERROR("Failed to call service display_draw_oval. Success = %d.", display_draw_oval_srv.response.success);

  display_draw_oval_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_draw_polygon_client;
  webots_ros::display_draw_polygon display_draw_polygon_srv;
  display_draw_polygon_client = n.serviceClient<webots_ros::display_draw_polygon>(model_name + "/display/draw_polygon");

  display_draw_polygon_srv.request.x.push_back(55);
  display_draw_polygon_srv.request.y.push_back(55);
  display_draw_polygon_srv.request.x.push_back(50);
  display_draw_polygon_srv.request.y.push_back(50);
  display_draw_polygon_srv.request.x.push_back(45);
  display_draw_polygon_srv.request.y.push_back(45);
  display_draw_polygon_srv.request.x.push_back(45);
  display_draw_polygon_srv.request.y.push_back(55);
  display_draw_polygon_srv.request.x.push_back(40);
  display_draw_polygon_srv.request.y.push_back(50);
  display_draw_polygon_srv.request.size = 5;
  if (display_draw_polygon_client.call(display_draw_polygon_srv) && display_draw_polygon_srv.response.success == 1)
    ROS_INFO("Polygon drawn on the display.");
  else
    ROS_ERROR("Failed to call service display_draw_polygon. Success = %d.", display_draw_polygon_srv.response.success);

  display_draw_polygon_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_draw_text_client;
  webots_ros::display_draw_text display_draw_text_srv;
  display_draw_text_client = n.serviceClient<webots_ros::display_draw_text>(model_name + "/display/draw_text");

  display_draw_text_srv.request.x = 10;
  display_draw_text_srv.request.y = 52;
  display_draw_text_srv.request.text = "hello world";
  if (display_draw_text_client.call(display_draw_text_srv) && display_draw_text_srv.response.success == 1)
    ROS_INFO("Hello World written at x =10 and y = 52 on the display.");
  else
    ROS_ERROR("Failed to call service display_draw_text. Success = %d.", display_draw_text_srv.response.success);

  display_draw_text_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_fill_rectangle_client;
  webots_ros::display_draw_rectangle display_fill_rectangle_srv;
  display_fill_rectangle_client = n.serviceClient<webots_ros::display_draw_rectangle>(model_name + "/display/fill_rectangle");

  display_fill_rectangle_srv.request.x = 2;
  display_fill_rectangle_srv.request.y = 32;
  display_fill_rectangle_srv.request.width = 10;
  display_fill_rectangle_srv.request.height = 5;
  if (display_fill_rectangle_client.call(display_fill_rectangle_srv) && display_fill_rectangle_srv.response.success == 1)
    ROS_INFO("Rectangle filled at x =32 and y = 32 with width = 10 and height = 5 on the display.");
  else
    ROS_ERROR("Failed to call service display_fill_rectangle. Success = %d.", display_fill_rectangle_srv.response.success);

  display_fill_rectangle_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_fill_oval_client;
  webots_ros::display_draw_oval display_fill_oval_srv;
  display_fill_oval_client = n.serviceClient<webots_ros::display_draw_oval>(model_name + "/display/fill_oval");

  display_fill_oval_srv.request.cx = 32;
  display_fill_oval_srv.request.cy = 6;
  display_fill_oval_srv.request.a = 10;
  display_fill_oval_srv.request.b = 5;

  if (display_fill_oval_client.call(display_fill_oval_srv) && display_fill_oval_srv.response.success == 1)
    ROS_INFO("Oval filled at x =32 and y = 6 and axes a = 10 and b = 5 on the display.");
  else
    ROS_ERROR("Failed to call service display_fill_oval. Success = %d.", display_fill_oval_srv.response.success);

  display_fill_oval_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_fill_polygon_client;
  webots_ros::display_draw_polygon display_fill_polygon_srv;
  display_fill_polygon_client = n.serviceClient<webots_ros::display_draw_polygon>(model_name + "/display/fill_polygon");

  display_fill_polygon_srv.request.x.push_back(55);
  display_fill_polygon_srv.request.y.push_back(55);
  display_fill_polygon_srv.request.x.push_back(50);
  display_fill_polygon_srv.request.y.push_back(50);
  display_fill_polygon_srv.request.x.push_back(45);
  display_fill_polygon_srv.request.y.push_back(45);
  display_fill_polygon_srv.request.x.push_back(45);
  display_fill_polygon_srv.request.y.push_back(55);
  display_fill_polygon_srv.request.x.push_back(40);
  display_fill_polygon_srv.request.y.push_back(50);
  display_fill_polygon_srv.request.size = 5;
  if (display_fill_polygon_client.call(display_fill_polygon_srv) && display_fill_polygon_srv.response.success == 1)
    ROS_INFO("Polygon filled on the display.");
  else
    ROS_ERROR("Failed to call service display_fill_polygon. Success = %d.", display_fill_polygon_srv.response.success);

  display_fill_polygon_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_new_client;
  webots_ros::display_image_new display_image_new_srv;
  display_image_new_client = n.serviceClient<webots_ros::display_image_new>(model_name + "/display/image_new");

  display_image_new_srv.request.format = 3;
  display_image_new_srv.request.width = 10;
  display_image_new_srv.request.height = 5;
  display_image_new_srv.request.data.push_back(1);
  display_image_new_srv.request.data.push_back(2);
  display_image_new_srv.request.data.push_back(5);
  display_image_new_srv.request.data.push_back(3);
  display_image_new_srv.request.data.push_back(4);
  display_image_new_client.call(display_image_new_srv);
  ROS_INFO("New image created.");
  uint64_t new_image = display_image_new_srv.response.ir;

  display_image_new_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_copy_client;
  webots_ros::display_image_copy display_image_copy_srv;
  display_image_copy_client = n.serviceClient<webots_ros::display_image_copy>(model_name + "/display/image_copy");

  display_image_copy_srv.request.x = 0;
  display_image_copy_srv.request.y = 32;
  display_image_copy_srv.request.width = 64;
  display_image_copy_srv.request.height = 32;
  display_image_copy_client.call(display_image_copy_srv);
  ROS_INFO("Image copied.");
  uint64_t copy_image = display_image_copy_srv.response.ir;

  display_image_copy_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_paste_client;
  webots_ros::display_image_paste display_image_paste_srv;
  display_image_paste_client = n.serviceClient<webots_ros::display_image_paste>(model_name + "/display/image_paste");

  display_image_paste_srv.request.ir = copy_image;
  display_image_paste_srv.request.x = 0;
  display_image_paste_srv.request.y = 0;
  display_image_paste_srv.request.blend = 1;
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully copy/paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

  display_image_paste_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_save_client;
  webots_ros::display_image_save display_image_save_srv;
  display_image_save_client = n.serviceClient<webots_ros::display_image_save>(model_name + "/display/image_save");

  display_image_save_srv.request.ir = copy_image;
  display_image_save_srv.request.filename = std::string(getenv("HOME")) + std::string("/copy_image.png");
  if (display_image_save_client.call(display_image_save_srv) && display_image_save_srv.response.success == 1)
    ROS_INFO("Image successfully saved.");
  else
    ROS_ERROR("Failed to call service display_image_save to save image.");

  display_image_save_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_load_client;
  webots_ros::display_image_load display_image_load_srv;
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/display/image_load");

  display_image_load_srv.request.filename = std::string(getenv("HOME")) + std::string("/test_image_camera.png");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");
  uint64_t loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

  display_image_load_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_delete_client;
  webots_ros::display_image_delete display_image_delete_srv;
  display_image_delete_client = n.serviceClient<webots_ros::display_image_delete>(model_name + "/display/image_delete");

  display_image_delete_srv.request.ir = loaded_image;
  if (display_image_delete_client.call(display_image_delete_srv) && display_image_delete_srv.response.success == 1)
    ROS_INFO("Loaded image has been deleted.");
  else
    ROS_ERROR("Failed to call service display_image_delete.");

  display_image_delete_srv.request.ir = copy_image;
  if (display_image_delete_client.call(display_image_delete_srv) && display_image_delete_srv.response.success == 1)
    ROS_INFO("Copy image has been deleted.");
  else
    ROS_ERROR("Failed to call service display_image_delete.");

  display_image_delete_srv.request.ir = new_image;
  if (display_image_delete_client.call(display_image_delete_srv) && display_image_delete_srv.response.success == 1)
    ROS_INFO("New image has been deleted.");
  else
    ROS_ERROR("Failed to call service display_image_delete.");

  display_image_delete_client.shutdown();
  time_step_client.call(time_step_srv);

  //////////////////////////////////
  // DISTANCE SENSOR METHODS TEST //
  //////////////////////////////////

  ros::ServiceClient set_distance_sensor_client;
  webots_ros::set_int distance_sensor_srv;
  ros::Subscriber sub_distance_sensor_32;
  set_distance_sensor_client = n.serviceClient<webots_ros::set_int>(model_name + "/distance_sensor/enable");

  ros::ServiceClient sampling_period_distance_sensor_client;
  webots_ros::get_int sampling_period_distance_sensor_srv;
  sampling_period_distance_sensor_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/distance_sensor/get_sampling_period");

  ros::ServiceClient min_value_distance_sensor_client;
  webots_ros::get_float min_value_distance_sensor_srv;
  min_value_distance_sensor_client = n.serviceClient<webots_ros::get_float>(model_name + "/distance_sensor/get_min_value");
  if (min_value_distance_sensor_client.call(min_value_distance_sensor_srv))
    ROS_INFO("Distance_sensor min value = %g.", min_value_distance_sensor_srv.response.value);
  else
    ROS_ERROR("Failed to get the minimum value of 'distance_sensor'.");
  min_value_distance_sensor_client.shutdown();

  ros::ServiceClient max_value_distance_sensor_client;
  webots_ros::get_float max_value_distance_sensor_srv;
  max_value_distance_sensor_client = n.serviceClient<webots_ros::get_float>(model_name + "/distance_sensor/get_max_value");
  if (max_value_distance_sensor_client.call(max_value_distance_sensor_srv))
    ROS_INFO("Distance_sensor max value = %g.", max_value_distance_sensor_srv.response.value);
  else
    ROS_ERROR("Failed to get the maximum value of 'distance_sensor'.");
  max_value_distance_sensor_client.shutdown();

  ros::ServiceClient aperture_distance_sensor_client;
  webots_ros::get_float aperture_distance_sensor_srv;
  aperture_distance_sensor_client = n.serviceClient<webots_ros::get_float>(model_name + "/distance_sensor/get_aperture");
  if (aperture_distance_sensor_client.call(aperture_distance_sensor_srv))
    ROS_INFO("Distance_sensor aperture = %g.", aperture_distance_sensor_srv.response.value);
  else
    ROS_ERROR("Failed to get the aperture of 'distance_sensor'.");
  aperture_distance_sensor_client.shutdown();

  ros::ServiceClient lookup_table_distance_sensor_client;
  webots_ros::get_float_array lookup_table_distance_sensor_srv;
  lookup_table_distance_sensor_client =
    n.serviceClient<webots_ros::get_float_array>(model_name + "/distance_sensor/get_lookup_table");
  if (lookup_table_distance_sensor_client.call(lookup_table_distance_sensor_srv))
    ROS_INFO("Distance_sensor lookup table size = %lu.", lookup_table_distance_sensor_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'distance_sensor'.");
  if (lookup_table_distance_sensor_srv.response.value.size() != 6)
    ROS_ERROR("Size of lookup table of 'distance_sensor' is wrong, expected 0 got %lu.",
              lookup_table_distance_sensor_srv.response.value.size());
  lookup_table_distance_sensor_client.shutdown();

  distance_sensor_srv.request.value = 32;
  if (set_distance_sensor_client.call(distance_sensor_srv) && distance_sensor_srv.response.success) {
    ROS_INFO("Distance_sensor enabled.");
    sub_distance_sensor_32 = n.subscribe(model_name + "/distance_sensor/value", 1, distance_sensorCallback);
    callbackCalled = false;
    while (sub_distance_sensor_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!distance_sensor_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable distance_sensor.");
    return 1;
  }

  sub_distance_sensor_32.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_distance_sensor_client.call(sampling_period_distance_sensor_srv);
  ROS_INFO("Distance_sensor is enabled with a sampling period of %d.", sampling_period_distance_sensor_srv.response.value);

  time_step_client.call(time_step_srv);

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  sampling_period_distance_sensor_client.call(sampling_period_distance_sensor_srv);
  ROS_INFO("Distance_sensor is disabled (sampling period is %d).", sampling_period_distance_sensor_srv.response.value);

  set_distance_sensor_client.shutdown();
  sampling_period_distance_sensor_client.shutdown();
  time_step_client.call(time_step_srv);

  //////////////////////////
  // EMITTER METHODS TEST //
  //////////////////////////

  ros::ServiceClient emitter_send_client;
  webots_ros::set_string emitter_send_srv;
  emitter_send_client = n.serviceClient<webots_ros::set_string>(model_name + "/emitter/send");

  emitter_send_srv.request.value = "abc";

  if (emitter_send_client.call(emitter_send_srv) && emitter_send_srv.response.success)
    ROS_INFO("Emitter has sent data.");
  else
    ROS_ERROR("Failed to call service emitter_send to send data.");

  emitter_send_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient emitter_get_buffer_size_client;
  webots_ros::get_int emitter_get_buffer_size_srv;
  emitter_get_buffer_size_client = n.serviceClient<webots_ros::get_int>(model_name + "/emitter/get_buffer_size");

  emitter_get_buffer_size_client.call(emitter_get_buffer_size_srv);
  ROS_INFO("Emitter's buffer is of size %d.", emitter_get_buffer_size_srv.response.value);

  emitter_get_buffer_size_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient emitter_set_channel_client;
  webots_ros::set_int emitter_set_channel_srv;
  emitter_set_channel_client = n.serviceClient<webots_ros::set_int>(model_name + "/emitter/set_channel");

  emitter_set_channel_srv.request.value = 1;

  if (emitter_set_channel_client.call(emitter_set_channel_srv) && emitter_set_channel_srv.response.success)
    ROS_INFO("Emitter has update the channel.");
  else
    ROS_ERROR("Failed to call service emitter_set_channel to update channel.");

  emitter_set_channel_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient emitter_set_range_client;
  webots_ros::set_float emitter_set_range_srv;
  emitter_set_range_client = n.serviceClient<webots_ros::set_float>(model_name + "/emitter/set_range");

  emitter_set_range_srv.request.value = 50;

  if (emitter_set_range_client.call(emitter_set_range_srv) && emitter_set_range_srv.response.success)
    ROS_INFO("Emitter has update the range.");
  else
    ROS_ERROR("Failed to call service emitter_set_range to update range.");

  emitter_set_range_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient emitter_get_channel_client;
  webots_ros::get_int emitter_get_channel_srv;
  emitter_get_channel_client = n.serviceClient<webots_ros::get_int>(model_name + "/emitter/get_channel");

  emitter_get_channel_client.call(emitter_get_channel_srv);
  ROS_INFO("Emitter uses channel %d.", emitter_get_channel_srv.response.value);

  emitter_get_channel_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient emitter_get_range_client;
  webots_ros::get_float emitter_get_range_srv;
  emitter_get_range_client = n.serviceClient<webots_ros::get_float>(model_name + "/emitter/get_range");

  emitter_get_range_client.call(emitter_get_range_srv);
  ROS_INFO("Emitter has a range of %f.", emitter_get_range_srv.response.value);

  emitter_get_range_client.shutdown();
  time_step_client.call(time_step_srv);

  //////////////////////
  // GPS METHODS TEST //
  //////////////////////

  ros::ServiceClient set_GPS_client;
  webots_ros::set_int GPS_srv;
  ros::Subscriber sub_GPS_32;
  ros::Subscriber sub_GPS_speed;
  set_GPS_client = n.serviceClient<webots_ros::set_int>(model_name + "/gps/enable");

  ros::ServiceClient sampling_period_GPS_client;
  webots_ros::get_int sampling_period_GPS_srv;
  sampling_period_GPS_client = n.serviceClient<webots_ros::get_int>(model_name + "/gps/get_sampling_period");

  GPS_srv.request.value = 32;
  if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
    ROS_INFO("GPS enabled.");
    sub_GPS_32 = n.subscribe(model_name + "/gps/values", 1, GPSCallback);
    callbackCalled = false;
    while (sub_GPS_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    sub_GPS_32.shutdown();
    time_step_client.call(time_step_srv);

    sub_GPS_speed = n.subscribe(model_name + "/gps/speed", 1, GPSSpeedCallback);
    callbackCalled = false;
    while (sub_GPS_speed.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!GPS_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable GPS.");
    return 1;
  }

  sub_GPS_speed.shutdown();
  time_step_client.call(time_step_srv);

  sampling_period_GPS_client.call(sampling_period_GPS_srv);
  ROS_INFO("GPS is enabled with a sampling period of %d.", sampling_period_GPS_srv.response.value);

  ros::ServiceClient gps_get_coordinate_system_client;
  webots_ros::get_int gps_get_coordinate_system_srv;
  gps_get_coordinate_system_client = n.serviceClient<webots_ros::get_int>(model_name + "/gps/get_coordinate_system");

  gps_get_coordinate_system_client.call(gps_get_coordinate_system_srv);
  ROS_INFO("GPS coordinate system type is: %d.", gps_get_coordinate_system_srv.response.value);

  time_step_client.call(time_step_srv);

  sampling_period_GPS_client.call(sampling_period_GPS_srv);
  ROS_INFO("GPS is disabled (sampling period is %d).", sampling_period_GPS_srv.response.value);

  set_GPS_client.shutdown();
  sampling_period_GPS_client.shutdown();
  gps_get_coordinate_system_client.shutdown();
  time_step_client.call(time_step_srv);

  ///////////////////////
  // GYRO METHODS TEST //
  ///////////////////////

  ros::ServiceClient set_gyro_client;
  webots_ros::set_int gyro_srv;
  ros::Subscriber sub_gyro_32;
  set_gyro_client = n.serviceClient<webots_ros::set_int>(model_name + "/gyro/enable");

  ros::ServiceClient sampling_period_gyro_client;
  webots_ros::get_int sampling_period_gyro_srv;
  sampling_period_gyro_client = n.serviceClient<webots_ros::get_int>(model_name + "/gyro/get_sampling_period");

  gyro_srv.request.value = 32;
  if (set_gyro_client.call(gyro_srv) && gyro_srv.response.success) {
    ROS_INFO("Gyro enabled.");
    sub_gyro_32 = n.subscribe(model_name + "/gyro/values", 1, gyroCallback);
    callbackCalled = false;
    while (sub_gyro_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!gyro_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable gyro.");
    return 1;
  }
  sub_gyro_32.shutdown();

  ros::ServiceClient lookup_table_gyro_client;
  webots_ros::get_float_array lookup_table_gyro_srv;
  lookup_table_gyro_client = n.serviceClient<webots_ros::get_float_array>(model_name + "/gyro/get_lookup_table");
  if (lookup_table_gyro_client.call(lookup_table_gyro_srv))
    ROS_INFO("Gyro lookup table size = %lu.", lookup_table_gyro_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'gyro'.");
  if (lookup_table_gyro_srv.response.value.size() != 0)
    ROS_ERROR("Size of lookup table of 'gyro' is wrong.");
  lookup_table_gyro_client.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_gyro_client.call(sampling_period_gyro_srv);
  ROS_INFO("Gyro is enabled with a sampling period of %d.", sampling_period_gyro_srv.response.value);

  time_step_client.call(time_step_srv);

  sampling_period_gyro_client.call(sampling_period_gyro_srv);
  ROS_INFO("Gyro is disabled (sampling period is %d).", sampling_period_gyro_srv.response.value);

  set_gyro_client.shutdown();
  sampling_period_gyro_client.shutdown();
  time_step_client.call(time_step_srv);

  ////////////////////////////////
  // INERTIAL_UNIT METHODS TEST //
  ////////////////////////////////

  ros::ServiceClient set_inertial_unit_client;
  webots_ros::set_int inertial_unit_srv;
  ros::Subscriber sub_inertial_unit_32;
  set_inertial_unit_client = n.serviceClient<webots_ros::set_int>(model_name + "/inertial_unit/enable");

  ros::ServiceClient sampling_period_inertial_unit_client;
  webots_ros::get_int sampling_period_inertial_unit_srv;
  sampling_period_inertial_unit_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/inertial_unit/get_sampling_period");

  inertial_unit_srv.request.value = 32;
  if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
    ROS_INFO("Inertial_unit enabled.");
    sub_inertial_unit_32 = n.subscribe(model_name + "/inertial_unit/roll_pitch_yaw", 1, inertialUnitCallback);
    callbackCalled = false;
    while (sub_inertial_unit_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!inertial_unit_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable inertial_unit.");
    return 1;
  }

  sub_inertial_unit_32.shutdown();

  ros::ServiceClient lookup_table_inertial_unit_client;
  webots_ros::get_float_array lookup_table_inertial_unit_srv;
  lookup_table_inertial_unit_client =
    n.serviceClient<webots_ros::get_float_array>(model_name + "/inertial_unit/get_lookup_table");
  if (lookup_table_inertial_unit_client.call(lookup_table_inertial_unit_srv))
    ROS_INFO("Inertial unit lookup table size = %lu.", lookup_table_inertial_unit_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'inertial_unit'.");
  if (lookup_table_inertial_unit_srv.response.value.size() != 0)
    ROS_ERROR("Size of lookup table of 'inertial_unit' is wrong.");
  lookup_table_inertial_unit_client.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_inertial_unit_client.call(sampling_period_inertial_unit_srv);
  ROS_INFO("Inertial_unit is enabled with a sampling period of %d.", sampling_period_inertial_unit_srv.response.value);

  time_step_client.call(time_step_srv);

  sampling_period_inertial_unit_client.call(sampling_period_inertial_unit_srv);
  ROS_INFO("Inertial_unit is disabled (sampling period is %d).", sampling_period_inertial_unit_srv.response.value);

  set_inertial_unit_client.shutdown();
  sampling_period_inertial_unit_client.shutdown();
  time_step_client.call(time_step_srv);

  ///////////////////////////
  // JOYSTICK METHODS TEST //
  ///////////////////////////

  ros::ServiceClient enable_joystick_client = n.serviceClient<webots_ros::set_int>(model_name + "/joystick/enable");
  webots_ros::set_int enable_joystick_srv;
  ros::Subscriber sub_joystick;

  enable_joystick_srv.request.value = 32;
  if (enable_joystick_client.call(enable_joystick_srv) && enable_joystick_srv.response.success) {
    ROS_INFO("Joystick of %s has been enabled.", model_name.c_str());
    sub_joystick = n.subscribe(model_name + "/joystick/pressed_button", 1, joystickCallback);
    callbackCalled = false;
    ROS_INFO("Topics for joystick initialized.");

    while (sub_joystick.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topics for joystick connected.");
  } else
    ROS_ERROR("Failed to enable joystick.");

  sub_joystick.shutdown();

  enable_joystick_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient joystick_get_model_client;
  webots_ros::get_string joystick_get_model_srv;
  joystick_get_model_client = n.serviceClient<webots_ros::get_string>(model_name + "/joystick/get_model");
  joystick_get_model_client.call(joystick_get_model_srv);
  ROS_INFO("Got josytick model: %s.", joystick_get_model_srv.response.value.c_str());

  joystick_get_model_client.shutdown();
  time_step_client.call(time_step_srv);

  //////////////////////
  // LED METHODS TEST //
  //////////////////////

  ros::ServiceClient set_led_client;
  webots_ros::set_int set_led_srv;
  set_led_client = n.serviceClient<webots_ros::set_int>(model_name + "/led/set_led");

  set_led_srv.request.value = 1;
  if (set_led_client.call(set_led_srv) && set_led_srv.response.success)
    ROS_INFO("LED set to 1.");
  else
    ROS_ERROR("Failed to call service set_led.");

  enable_joystick_client.shutdown();
  set_led_client.call(time_step_srv);

  ros::ServiceClient get_led_client;
  webots_ros::get_int get_led_srv;
  get_led_client = n.serviceClient<webots_ros::get_int>(model_name + "/led/get_led");

  get_led_client.call(get_led_srv);
  ROS_INFO("LED value is %d.", get_led_srv.response.value);

  set_led_srv.request.value = 0;
  if (set_led_client.call(set_led_srv) && set_led_srv.response.success)
    ROS_INFO("LED set to 0.");
  else
    ROS_ERROR("Failed to call service set_led.");

  set_led_client.shutdown();
  get_led_client.shutdown();
  time_step_client.call(time_step_srv);

  ///////////////////////////////
  //     LIDAR METHODS TEST    //
  ///////////////////////////////

  // lidar enable
  ros::ServiceClient set_lidar_client;
  webots_ros::set_int lidar_srv;
  ros::Subscriber sub_lidar;

  set_lidar_client = n.serviceClient<webots_ros::set_int>(model_name + "/lidar/enable");
  lidar_srv.request.value = TIME_STEP;
  if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
    ROS_INFO("Lidar enabled.");
    sub_lidar = n.subscribe(model_name + "/lidar/range_image", 1, lidarCallback);
    callbackCalled = false;
    ROS_INFO("Topic for lidar initialized.");

    while (sub_lidar.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topic for lidar color connected.");
  } else {
    if (!lidar_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable lidar.");
    return 1;
  }

  sub_lidar.shutdown();
  set_lidar_client.shutdown();
  time_step_client.call(time_step_srv);

  // lidar_get_info
  get_info_client = n.serviceClient<webots_ros::lidar_get_info>(model_name + "/lidar/get_info");
  webots_ros::lidar_get_info get_lidar_info_srv;
  if (get_info_client.call(get_lidar_info_srv))
    ROS_INFO("Lidar of %s has a horizontal resolution of %d, %d layers, a field of view of %f, a min range of %f and a max "
             "range of %f.",
             model_name.c_str(), get_lidar_info_srv.response.horizontalResolution, get_lidar_info_srv.response.numberOfLayers,
             get_lidar_info_srv.response.fov, get_lidar_info_srv.response.minRange, get_lidar_info_srv.response.maxRange);
  else
    ROS_ERROR("Failed to call service lidar_get_info.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  // lidar_get_frequency_info
  get_info_client = n.serviceClient<webots_ros::lidar_get_frequency_info>(model_name + "/lidar/get_frequency_info");
  webots_ros::lidar_get_frequency_info get_lidar_frequency_info_srv;
  if (get_info_client.call(get_lidar_frequency_info_srv))
    ROS_INFO("Lidar %s current frequency is %f, maximum frequency is %f and minimum frequency is %f.", model_name.c_str(),
             get_lidar_frequency_info_srv.response.frequency, get_lidar_frequency_info_srv.response.maxFrequency,
             get_lidar_frequency_info_srv.response.minFrequency);
  else
    ROS_ERROR("Failed to call service lidar_get_frequency_info.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  ROS_INFO("Lidar disabled.");

  ///////////////////////////////
  // LIGHT SENSOR METHODS TEST //
  ///////////////////////////////

  ros::ServiceClient set_light_sensor_client;
  webots_ros::set_int light_sensor_srv;
  ros::Subscriber sub_light_sensor_32;
  set_light_sensor_client = n.serviceClient<webots_ros::set_int>(model_name + "/light_sensor/enable");

  ros::ServiceClient sampling_period_light_sensor_client;
  webots_ros::get_int sampling_period_light_sensor_srv;
  sampling_period_light_sensor_client = n.serviceClient<webots_ros::get_int>(model_name + "/light_sensor/get_sampling_period");

  light_sensor_srv.request.value = 32;
  if (set_light_sensor_client.call(light_sensor_srv) && light_sensor_srv.response.success) {
    ROS_INFO("Light_sensor enabled.");
    sub_light_sensor_32 = n.subscribe(model_name + "/light_sensor/value", 1, lightSensorCallback);
    callbackCalled = false;
    while (sub_light_sensor_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!light_sensor_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable light_sensor.");
    return 1;
  }

  sub_light_sensor_32.shutdown();

  ros::ServiceClient lookup_table_light_sensor_client;
  webots_ros::get_float_array lookup_table_light_sensor_srv;
  lookup_table_light_sensor_client =
    n.serviceClient<webots_ros::get_float_array>(model_name + "/light_sensor/get_lookup_table");
  if (lookup_table_light_sensor_client.call(lookup_table_light_sensor_srv))
    ROS_INFO("Light sensor lookup table size = %lu.", lookup_table_light_sensor_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'light_sensor'.");
  if (lookup_table_light_sensor_srv.response.value.size() != 6)
    ROS_ERROR("Size of lookup table of 'light_sensor' is wrong.");
  lookup_table_light_sensor_client.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_light_sensor_client.call(sampling_period_light_sensor_srv);
  ROS_INFO("Light_sensor is enabled with a sampling period of %d.", sampling_period_light_sensor_srv.response.value);

  time_step_client.call(time_step_srv);

  sampling_period_light_sensor_client.call(sampling_period_light_sensor_srv);
  ROS_INFO("Light_sensor is disabled (sampling period is %d).", sampling_period_light_sensor_srv.response.value);

  set_light_sensor_client.shutdown();
  sampling_period_light_sensor_client.shutdown();
  time_step_client.call(time_step_srv);

  ////////////////////////
  // MOTOR METHODS TEST //
  ////////////////////////

  ros::ServiceClient motor_get_type_client;
  webots_ros::get_int motor_get_type_srv;
  motor_get_type_client = n.serviceClient<webots_ros::get_int>(model_name + "/rotational_motor/get_type");
  motor_get_type_client.call(motor_get_type_srv);
  ROS_INFO("Rotational_motor is of type %d.", motor_get_type_srv.response.value);

  motor_get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient linear_motor_get_type_client;
  webots_ros::get_int linear_motor_get_type_srv;
  linear_motor_get_type_client = n.serviceClient<webots_ros::get_int>(model_name + "/linear_motor/get_type");
  linear_motor_get_type_client.call(linear_motor_get_type_srv);
  ROS_INFO("Linear_motor is of type %d.", linear_motor_get_type_srv.response.value);

  linear_motor_get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  // motor_get_brake_name
  ros::ServiceClient motor_get_brake_name_client;
  webots_ros::get_string motor_get_brake_name_srv;
  motor_get_brake_name_client = n.serviceClient<webots_ros::get_string>(model_name + "/linear_motor/get_brake_name");
  if (motor_get_brake_name_client.call(motor_get_brake_name_srv)) {
    ROS_INFO("Brake name returned from Motor API: %s.", motor_get_brake_name_srv.response.value.c_str());
    if (motor_get_brake_name_srv.response.value.compare("my_brake") != 0)
      ROS_ERROR("Failed to call service motor_get_brake_name: received '%s' instead of 'my_brake'",
                motor_get_brake_name_srv.response.value.c_str());
  } else
    ROS_ERROR("Failed to call service motor_get_brake_name.");

  motor_get_brake_name_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_acceleration_client;
  webots_ros::set_float set_acceleration_srv;
  set_acceleration_client = n.serviceClient<webots_ros::set_float>(model_name + "/rotational_motor/set_acceleration");

  set_acceleration_srv.request.value = 0.5;
  if (set_acceleration_client.call(set_acceleration_srv) && set_acceleration_srv.response.success)
    ROS_INFO("Acceleration set to 0.5.");
  else
    ROS_ERROR("Failed to call service set_acceleration on motor.");

  set_acceleration_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_velocity_client;
  webots_ros::set_float set_velocity_srv;
  set_velocity_client = n.serviceClient<webots_ros::set_float>(model_name + "/rotational_motor/set_velocity");

  set_velocity_srv.request.value = 0.9;
  if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success)
    ROS_INFO("Velocity set to 0.9.");
  else
    ROS_ERROR("Failed to call service set_velocity on motor.");

  set_velocity_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_force_client;
  webots_ros::set_float set_force_srv;
  set_force_client = n.serviceClient<webots_ros::set_float>(model_name + "/linear_motor/set_force");

  set_force_srv.request.value = 0.2;
  if (set_force_client.call(set_force_srv) && set_force_srv.response.success)
    ROS_INFO("Force set to 0.2.");
  else
    ROS_ERROR("Failed to call service set_force on motor.");

  set_force_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_torque_client;
  webots_ros::set_float set_torque_srv;
  set_torque_client = n.serviceClient<webots_ros::set_float>(model_name + "/rotational_motor/set_torque");

  set_torque_srv.request.value = 0.5;
  if (set_torque_client.call(set_torque_srv) && set_torque_srv.response.success)
    ROS_INFO("Torque set to 0.5.");
  else
    ROS_ERROR("Failed to call service set_torque on motor.");

  set_torque_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_available_force_client;
  webots_ros::set_float set_available_force_srv;
  set_available_force_client = n.serviceClient<webots_ros::set_float>(model_name + "/linear_motor/set_available_force");

  set_available_force_srv.request.value = 0.8;
  if (set_available_force_client.call(set_available_force_srv) && set_available_force_srv.response.success)
    ROS_INFO("Available_force set to 0.8.");
  else
    ROS_ERROR("Failed to call service set_available_force on motor.");

  set_available_force_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_available_torque_client;
  webots_ros::set_float set_available_torque_srv;
  set_available_torque_client = n.serviceClient<webots_ros::set_float>(model_name + "/rotational_motor/set_available_torque");

  set_available_torque_srv.request.value = 0.8;
  if (set_available_torque_client.call(set_available_torque_srv) && set_available_torque_srv.response.success)
    ROS_INFO("Available_torque set to 0.8.");
  else
    ROS_ERROR("Failed to call service set_available_torque on motor.");

  set_available_torque_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_control_pid_client;
  webots_ros::motor_set_control_pid set_control_pid_srv;
  set_control_pid_client = n.serviceClient<webots_ros::motor_set_control_pid>(model_name + "/rotational_motor/set_control_pid");

  set_control_pid_srv.request.controlp = 1;
  if (set_control_pid_client.call(set_control_pid_srv) && set_control_pid_srv.response.success == 1)
    ROS_INFO("Control p set to 1.");
  else
    ROS_ERROR("Failed to call service set_controlp on motor.");

  set_control_pid_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_linear_control_pid_client;
  webots_ros::motor_set_control_pid set_linear_control_pid_srv;
  set_linear_control_pid_client =
    n.serviceClient<webots_ros::motor_set_control_pid>(model_name + "/linear_motor/set_control_pid");

  set_linear_control_pid_srv.request.controlp = 1;
  if (set_linear_control_pid_client.call(set_linear_control_pid_srv) && set_linear_control_pid_srv.response.success == 1)
    ROS_INFO("Control p set to 1 for linear_motor.");
  else
    ROS_ERROR("Failed to call service set_controlp on linear_motor.");

  set_linear_control_pid_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_position_client;
  webots_ros::set_float set_position_srv;
  set_position_client = n.serviceClient<webots_ros::set_float>(model_name + "/rotational_motor/set_position");

  set_position_srv.request.value = 1.5;
  if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
    ROS_INFO("Position set to 1.5.");
  else
    ROS_ERROR("Failed to call service set_position on motor.");

  set_position_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_linear_position_client;
  webots_ros::set_float set_linear_position_srv;
  set_linear_position_client = n.serviceClient<webots_ros::set_float>(model_name + "/linear_motor/set_position");

  set_linear_position_srv.request.value = 0.3;
  if (set_linear_position_client.call(set_linear_position_srv) && set_linear_position_srv.response.success)
    ROS_INFO("Position set to 0.3 for linear_motor.");
  else
    ROS_ERROR("Failed to call service set_position on linear_motor.");

  set_linear_position_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_target_position_client;
  webots_ros::get_float get_target_position_srv;
  get_target_position_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_target_position");

  get_target_position_client.call(get_target_position_srv);
  ROS_INFO("Target position for rotational_motor is %f.", get_target_position_srv.response.value);

  get_target_position_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_min_position_client;
  webots_ros::get_float get_min_position_srv;
  get_min_position_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_min_position");

  get_min_position_client.call(get_min_position_srv);
  ROS_INFO("Min position for rotational_motor is %f.", get_min_position_srv.response.value);

  get_min_position_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_position_client;
  webots_ros::get_float get_max_position_srv;
  get_max_position_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_max_position");

  get_max_position_client.call(get_max_position_srv);
  ROS_INFO("Max position for rotational_motor is %f.", get_max_position_srv.response.value);

  get_max_position_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_velocity_client;
  webots_ros::get_float get_velocity_srv;
  get_velocity_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_velocity");

  get_velocity_client.call(get_velocity_srv);
  ROS_INFO("Velocity for rotational_motor is %f.", get_velocity_srv.response.value);

  get_velocity_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_velocity_client;
  webots_ros::get_float get_max_velocity_srv;
  get_max_velocity_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_max_velocity");

  get_max_velocity_client.call(get_max_velocity_srv);
  ROS_INFO("Max velocity for rotational_motor is %f.", get_max_velocity_srv.response.value);

  get_max_velocity_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_acceleration_client;
  webots_ros::get_float get_acceleration_srv;
  get_acceleration_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_acceleration");

  get_acceleration_client.call(get_acceleration_srv);
  ROS_INFO("Acceleration for rotational_motor is %f.", get_acceleration_srv.response.value);

  get_acceleration_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_available_force_client;
  webots_ros::get_float get_available_force_srv;
  get_available_force_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_available_force");

  get_available_force_client.call(get_available_force_srv);
  ROS_INFO("Available force for rotational_motor is %f.", get_available_force_srv.response.value);

  get_available_force_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_force_client;
  webots_ros::get_float get_max_force_srv;
  get_max_force_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_max_force");

  get_max_force_client.call(get_max_force_srv);
  ROS_INFO("Max force for rotational_motor is %f.", get_max_force_srv.response.value);

  get_max_force_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_available_torque_client;
  webots_ros::get_float get_available_torque_srv;
  get_available_torque_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_available_torque");

  get_available_torque_client.call(get_available_torque_srv);
  ROS_INFO("Available torque for rotational_motor is %f.", get_available_torque_srv.response.value);

  get_available_torque_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_torque_client;
  webots_ros::get_float get_max_torque_srv;
  get_max_torque_client = n.serviceClient<webots_ros::get_float>(model_name + "/rotational_motor/get_max_torque");

  get_max_torque_client.call(get_max_torque_srv);
  ROS_INFO("Max torque for rotational_motor is %f.", get_max_torque_srv.response.value);

  get_max_torque_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_motor_feedback_client;
  webots_ros::set_int motor_feedback_srv;
  ros::Subscriber sub_motor_feedback_32;
  set_motor_feedback_client =
    n.serviceClient<webots_ros::set_int>(model_name + "/rotational_motor/torque_feedback_sensor/enable");

  ros::ServiceClient sampling_period_motor_feedback_client;
  webots_ros::get_int sampling_period_motor_feedback_srv;
  sampling_period_motor_feedback_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/rotational_motor/torque_feedback_sensor/get_sampling_period");

  motor_feedback_srv.request.value = 32;
  if (set_motor_feedback_client.call(motor_feedback_srv) && motor_feedback_srv.response.success) {
    ROS_INFO("Motor feedback enabled.");
    sub_motor_feedback_32 = n.subscribe(model_name + "/rotational_motor/torque_feedback", 1, motorSensorCallback);
    callbackCalled = false;
    while (sub_motor_feedback_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!motor_feedback_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable motor_feedback.");
    return 1;
  }

  sub_motor_feedback_32.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_motor_feedback_client.call(sampling_period_motor_feedback_srv);
  ROS_INFO("Motor feedback is enabled with a sampling period of %d.", sampling_period_motor_feedback_srv.response.value);

  time_step_client.call(time_step_srv);

  sampling_period_motor_feedback_client.call(sampling_period_motor_feedback_srv);
  ROS_INFO("Motor feedback is disabled (sampling period is %d).", sampling_period_motor_feedback_srv.response.value);

  set_motor_feedback_client.shutdown();
  sampling_period_motor_feedback_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  //////////////////////
  // PEN METHODS TEST //
  //////////////////////

  ros::ServiceClient set_ink_color_client;
  webots_ros::pen_set_ink_color set_ink_color_srv;
  set_ink_color_client = n.serviceClient<webots_ros::pen_set_ink_color>(model_name + "/pen/set_ink_color");

  set_ink_color_srv.request.color = 0x00FF08;
  if (set_ink_color_client.call(set_ink_color_srv) && set_ink_color_srv.response.success == 1)
    ROS_INFO("Ink color set to turquoise.");
  else
    ROS_ERROR("Failed to call service set_ink_color.");

  set_ink_color_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient pen_write_client;
  webots_ros::set_bool pen_write_srv;
  pen_write_client = n.serviceClient<webots_ros::set_bool>(model_name + "/pen/write");

  pen_write_srv.request.value = true;
  if (pen_write_client.call(pen_write_srv) && pen_write_srv.response.success)
    ROS_INFO("Pen is now writing.");
  else
    ROS_ERROR("Failed to call service pen_write.");

  pen_write_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  //////////////////////////////////
  // POSITION SENSOR METHODS TEST //
  //////////////////////////////////

  ros::ServiceClient set_position_sensor_client;
  webots_ros::set_int position_sensor_srv;
  ros::Subscriber sub_position_sensor_32;
  set_position_sensor_client = n.serviceClient<webots_ros::set_int>(model_name + "/position_sensor/enable");

  ros::ServiceClient sampling_period_position_sensor_client;
  webots_ros::get_int sampling_period_position_sensor_srv;
  sampling_period_position_sensor_client =
    n.serviceClient<webots_ros::get_int>(model_name + "/position_sensor/get_sampling_period");

  position_sensor_srv.request.value = 32;
  if (set_position_sensor_client.call(position_sensor_srv) && position_sensor_srv.response.success) {
    ROS_INFO("Position_sensor enabled.");
    sub_position_sensor_32 = n.subscribe(model_name + "/position_sensor/value", 1, positionSensorCallback);
    callbackCalled = false;
    while (sub_position_sensor_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!position_sensor_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable position_sensor.");
    return 1;
  }

  sub_position_sensor_32.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_position_sensor_client.call(sampling_period_position_sensor_srv);
  ROS_INFO("Position_sensor is enabled with a sampling period of %d.", sampling_period_position_sensor_srv.response.value);

  time_step_client.call(time_step_srv);

  ros::ServiceClient position_sensor_get_type_client;
  webots_ros::get_int position_sensor_get_type_srv;
  position_sensor_get_type_client = n.serviceClient<webots_ros::get_int>(model_name + "/position_sensor/get_type");

  position_sensor_get_type_client.call(position_sensor_get_type_srv);
  ROS_INFO("Position_sensor is of type %d.", position_sensor_get_type_srv.response.value);

  position_sensor_get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  sampling_period_position_sensor_client.call(sampling_period_position_sensor_srv);
  ROS_INFO("Position_sensor is disabled (sampling period is %d).", sampling_period_position_sensor_srv.response.value);

  set_position_sensor_client.shutdown();
  sampling_period_position_sensor_client.shutdown();
  time_step_client.call(time_step_srv);

  ///////////////////////////////
  //     RADAR METHODS TEST    //
  ///////////////////////////////

  // radar enable
  ros::ServiceClient set_radar_client;
  webots_ros::set_int radar_srv;
  ros::Subscriber sub_radar_target;
  ros::Subscriber sub_radar_target_number;

  set_radar_client = n.serviceClient<webots_ros::set_int>(model_name + "/radar/enable");
  radar_srv.request.value = TIME_STEP;
  if (set_radar_client.call(radar_srv) && radar_srv.response.success) {
    ROS_INFO("Radar enabled.");
    sub_radar_target = n.subscribe(model_name + "/radar/targets", 1, radarTargetsCallback);
    sub_radar_target_number = n.subscribe(model_name + "/radar/number_of_targets", 1, radarTargetsNumberCallback);
    callbackCalled = false;
    ROS_INFO("Topics for radar initialized.");

    while (sub_radar_target.getNumPublishers() == 0 && sub_radar_target_number.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topics for radar connected.");
  } else {
    if (!radar_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable radar.");
    return 1;
  }

  sub_radar_target.shutdown();
  sub_radar_target_number.shutdown();
  set_radar_client.shutdown();
  time_step_client.call(time_step_srv);

  // get max and min range
  ros::ServiceClient radar_range_client = n.serviceClient<webots_ros::get_float>(model_name + "/radar/get_max_range");
  webots_ros::get_float radar_get_max_range_srv;
  if (radar_range_client.call(radar_get_max_range_srv)) {
    if (radar_get_max_range_srv.response.value == 2.0)
      ROS_INFO("Received correct radar max range.");
    else
      ROS_ERROR("Received wrong radar max range.");
  } else
    ROS_ERROR("Failed to call service radar_get_max_range.");

  radar_range_client.shutdown();
  time_step_client.call(time_step_srv);

  radar_range_client = n.serviceClient<webots_ros::get_float>(model_name + "/radar/get_min_range");
  webots_ros::get_float radar_get_min_range_srv;
  if (radar_range_client.call(radar_get_min_range_srv)) {
    if (radar_get_min_range_srv.response.value == 1.0)
      ROS_INFO("Received correct radar min range.");
    else
      ROS_ERROR("Received wrong radar min range.");
  } else
    ROS_ERROR("Failed to call service radar_get_min_range.");

  radar_range_client.shutdown();
  time_step_client.call(time_step_srv);

  ROS_INFO("Radar disabled.");

  ///////////////////////////////
  // RANGE-FINDER METHODS TEST //
  ///////////////////////////////

  // range-finder enable
  ros::ServiceClient set_range_finder_client;
  webots_ros::set_int range_finder_srv;
  ros::Subscriber sub_range_finder_color;

  set_range_finder_client = n.serviceClient<webots_ros::set_int>(model_name + "/range_finder/enable");
  range_finder_srv.request.value = TIME_STEP;
  if (set_range_finder_client.call(range_finder_srv) && range_finder_srv.response.success) {
    ROS_INFO("Range-finder enabled.");
    sub_range_finder_color = n.subscribe(model_name + "/range_finder/range_image", 1, rangeFinderCallback);
    callbackCalled = false;
    ROS_INFO("Topic for range-finder initialized.");

    while (sub_range_finder_color.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
    ROS_INFO("Topic for range-finder connected.");
  } else {
    if (!range_finder_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable range-finder.");
    return 1;
  }

  sub_range_finder_color.shutdown();
  set_range_finder_client.shutdown();
  time_step_client.call(time_step_srv);

  // range_finder_get_info
  get_info_client = n.serviceClient<webots_ros::range_finder_get_info>(model_name + "/range_finder/get_info");
  webots_ros::range_finder_get_info get_range_finder_info_srv;
  if (get_info_client.call(get_range_finder_info_srv))
    ROS_INFO(
      "Range-finder of %s has a width of %d, a height of %d, a field of view of %f, a min range of %f and a max range of %f.",
      model_name.c_str(), get_range_finder_info_srv.response.width, get_range_finder_info_srv.response.height,
      get_range_finder_info_srv.response.Fov, get_range_finder_info_srv.response.minRange,
      get_range_finder_info_srv.response.maxRange);
  else
    ROS_ERROR("Failed to call service range_finder_get_info.");

  get_info_client.shutdown();
  time_step_client.call(time_step_srv);

  // range_finder_save_image
  save_image_client = n.serviceClient<webots_ros::save_image>(model_name + "/range_finder/save_image");
  webots_ros::save_image save_range_image_srv;
  save_range_image_srv.request.filename = std::string(getenv("HOME")) + std::string("/test_image_range_finder.png");
  save_range_image_srv.request.quality = 100;

  if (save_image_client.call(save_range_image_srv) && save_range_image_srv.response.success == 1)
    ROS_INFO("Image saved.");
  else
    ROS_ERROR("Failed to call service save_image.");

  save_image_client.shutdown();
  time_step_client.call(time_step_srv);

  ROS_INFO("Range-finder disabled.");

  ///////////////////////////
  // RECEIVER METHODS TEST //
  ///////////////////////////

  ros::ServiceClient set_receiver_client;
  webots_ros::set_int receiver_srv;
  ros::Subscriber sub_receiver_32;
  set_receiver_client = n.serviceClient<webots_ros::set_int>(model_name + "/receiver/enable");

  ros::ServiceClient sampling_period_receiver_client;
  webots_ros::get_int sampling_period_receiver_srv;
  sampling_period_receiver_client = n.serviceClient<webots_ros::get_int>(model_name + "/receiver/get_sampling_period");

  receiver_srv.request.value = 32;
  if (set_receiver_client.call(receiver_srv) && receiver_srv.response.success) {
    ROS_INFO("Receiver enabled.");
    sub_receiver_32 = n.subscribe(model_name + "/receiver/data", 1, receiverCallback);
    callbackCalled = false;
    while (sub_receiver_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!receiver_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable receiver.");
    return 1;
  }

  sub_receiver_32.shutdown();
  set_receiver_client.shutdown();
  time_step_client.call(time_step_srv);

  sampling_period_receiver_client.call(sampling_period_receiver_srv);
  ROS_INFO("Receiver is enabled with a sampling period of %d.", sampling_period_receiver_srv.response.value);

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  // test receiver_get_data_size
  // An error message will probably appear since no data has been sent to the receiver.
  ros::ServiceClient receiver_get_data_size_client;
  webots_ros::get_int receiver_get_data_size_srv;
  receiver_get_data_size_client = n.serviceClient<webots_ros::get_int>(model_name + "/receiver/get_data_size");

  receiver_get_data_size_client.call(receiver_get_data_size_srv);
  if (receiver_get_data_size_srv.response.value != -1)
    ROS_INFO("Emitter's buffer is of size %d.", receiver_get_data_size_srv.response.value);
  else
    ROS_INFO("No message received by emitter, impossible to get buffer size.");

  receiver_get_data_size_client.shutdown();
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient receiver_set_channel_client;
  webots_ros::set_int receiver_set_channel_srv;
  receiver_set_channel_client = n.serviceClient<webots_ros::set_int>(model_name + "/receiver/set_channel");

  receiver_set_channel_srv.request.value = 1;

  if (receiver_set_channel_client.call(receiver_set_channel_srv) && receiver_set_channel_srv.response.success)
    ROS_INFO("Receiver has update the channel.");
  else
    ROS_ERROR("Failed to call service receiver_set_channel to update channel.");

  receiver_set_channel_client.shutdown();
  time_step_client.call(time_step_srv);

  // test receiver_get_channel
  ros::ServiceClient receiver_get_channel_client;
  webots_ros::get_int receiver_get_channel_srv;
  receiver_get_channel_client = n.serviceClient<webots_ros::get_int>(model_name + "/receiver/get_channel");

  receiver_get_channel_client.call(receiver_get_channel_srv);
  ROS_INFO("Receiver uses channel %d.", receiver_get_channel_srv.response.value);

  receiver_get_channel_client.shutdown();
  time_step_client.call(time_step_srv);

  // test receiver_get_queue_length
  ros::ServiceClient receiver_get_queue_length_client;
  webots_ros::get_int receiver_get_queue_length_srv;
  receiver_get_queue_length_client = n.serviceClient<webots_ros::get_int>(model_name + "/receiver/get_queue_length");

  receiver_get_queue_length_client.call(receiver_get_queue_length_srv);
  ROS_INFO("Length of receiver queue is %d.", receiver_get_queue_length_srv.response.value);

  receiver_get_queue_length_client.shutdown();
  time_step_client.call(time_step_srv);

  // test receiver_get_signal_strength
  // An error message will probably appear since no signal has been sent to the receiver.
  ros::ServiceClient receiver_get_signal_strength_client;
  webots_ros::get_float receiver_get_signal_strength_srv;
  receiver_get_signal_strength_client = n.serviceClient<webots_ros::get_float>(model_name + "/receiver/get_signal_strength");

  receiver_get_signal_strength_client.call(receiver_get_signal_strength_srv);
  if (receiver_get_signal_strength_srv.response.value != -1.0)
    ROS_INFO("Strength of the signal is %lf.", receiver_get_signal_strength_srv.response.value);
  else
    ROS_INFO("No message received by emitter, impossible to get signal strength.");

  receiver_get_signal_strength_client.shutdown();
  time_step_client.call(time_step_srv);

  // test receiver_get_emitter_direction
  // An error message will probably appear since no signal has been sent to the receiver
  ros::ServiceClient receiver_get_emitter_direction_client;
  webots_ros::receiver_get_emitter_direction receiver_get_emitter_direction_srv;
  receiver_get_emitter_direction_client =
    n.serviceClient<webots_ros::receiver_get_emitter_direction>(model_name + "/receiver/get_emitter_direction");

  receiver_get_emitter_direction_client.call(receiver_get_emitter_direction_srv);
  if (receiver_get_emitter_direction_srv.response.direction[0] != 0 ||
      receiver_get_emitter_direction_srv.response.direction[1] != 0 ||
      receiver_get_emitter_direction_srv.response.direction[2] != 0)
    ROS_INFO("Signal from emitter comes from direction {%f. %f, %f}.", receiver_get_emitter_direction_srv.response.direction[0],
             receiver_get_emitter_direction_srv.response.direction[1],
             receiver_get_emitter_direction_srv.response.direction[2]);
  else
    ROS_INFO("No message received by emitter, impossible to get signal direction.");

  receiver_get_emitter_direction_client.shutdown();
  time_step_client.call(time_step_srv);

  // test receiver_next_packet
  // An error message will probably appear since there is no packet to read
  ros::ServiceClient receiver_next_packet_client;
  webots_ros::get_bool receiver_next_packet_srv;
  receiver_next_packet_client = n.serviceClient<webots_ros::get_bool>(model_name + "/receiver/next_packet");

  if (receiver_next_packet_client.call(receiver_next_packet_srv) && receiver_next_packet_srv.response.value)
    ROS_INFO("Next packet is ready to be read.");
  else if (!receiver_next_packet_srv.response.value)
    ROS_INFO("No message received by emitter, impossible to get next packet.");
  else
    ROS_ERROR("Failed to call service receiver_next_packet.");

  receiver_next_packet_client.shutdown();
  time_step_client.call(time_step_srv);

  sampling_period_receiver_client.call(sampling_period_receiver_srv);
  ROS_INFO("Receiver is disabled (sampling period is %d).", sampling_period_receiver_srv.response.value);

  sampling_period_receiver_client.shutdown();
  time_step_client.call(time_step_srv);

  ///////////////////////////////
  // TOUCH SENSOR METHODS TEST //
  ///////////////////////////////

  ros::ServiceClient set_touch_sensor_client;
  webots_ros::set_int touch_sensor_srv;
  ros::Subscriber sub_touch_sensor_32;
  set_touch_sensor_client = n.serviceClient<webots_ros::set_int>(model_name + "/touch_sensor/enable");

  ros::ServiceClient sampling_period_touch_sensor_client;
  webots_ros::get_int sampling_period_touch_sensor_srv;
  sampling_period_touch_sensor_client = n.serviceClient<webots_ros::get_int>(model_name + "/touch_sensor/get_sampling_period");

  ros::ServiceClient touch_sensor_get_type_client;
  webots_ros::get_int touch_sensor_get_type_srv;
  touch_sensor_get_type_client = n.serviceClient<webots_ros::get_int>(model_name + "/touch_sensor/get_type");

  touch_sensor_get_type_client.call(touch_sensor_get_type_srv);
  ROS_INFO("Touch_sensor is of type %d.", touch_sensor_get_type_srv.response.value);

  touch_sensor_get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  touch_sensor_srv.request.value = 32;
  if (set_touch_sensor_client.call(touch_sensor_srv) && touch_sensor_srv.response.success) {
    ROS_INFO("Touch_sensor enabled.");
    if (touch_sensor_get_type_srv.response.value == 0)
      sub_touch_sensor_32 = n.subscribe(model_name + "/touch_sensor/value", 1, touchSensorBumperCallback);
    else if (touch_sensor_get_type_srv.response.value == 1)
      sub_touch_sensor_32 = n.subscribe(model_name + "/touch_sensor/value", 1, touchSensorCallback);
    else
      sub_touch_sensor_32 = n.subscribe(model_name + "/touch_sensor/values", 1, touchSensor3DCallback);
    callbackCalled = false;
    while (sub_touch_sensor_32.getNumPublishers() == 0 && !callbackCalled) {
      ros::spinOnce();
      time_step_client.call(time_step_srv);
    }
  } else {
    if (!touch_sensor_srv.response.success)
      ROS_ERROR("Sampling period is not valid.");
    ROS_ERROR("Failed to enable touch_sensor.");
    return 1;
  }

  sub_touch_sensor_32.shutdown();

  ros::ServiceClient lookup_table_touch_sensor_client;
  webots_ros::get_float_array lookup_table_touch_sensor_srv;
  lookup_table_touch_sensor_client =
    n.serviceClient<webots_ros::get_float_array>(model_name + "/touch_sensor/get_lookup_table");
  if (lookup_table_touch_sensor_client.call(lookup_table_touch_sensor_srv))
    ROS_INFO("Touch sensor lookup table size = %lu.", lookup_table_touch_sensor_srv.response.value.size());
  else
    ROS_ERROR("Failed to get the lookup table of 'touch_sensor'.");
  if (lookup_table_touch_sensor_srv.response.value.size() != 6)
    ROS_ERROR("Size of lookup table of 'touch_sensor' is wrong.");
  lookup_table_touch_sensor_client.shutdown();

  time_step_client.call(time_step_srv);

  sampling_period_touch_sensor_client.call(sampling_period_touch_sensor_srv);
  ROS_INFO("Touch_sensor is enabled with a sampling period of %d.", sampling_period_touch_sensor_srv.response.value);

  time_step_client.call(time_step_srv);

  sampling_period_touch_sensor_client.call(sampling_period_touch_sensor_srv);
  ROS_INFO("Touch_sensor is disabled (sampling period is %d).", sampling_period_touch_sensor_srv.response.value);

  set_touch_sensor_client.shutdown();
  sampling_period_touch_sensor_client.shutdown();
  time_step_client.call(time_step_srv);

  /////////////////////////////
  // SUPERVISOR METHODS TEST //
  /////////////////////////////

  ros::ServiceClient supervisor_simulation_reset_physics_client;
  webots_ros::get_bool supervisor_simulation_reset_physics_srv;
  supervisor_simulation_reset_physics_client =
    n.serviceClient<webots_ros::get_bool>(model_name + "/supervisor/simulation_reset_physics");

  if (supervisor_simulation_reset_physics_client.call(supervisor_simulation_reset_physics_srv) &&
      supervisor_simulation_reset_physics_srv.response.value)
    ROS_INFO("Simulation has reset_physics successfully.");
  else
    ROS_ERROR("Failed to call service simulation_reset_physics.");

  supervisor_simulation_reset_physics_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_export_image_client;
  webots_ros::save_image supervisor_export_image_srv;
  supervisor_export_image_client = n.serviceClient<webots_ros::save_image>(model_name + "/supervisor/export_image");

  supervisor_export_image_srv.request.filename = std::string(getenv("HOME")) + std::string("/main_window_test.jpg");
  supervisor_export_image_srv.request.quality = 100;
  if (supervisor_export_image_client.call(supervisor_export_image_srv) && supervisor_export_image_srv.response.success == 1)
    ROS_INFO("Image from main window saved successfully.");
  else
    ROS_ERROR("Failed to call service export_image.");

  supervisor_export_image_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_set_label_client;
  webots_ros::supervisor_set_label supervisor_set_label_srv;
  supervisor_set_label_client = n.serviceClient<webots_ros::supervisor_set_label>(model_name + "/supervisor/set_label");

  supervisor_set_label_srv.request.id = 1;
  supervisor_set_label_srv.request.label = "This is a label";
  supervisor_set_label_srv.request.xpos = 0.02;
  supervisor_set_label_srv.request.ypos = 0.2;
  supervisor_set_label_srv.request.size = 0.1;
  supervisor_set_label_srv.request.color = 0XFF0000;
  supervisor_set_label_srv.request.transparency = 0;
  supervisor_set_label_srv.request.font = "Lucida Console";
  if (supervisor_set_label_client.call(supervisor_set_label_srv) && supervisor_set_label_srv.response.success == 1)
    ROS_INFO("Label set successfully.");
  else
    ROS_ERROR("Failed to call service set_label.");

  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_get_root_client;
  webots_ros::get_uint64 supervisor_get_root_srv;
  supervisor_get_root_client = n.serviceClient<webots_ros::get_uint64>(model_name + "/supervisor/get_root");

  supervisor_get_root_client.call(supervisor_get_root_srv);
  ROS_INFO("Got root node: %lu.", supervisor_get_root_srv.response.value);
  uint64_t root_node = supervisor_get_root_srv.response.value;

  supervisor_get_root_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_get_self_client;
  webots_ros::get_uint64 supervisor_get_self_srv;
  supervisor_get_self_client = n.serviceClient<webots_ros::get_uint64>(model_name + "/supervisor/get_self");

  supervisor_get_self_client.call(supervisor_get_self_srv);
  ROS_INFO("Got self node: %lu.", supervisor_get_self_srv.response.value);
  uint64_t self_node = supervisor_get_self_srv.response.value;

  supervisor_get_self_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_get_from_def_client;
  webots_ros::supervisor_get_from_def supervisor_get_from_def_srv;
  supervisor_get_from_def_client =
    n.serviceClient<webots_ros::supervisor_get_from_def>(model_name + "/supervisor/get_from_def");

  supervisor_get_from_def_srv.request.name = "TEST";
  supervisor_get_from_def_client.call(supervisor_get_from_def_srv);
  uint64_t from_def_node = 0;
  if (supervisor_get_from_def_srv.response.node != 0) {
    ROS_INFO("Got from DEF node: %ld.", supervisor_get_from_def_srv.response.node);
    from_def_node = supervisor_get_from_def_srv.response.node;
  } else
    ROS_ERROR("Could not get node from DEF.");

  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_type_client;
  webots_ros::node_get_type supervisor_node_get_type_srv;
  supervisor_node_get_type_client = n.serviceClient<webots_ros::node_get_type>(model_name + "/supervisor/node/get_type");

  supervisor_node_get_type_srv.request.node = from_def_node;
  supervisor_node_get_type_client.call(supervisor_node_get_type_srv);
  ROS_INFO("Got node type: %d.", supervisor_node_get_type_srv.response.type);

  supervisor_node_get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_type_name_client;
  webots_ros::node_get_name supervisor_node_get_type_name_srv;
  supervisor_node_get_type_name_client =
    n.serviceClient<webots_ros::node_get_name>(model_name + "/supervisor/node/get_type_name");

  supervisor_node_get_type_name_srv.request.node = from_def_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("Got node type name: %s.", supervisor_node_get_type_name_srv.response.name.c_str());

  supervisor_node_get_type_name_srv.request.node = root_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("Got type name of root node: %s.", supervisor_node_get_type_name_srv.response.name.c_str());

  supervisor_node_get_type_name_srv.request.node = self_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("Got type name of self node: %s.", supervisor_node_get_type_name_srv.response.name.c_str());

  supervisor_get_from_def_srv.request.name = "GROUND";
  supervisor_get_from_def_client.call(supervisor_get_from_def_srv);
  uint64_t ground_node = 0;
  if (supervisor_get_from_def_srv.response.node != 0) {
    ROS_INFO("Got from DEF GROUND node: %ld.", supervisor_get_from_def_srv.response.node);
    ground_node = supervisor_get_from_def_srv.response.node;
  } else
    ROS_ERROR("Could not get node from DEF GROUND.");

  supervisor_node_get_type_name_srv.request.node = ground_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("Got type name of GROUND node: %s.", supervisor_node_get_type_name_srv.response.name.c_str());

  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_base_type_name_client;
  webots_ros::node_get_name supervisor_node_get_base_type_name_srv;
  supervisor_node_get_base_type_name_client =
    n.serviceClient<webots_ros::node_get_name>(model_name + "/supervisor/node/get_base_type_name");
  supervisor_node_get_base_type_name_srv.request.node = ground_node;
  supervisor_node_get_base_type_name_client.call(supervisor_node_get_base_type_name_srv);
  ROS_INFO("Got base type name of GROUND node: %s.", supervisor_node_get_base_type_name_srv.response.name.c_str());

  supervisor_node_get_base_type_name_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_def_client;
  webots_ros::node_get_name supervisor_node_get_def_srv;
  supervisor_node_get_def_client = n.serviceClient<webots_ros::node_get_name>(model_name + "/supervisor/node/get_def");
  supervisor_node_get_def_srv.request.node = ground_node;
  supervisor_node_get_def_client.call(supervisor_node_get_def_srv);
  ROS_INFO("Got DEF name of GROUND node: %s.", supervisor_node_get_def_srv.response.name.c_str());

  supervisor_node_get_def_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_position_client;
  webots_ros::node_get_position supervisor_node_get_position_srv;
  supervisor_node_get_position_client =
    n.serviceClient<webots_ros::node_get_position>(model_name + "/supervisor/node/get_position");

  supervisor_node_get_position_srv.request.node = from_def_node;
  supervisor_node_get_position_client.call(supervisor_node_get_position_srv);
  ROS_INFO("From_def node got position: x = %f y = %f z = %f.", supervisor_node_get_position_srv.response.position.x,
           supervisor_node_get_position_srv.response.position.y, supervisor_node_get_position_srv.response.position.z);

  supervisor_node_get_position_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_orientation_client;
  webots_ros::node_get_orientation supervisor_node_get_orientation_srv;
  supervisor_node_get_orientation_client =
    n.serviceClient<webots_ros::node_get_orientation>(model_name + "/supervisor/node/get_orientation");

  supervisor_node_get_orientation_srv.request.node = from_def_node;
  supervisor_node_get_orientation_client.call(supervisor_node_get_orientation_srv);
  ROS_INFO(
    "From_def orientation quaternion is:\nw=%f x=%f y=%f z=%f.", supervisor_node_get_orientation_srv.response.orientation.w,
    supervisor_node_get_orientation_srv.response.orientation.x, supervisor_node_get_orientation_srv.response.orientation.y,
    supervisor_node_get_orientation_srv.response.orientation.z);

  supervisor_node_get_orientation_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_center_of_mass_client;
  webots_ros::node_get_center_of_mass supervisor_node_get_center_of_mass_srv;
  supervisor_node_get_center_of_mass_client =
    n.serviceClient<webots_ros::node_get_center_of_mass>(model_name + "/supervisor/node/get_center_of_mass");

  supervisor_node_get_center_of_mass_srv.request.node = from_def_node;
  supervisor_node_get_center_of_mass_client.call(supervisor_node_get_center_of_mass_srv);
  ROS_INFO("From_def node's center of mass coordinates are: x = %f y = %f z = %f.",
           supervisor_node_get_center_of_mass_srv.response.centerOfMass.x,
           supervisor_node_get_center_of_mass_srv.response.centerOfMass.y,
           supervisor_node_get_center_of_mass_srv.response.centerOfMass.z);

  supervisor_node_get_center_of_mass_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_number_of_contact_points_client;
  webots_ros::node_get_number_of_contact_points supervisor_node_get_number_of_contact_points_srv;
  supervisor_node_get_number_of_contact_points_client = n.serviceClient<webots_ros::node_get_number_of_contact_points>(
    model_name + "/supervisor/node/get_number_of_contact_points");

  supervisor_node_get_number_of_contact_points_srv.request.node = from_def_node;
  supervisor_node_get_number_of_contact_points_client.call(supervisor_node_get_number_of_contact_points_srv);
  ROS_INFO("From_def node got %d contact points.",
           supervisor_node_get_number_of_contact_points_srv.response.numberOfContactPoints);

  supervisor_node_get_number_of_contact_points_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_contact_point_client;
  webots_ros::node_get_contact_point supervisor_node_get_contact_point_srv;
  supervisor_node_get_contact_point_client =
    n.serviceClient<webots_ros::node_get_contact_point>(model_name + "/supervisor/node/get_contact_point");

  supervisor_node_get_contact_point_srv.request.node = from_def_node;
  supervisor_node_get_contact_point_srv.request.index = 0;
  supervisor_node_get_contact_point_client.call(supervisor_node_get_contact_point_srv);
  ROS_INFO("From_def_node first contact point is at x = %f, y = %f z = %f.",
           supervisor_node_get_contact_point_srv.response.point.x, supervisor_node_get_contact_point_srv.response.point.y,
           supervisor_node_get_contact_point_srv.response.point.z);

  supervisor_node_get_contact_point_client.shutdown();
  time_step_client.call(time_step_srv);

  // test get_static_balance
  // if the node isn't a top Solid webots will throw a warning but still return true to ros
  ros::ServiceClient supervisor_node_get_static_balance_client;
  webots_ros::node_get_static_balance supervisor_node_get_static_balance_srv;
  supervisor_node_get_static_balance_client =
    n.serviceClient<webots_ros::node_get_static_balance>(model_name + "/supervisor/node/get_static_balance");

  supervisor_node_get_static_balance_srv.request.node = from_def_node;
  supervisor_node_get_static_balance_client.call(supervisor_node_get_static_balance_srv);
  ROS_INFO("From_def node balance is %d.", supervisor_node_get_static_balance_srv.response.balance);

  supervisor_node_get_static_balance_client.shutdown();
  time_step_client.call(time_step_srv);

  // test reset_physics
  // if the node isn't a top Solid webots will throw a warning but still return true to ros
  ros::ServiceClient supervisor_node_reset_physics_client =
    n.serviceClient<webots_ros::node_reset_functions>(model_name + "/supervisor/node/reset_physics");
  webots_ros::node_reset_functions supervisor_node_reset_physics_srv;

  supervisor_node_reset_physics_srv.request.node = from_def_node;
  if (supervisor_node_reset_physics_client.call(supervisor_node_reset_physics_srv) &&
      supervisor_node_reset_physics_srv.response.success == 1)
    ROS_INFO("Node physics has been reset successfully.");
  else
    ROS_ERROR("Failed to call service node_reset_physics.");

  supervisor_node_reset_physics_client.shutdown();
  time_step_client.call(time_step_srv);

  // test restart_controller
  ros::ServiceClient supervisor_node_restart_controller_client =
    n.serviceClient<webots_ros::node_reset_functions>(model_name + "/supervisor/node/restart_controller");
  webots_ros::node_reset_functions supervisor_node_restart_controller_srv;

  supervisor_node_restart_controller_srv.request.node = from_def_node;
  if (supervisor_node_restart_controller_client.call(supervisor_node_restart_controller_srv) &&
      supervisor_node_restart_controller_srv.response.success == 1)
    ROS_INFO("Robot controller has been restarted successfully.");
  else
    ROS_ERROR("Failed to call service node_restart_controller.");

  supervisor_node_restart_controller_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_field_client;
  webots_ros::node_get_field supervisor_node_get_field_srv;
  supervisor_node_get_field_client = n.serviceClient<webots_ros::node_get_field>(model_name + "/supervisor/node/get_field");

  supervisor_node_get_field_srv.request.node = root_node;
  supervisor_node_get_field_srv.request.fieldName = "children";
  supervisor_node_get_field_srv.request.proto = 0;
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  uint64_t field = supervisor_node_get_field_srv.response.field;

  supervisor_node_get_field_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_field_get_type_client;
  webots_ros::field_get_type supervisor_field_get_type_srv;
  supervisor_field_get_type_client = n.serviceClient<webots_ros::field_get_type>(model_name + "/supervisor/field/get_type");

  supervisor_field_get_type_srv.request.field = field;
  supervisor_field_get_type_client.call(supervisor_field_get_type_srv);
  ROS_INFO("World's children field is of type %d.", supervisor_field_get_type_srv.response.type);

  supervisor_field_get_type_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_field_get_type_name_client;
  webots_ros::field_get_type_name supervisor_field_get_type_name_srv;
  supervisor_field_get_type_name_client =
    n.serviceClient<webots_ros::field_get_type_name>(model_name + "/supervisor/field/get_type_name");

  supervisor_field_get_type_name_srv.request.field = field;
  supervisor_field_get_type_name_client.call(supervisor_field_get_type_name_srv);
  ROS_INFO("Also known as %s.", supervisor_field_get_type_name_srv.response.name.c_str());

  supervisor_field_get_type_name_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_field_get_count_client;
  webots_ros::field_get_count supervisor_field_get_count_srv;
  supervisor_field_get_count_client = n.serviceClient<webots_ros::field_get_count>(model_name + "/supervisor/field/get_count");

  supervisor_field_get_count_srv.request.field = field;
  supervisor_field_get_count_client.call(supervisor_field_get_count_srv);
  if (supervisor_field_get_count_srv.response.count != -1)
    ROS_INFO("There is %d nodes in this field.", supervisor_field_get_count_srv.response.count);
  else
    ROS_ERROR("Illegal call to Field::getCount() argument must be multiple fields.");

  supervisor_field_get_count_client.shutdown();
  time_step_client.call(time_step_srv);

  supervisor_node_get_field_srv.request.node = from_def_node;
  supervisor_node_get_field_srv.request.fieldName = "name";
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  field = supervisor_node_get_field_srv.response.field;

  ros::ServiceClient supervisor_field_set_string_client;
  webots_ros::field_set_string supervisor_field_set_string_srv;
  supervisor_field_set_string_client =
    n.serviceClient<webots_ros::field_set_string>(model_name + "/supervisor/field/set_string");

  supervisor_field_set_string_srv.request.field = field;
  supervisor_field_set_string_srv.request.value = "solid_test";
  if (supervisor_field_set_string_client.call(supervisor_field_set_string_srv) &&
      supervisor_field_set_string_srv.response.success == 1)
    ROS_INFO("Field's string updated to: 'solid_test'.");
  else
    ROS_ERROR("Failed to call service field_set_string.");

  supervisor_field_set_string_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_field_get_string_client;
  webots_ros::field_get_string supervisor_field_get_string_srv;
  supervisor_field_get_string_client =
    n.serviceClient<webots_ros::field_get_string>(model_name + "/supervisor/field/get_string");

  supervisor_field_get_string_srv.request.field = field;
  supervisor_field_get_string_client.call(supervisor_field_get_string_srv);
  ROS_INFO("Field contains \"%s\".", supervisor_field_get_string_srv.response.value.c_str());

  supervisor_field_get_string_client.shutdown();
  time_step_client.call(time_step_srv);

  supervisor_node_get_field_srv.request.node = root_node;
  supervisor_node_get_field_srv.request.fieldName = "children";
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  field = supervisor_node_get_field_srv.response.field;

  ros::ServiceClient supervisor_field_get_node_client;
  webots_ros::field_get_node supervisor_field_get_node_srv;
  supervisor_field_get_node_client = n.serviceClient<webots_ros::field_get_node>(model_name + "/supervisor/field/get_node");

  supervisor_field_get_node_srv.request.field = field;
  supervisor_field_get_node_srv.request.index = 7;
  supervisor_field_get_node_client.call(supervisor_field_get_node_srv);

  supervisor_node_get_type_name_srv.request.node = supervisor_field_get_node_srv.response.node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("Node got from field_get_node is of type %s.", supervisor_node_get_type_name_srv.response.name.c_str());

  // supervisor_node_get_from_id
  supervisor_get_from_def_srv.request.name = "CONE";
  supervisor_get_from_def_srv.request.proto = 0;
  supervisor_get_from_def_client.call(supervisor_get_from_def_srv);
  uint64_t cone_node = 0;
  if (supervisor_get_from_def_srv.response.node != 0) {
    ROS_INFO("Got CONE node from DEF: %lu.", supervisor_get_from_def_srv.response.node);
    cone_node = supervisor_get_from_def_srv.response.node;
  } else
    ROS_ERROR("could not get CONE node from DEF.");

  supervisor_node_get_type_name_client.shutdown();
  supervisor_get_from_def_client.shutdown();
  supervisor_field_get_node_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient node_get_id_client;
  webots_ros::node_get_id node_get_id_srv;
  node_get_id_client = n.serviceClient<webots_ros::node_get_id>(model_name + "/supervisor/node/get_id");
  node_get_id_srv.request.node = cone_node;
  node_get_id_client.call(node_get_id_srv);
  int cone_node_id = node_get_id_srv.response.id;
  if (cone_node_id > 0)
    ROS_INFO("Node id got successfully.");
  else
    ROS_ERROR("Failed to call service node_get_id.");

  node_get_id_client.shutdown();
  time_step_client.call(time_step_srv);

  // supervisor_get_from_id
  webots_ros::supervisor_get_from_id supervisor_get_from_id_srv;
  node_get_id_client = n.serviceClient<webots_ros::supervisor_get_from_id>(model_name + "/supervisor/get_from_id");
  supervisor_get_from_id_srv.request.id = cone_node_id;
  node_get_id_client.call(supervisor_get_from_id_srv);
  uint64_t cone_node_copy = supervisor_get_from_id_srv.response.node;
  if (cone_node_copy == cone_node)
    ROS_INFO("Cone node got successfully from id.");
  else
    ROS_ERROR("Failed to call service supervisor_get_from_id.");

  node_get_id_client.shutdown();
  time_step_client.call(time_step_srv);

  // node_set_velocity
  ros::ServiceClient node_velocity_client;
  webots_ros::node_set_velocity node_set_velocity_srv;
  node_velocity_client = n.serviceClient<webots_ros::node_set_velocity>(model_name + "/supervisor/node/set_velocity");
  node_set_velocity_srv.request.node = cone_node;
  node_set_velocity_srv.request.velocity.linear.x = 0.0;
  node_set_velocity_srv.request.velocity.linear.y = 0.0;
  node_set_velocity_srv.request.velocity.linear.z = 1.0;
  node_set_velocity_srv.request.velocity.angular.x = 0.0;
  node_set_velocity_srv.request.velocity.angular.y = 0.0;
  node_set_velocity_srv.request.velocity.angular.z = 0.0;
  if (node_velocity_client.call(node_set_velocity_srv) && node_set_velocity_srv.response.success == 1)
    ROS_INFO("Node velocity set successfully.");
  else
    ROS_ERROR("Failed to call service node_set_velocity.");

  node_velocity_client.shutdown();
  time_step_client.call(time_step_srv);

  // node_get_velocity
  webots_ros::node_get_velocity node_get_velocity_srv;
  node_velocity_client = n.serviceClient<webots_ros::node_get_velocity>(model_name + "/supervisor/node/get_velocity");
  node_get_velocity_srv.request.node = cone_node;
  node_velocity_client.call(node_get_velocity_srv);
  if (node_get_velocity_srv.response.velocity.linear.z > 0.8)
    ROS_INFO("Node velocity get successfully.");
  else
    ROS_ERROR("Failed to call service node_get_velocity.");

  node_velocity_client.shutdown();
  time_step_client.call(time_step_srv);

  // node_add_force_or_torque
  ros::ServiceClient node_add_force_or_torque_client;
  webots_ros::node_add_force_or_torque node_add_force_or_torque_srv;
  node_add_force_or_torque_client =
    n.serviceClient<webots_ros::node_add_force_or_torque>(model_name + "/supervisor/node/add_torque");
  node_add_force_or_torque_srv.request.node = cone_node;
  node_add_force_or_torque_srv.request.force.x = 0.0;
  node_add_force_or_torque_srv.request.force.y = 0.0;
  node_add_force_or_torque_srv.request.force.z = 1.0;
  node_add_force_or_torque_srv.request.relative = 0;
  if (node_add_force_or_torque_client.call(node_add_force_or_torque_srv) && node_add_force_or_torque_srv.response.success == 1)
    ROS_INFO("Node force added successfully.");
  else
    ROS_ERROR("Failed to call service node_add_force_or_torque.");

  node_add_force_or_torque_client.shutdown();
  time_step_client.call(time_step_srv);

  // node_add_force_with_offset
  ros::ServiceClient node_add_force_with_offset_client;
  webots_ros::node_add_force_with_offset node_add_force_with_offset_srv;
  node_add_force_with_offset_client =
    n.serviceClient<webots_ros::node_add_force_with_offset>(model_name + "/supervisor/node/add_force_with_offset");
  node_add_force_with_offset_srv.request.node = cone_node;
  node_add_force_with_offset_srv.request.force.x = 0.0;
  node_add_force_with_offset_srv.request.force.y = 0.0;
  node_add_force_with_offset_srv.request.force.z = 1.0;
  node_add_force_with_offset_srv.request.offset.x = 0.0;
  node_add_force_with_offset_srv.request.offset.y = 0.0;
  node_add_force_with_offset_srv.request.offset.z = 1.0;
  node_add_force_with_offset_srv.request.relative = 0;
  if (node_add_force_with_offset_client.call(node_add_force_with_offset_srv) &&
      node_add_force_with_offset_srv.response.success == 1)
    ROS_INFO("Node force added successfully.");
  else
    ROS_ERROR("Failed to call service node_add_force_with_offset.");

  node_add_force_with_offset_client.shutdown();
  time_step_client.call(time_step_srv);

  // node_get_parent
  ros::ServiceClient node_get_parent_node_client;
  webots_ros::node_get_parent_node node_get_parent_node_srv;
  node_get_parent_node_client =
    n.serviceClient<webots_ros::node_get_parent_node>(model_name + "/supervisor/node/get_parent_node");
  node_get_parent_node_srv.request.node = cone_node;
  node_get_parent_node_client.call(node_get_parent_node_srv);
  if (node_get_parent_node_srv.response.node == root_node)
    ROS_INFO("Node parent got successfully.");
  else
    ROS_ERROR("Failed to call service node_get_parent_node.");

  node_get_parent_node_client.shutdown();
  time_step_client.call(time_step_srv);

  // movie
  ros::ServiceClient supervisor_movie_is_ready_client;
  webots_ros::node_get_status supervisor_movie_is_ready_srv;
  supervisor_movie_is_ready_client = n.serviceClient<webots_ros::node_get_status>(model_name + "/supervisor/movie_is_ready");

  if (supervisor_movie_is_ready_client.call(supervisor_movie_is_ready_srv) &&
      supervisor_movie_is_ready_srv.response.status == 1)
    ROS_INFO("Ready to record a movie.");
  else
    ROS_ERROR("Failed to call service supervisor_movie_is_ready.");

  supervisor_movie_is_ready_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_movie_start_client;
  webots_ros::supervisor_movie_start_recording supervisor_movie_start_srv;
  supervisor_movie_start_client =
    n.serviceClient<webots_ros::supervisor_movie_start_recording>(model_name + "/supervisor/movie_start_recording");

  supervisor_movie_start_srv.request.filename = std::string(getenv("HOME")) + std::string("/movie_test.mp4");
  supervisor_movie_start_srv.request.width = 480;
  supervisor_movie_start_srv.request.height = 360;
  supervisor_movie_start_srv.request.codec = 1337;
  supervisor_movie_start_srv.request.quality = 100;
  supervisor_movie_start_srv.request.acceleration = 1;
  supervisor_movie_start_srv.request.caption = false;
  if (supervisor_movie_start_client.call(supervisor_movie_start_srv) && supervisor_movie_start_srv.response.success == 1)
    ROS_INFO("Movie started successfully.");
  else
    ROS_ERROR("Failed to call service movie_start_recording.");

  supervisor_movie_start_client.shutdown();
  for (int i = 0; i < 25; ++i)
    time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_movie_stop_client;
  webots_ros::get_bool supervisor_movie_stop_srv;
  supervisor_movie_stop_client = n.serviceClient<webots_ros::get_bool>(model_name + "/supervisor/movie_stop_recording");

  if (supervisor_movie_stop_client.call(supervisor_movie_stop_srv) && supervisor_movie_stop_srv.response.value)
    ROS_INFO("Movie stopped successfully.");
  else
    ROS_ERROR("Failed to call service movie_stop_recording.");

  supervisor_movie_stop_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_movie_failed_client;
  webots_ros::node_get_status supervisor_movie_failed_srv;
  supervisor_movie_failed_client = n.serviceClient<webots_ros::node_get_status>(model_name + "/supervisor/movie_failed");

  if (supervisor_movie_failed_client.call(supervisor_movie_failed_srv) && supervisor_movie_failed_srv.response.status == 0)
    ROS_INFO("Movie recording executing successfully.");
  else
    ROS_ERROR("Failed movie recording.");

  supervisor_movie_failed_client.shutdown();
  time_step_client.call(time_step_srv);

  supervisor_set_label_srv.request.label = "";
  supervisor_set_label_client.call(supervisor_set_label_srv);

  supervisor_set_label_client.shutdown();
  time_step_client.call(time_step_srv);

  // field_remove
  ros::ServiceClient remove_node_client;
  webots_ros::field_remove field_remove_srv;
  remove_node_client = n.serviceClient<webots_ros::field_remove>(model_name + "/supervisor/field/remove");
  field_remove_srv.request.field = field;
  field_remove_srv.request.index = 5;
  if (remove_node_client.call(field_remove_srv) && field_remove_srv.response.success == 1)
    ROS_INFO("Field node removed successfully.");
  else
    ROS_ERROR("Failed to call service field_remove.");

  remove_node_client.shutdown();
  time_step_client.call(time_step_srv);

  // node_remove
  webots_ros::node_remove node_remove_srv;
  remove_node_client = n.serviceClient<webots_ros::node_remove>(model_name + "/supervisor/node/remove");
  node_remove_srv.request.node = cone_node;
  remove_node_client.call(node_remove_srv);
  int success1 = node_remove_srv.response.success;
  if (success1 == 1)
    ROS_INFO("Node removed successfully.");
  else
    ROS_ERROR("Failed to call service node_removed.");

  remove_node_client.shutdown();
  time_step_client.call(time_step_srv);

  // html robot window
  ros::ServiceClient wwi_send_client;
  wwi_send_client = n.serviceClient<webots_ros::set_string>(model_name + "/robot/wwi_send_text");
  webots_ros::set_string wwi_send_srv;
  wwi_send_srv.request.value = "test wwi functions from complete_test controller.";
  if (wwi_send_client.call(wwi_send_srv) && wwi_send_srv.response.success == 1)
    ROS_INFO("Text to robot window successfully sent.");
  else
    ROS_ERROR("Failed to call service robot/wwi_send_text.");

  wwi_send_client.shutdown();
  time_step_client.call(time_step_srv);

  ros::ServiceClient wwi_receive_client;
  wwi_receive_client = n.serviceClient<webots_ros::get_string>(model_name + "/robot/wwi_receive_text");
  webots_ros::get_string wwi_receive_srv;
  if (wwi_receive_client.call(wwi_receive_srv) &&
      wwi_receive_srv.response.value.compare("Answer: test wwi functions from complete_test controller.") == 0)
    ROS_INFO("Text from robot window successfully received.");
  else
    ROS_ERROR("Failed to call service robot/wwi_receive_text.");

  wwi_receive_client.shutdown();
  time_step_client.call(time_step_srv);

  // virtual reality headset
  ros::ServiceClient virtual_reality_headset_client;
  webots_ros::get_bool supervisor_virtual_reality_headset_is_used_srv;
  virtual_reality_headset_client =
    n.serviceClient<webots_ros::get_bool>(model_name + "/supervisor/vitual_reality_headset_is_used");
  virtual_reality_headset_client.call(supervisor_virtual_reality_headset_is_used_srv);
  bool used = supervisor_virtual_reality_headset_is_used_srv.response.value;
  // to test this service we assume no virtual reality headset is connected
  if (!used)
    ROS_INFO("No virtual reality headset connected.");
  else
    ROS_ERROR("Virtual reality headset wrongly detected.");

  virtual_reality_headset_client.shutdown();
  time_step_client.call(time_step_srv);

  // test field_import_node
  // this test is disabled as it needs a webots object file but it will work if you export the cone_test solid from the world
  // before
  //  ros::ServiceClient supervisor_field_import_node_client;
  //  webots_ros::field_import_node supervisor_field_import_node_srv;
  //  supervisor_field_import_node_client =
  //  n.serviceClient<webots_ros::field_import_node>(model_name+"/supervisor/field/import_node");
  //
  //  supervisor_field_import_node_srv.request.field = field;
  //  supervisor_field_import_node_srv.request.position = 6;
  //  supervisor_field_import_node_srv.request.filename = "cone_test.wbo";
  //  if (supervisor_field_import_node_client.call(supervisor_field_import_node_srv) &&
  //  supervisor_field_import_node_srv.response.success == 1)
  //    ROS_INFO("New cone add in world.");
  //  else
  //    ROS_ERROR("Failed to call service field_import_node.");

  // The next 2 tests are commented but works if you want to use them.
  // Since they stop simulation we can't use them if we wants to do other tests afterwards

  //~ ros::ServiceClient supervisor_simulation_quit_client;
  //~ webots_ros::set_int supervisor_simulation_quit_srv;
  //~ supervisor_simulation_quit_client = n.serviceClient<webots_ros::set_int>(model_name+"/supervisor/simulation_quit");
  //~
  //~ supervisor_simulation_quit_srv.request.value = EXIT_SUCCESS;
  //~ if (supervisor_simulation_quit_client.call(supervisor_simulation_quit_srv) &&
  //~    supervisor_simulation_quit_srv.response.success)
  //~   ROS_INFO("Webots quit successfully.");
  //~ else
  //~   ROS_ERROR("Failed to call service simulation_quit.");

  //~ ros::ServiceClient supervisor_simulation_revert_client;
  //~ webots_ros::get_bool supervisor_simulation_revert_srv;
  //~ supervisor_simulation_revert_client = n.serviceClient<webots_ros::get_bool>(model_name+"/supervisor/simulation_revert");
  //~
  //~ if (supervisor_simulation_revert_client.call(supervisor_simulation_revert_srv) &&
  //~    supervisor_simulation_revert_srv.response.value)
  //~   ROS_INFO("Simulation has revert successfully.");
  //~ else
  //~   ROS_ERROR("Failed to call service simulation_revert.");

  // end of tests
  time_step_srv.request.value = 0;
  if (time_step_client.call(time_step_srv) && time_step_srv.response.success)
    ROS_INFO("Robot's time step called to end tests.");
  else
    ROS_ERROR("Failed to call service time_step to end tests.");

  time_step_client.shutdown();

  ros::shutdown();

  printf("\nTest Completed\n");
  return 0;
}
