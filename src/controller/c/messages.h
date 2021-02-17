/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MESSAGES_H
#define MESSAGES_H

// for any device

// both directions
#define C_CONFIGURE 0

// ctr -> sim
#define C_SET_SAMPLING_PERIOD 1

// for the root device (robot)

// sim -> ctr
#define C_ROBOT_QUIT 2
#define C_ROBOT_JOYSTICK_CONFIG 3
#define C_ROBOT_TIME 4
#define C_ROBOT_BATTERY_VALUE 5
#define C_ROBOT_KEYBOARD_VALUE 6
#define C_ROBOT_JOYSTICK_VALUE 7
#define C_ROBOT_MOUSE_VALUE 8
#define C_ROBOT_WINDOW_SHOW 9
#define C_ROBOT_SIMULATION_CHANGE_MODE 10
#define C_ROBOT_DATA 11
#define C_ROBOT_SUPERVISOR 12
#define C_ROBOT_MODEL 13
#define C_ROBOT_WINDOW_UPDATE 14
#define C_ROBOT_NEW_DEVICE 15
#define C_SUPERVISOR_ANIMATION_START_STATUS 16
#define C_SUPERVISOR_ANIMATION_STOP_STATUS 17
#define C_SUPERVISOR_MOVIE_STATUS 18
#define C_SUPERVISOR_NODE_REGENERATED 19

// ctr -> sim
#define C_CONSOLE_MESSAGE 30
#define C_ROBOT_SET_BATTERY_SAMPLING_PERIOD 31
#define C_ROBOT_REMOTE_ON 32
#define C_ROBOT_REMOTE_OFF 33
#define C_ROBOT_SET_KEYBOARD_SAMPLING_PERIOD 34
#define C_ROBOT_SET_JOYSTICK_SAMPLING_PERIOD 35
#define C_ROBOT_SET_MOUSE_SAMPLING_PERIOD 36
#define C_ROBOT_SET_DATA 37
#define C_ROBOT_CLIENT_EXIT_NOTIFY 38
#define C_ROBOT_MOUSE_ENABLE_3D_POSITION 39
#define C_ROBOT_PIN 40
#define C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK 41
#define C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK_DURATION 42
#define C_ROBOT_SET_JOYSTICK_AUTO_CENTERING_GAIN 43
#define C_ROBOT_SET_JOYSTICK_RESISTANCE_GAIN 44
#define C_ROBOT_SET_JOYSTICK_FORCE_AXIS 45
#define C_ROBOT_URDF 46
#define C_SUPERVISOR_EXPORT_IMAGE 50
#define C_SUPERVISOR_LOAD_WORLD 51
#define C_SUPERVISOR_RELOAD_WORLD 52
#define C_SUPERVISOR_SET_LABEL 53
#define C_SUPERVISOR_SIMULATION_QUIT 54
#define C_SUPERVISOR_SIMULATION_RESET 55
#define C_SUPERVISOR_SIMULATION_CHANGE_MODE 56
#define C_SUPERVISOR_SIMULATION_RESET_PHYSICS 57
#define C_SUPERVISOR_START_MOVIE 58
#define C_SUPERVISOR_STOP_MOVIE 59
#define C_SUPERVISOR_START_ANIMATION 60
#define C_SUPERVISOR_STOP_ANIMATION 61
#define C_SUPERVISOR_FIELD_SET_VALUE 62
#define C_SUPERVISOR_FIELD_IMPORT_NODE_FROM_STRING 63
#define C_SUPERVISOR_FIELD_REMOVE_VALUE 64
#define C_SUPERVISOR_NODE_SET_VELOCITY 65
#define C_SUPERVISOR_NODE_RESET_PHYSICS 66
#define C_SUPERVISOR_NODE_RESTART_CONTROLLER 67
#define C_SUPERVISOR_NODE_SET_VISIBILITY 68
#define C_SUPERVISOR_NODE_MOVE_VIEWPOINT 69
#define C_SUPERVISOR_NODE_ADD_FORCE 70
#define C_SUPERVISOR_NODE_ADD_FORCE_WITH_OFFSET 71
#define C_SUPERVISOR_NODE_ADD_TORQUE 72
#define C_SUPERVISOR_NODE_EXPORT_STRING 75

// ctr <-> sim
#define C_ROBOT_WAIT_FOR_USER_INPUT_EVENT 80
#define C_ROBOT_WWI_MESSAGE 81
#define C_SUPERVISOR_SAVE_WORLD 82
#define C_SUPERVISOR_NODE_GET_FROM_ID 83
#define C_SUPERVISOR_NODE_GET_FROM_DEF 84
#define C_SUPERVISOR_NODE_GET_FROM_TAG 85
#define C_SUPERVISOR_NODE_GET_SELECTED 86
#define C_SUPERVISOR_FIELD_GET_FROM_NAME 87
#define C_SUPERVISOR_FIELD_GET_VALUE 88
#define C_SUPERVISOR_FIELD_INSERT_VALUE 89
#define C_SUPERVISOR_NODE_GET_POSITION 90
#define C_SUPERVISOR_NODE_GET_ORIENTATION 91
#define C_SUPERVISOR_NODE_GET_CENTER_OF_MASS 92
#define C_SUPERVISOR_NODE_GET_CONTACT_POINTS 93
#define C_SUPERVISOR_NODE_GET_STATIC_BALANCE 94
#define C_SUPERVISOR_NODE_GET_VELOCITY 95
#define C_SUPERVISOR_NODE_REMOVE_NODE 96
#define C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_IS_USED 97
#define C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_POSITION 98
#define C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION 99

// for the camera device
// ctr -> sim
#define C_CAMERA_SET_FOV 4
#define C_CAMERA_SET_FOCAL 5
#define C_CAMERA_SET_EXPOSURE 6
// sim -> ctr
#define C_CAMERA_RECONFIGURE 7
#define C_CAMERA_SHARED_MEMORY 8
// ctr <-> sim
#define C_CAMERA_GET_IMAGE 9

// for the camera recognition
// ctr -> sim
#define C_CAMERA_SET_RECOGNITION_SAMPLING_PERIOD 10
#define C_CAMERA_ENABLE_SEGMENTATION 11
// sim -> ctr
#define C_CAMERA_OBJECTS 12
#define C_CAMERA_SEGMENTATION_SHARED_MEMORY 13
#define C_CAMERA_SET_SEGMENTATION 14
// ctr <-> sim
#define C_CAMERA_GET_SEGMENTATION_IMAGE 15

// for the emitter device
// ctr -> sim
#define C_EMITTER_SEND 0
// ctr <-> sim
#define C_EMITTER_SET_CHANNEL 1
#define C_EMITTER_SET_RANGE 2
// sim -> ctr
#define C_EMITTER_SET_BUFFER_SIZE 3

// for the receiver device
// ctr -> sim
#define C_RECEIVER_RECEIVE 1
#define C_RECEIVER_SET_CHANNEL 2

// for the skin device
// ctr -> sim
#define C_SKIN_SET_BONE_ORIENTATION 1
#define C_SKIN_SET_BONE_POSITION 2
// ctr <-> sim
#define C_SKIN_GET_BONE_ORIENTATION 3
#define C_SKIN_GET_BONE_POSITION 4

// for the motor device
// ctr -> sim
#define C_MOTOR_SET_POSITION 1
#define C_MOTOR_SET_VELOCITY 2
#define C_MOTOR_SET_AVAILABLE_FORCE 3
#define C_MOTOR_SET_FORCE 4
#define C_MOTOR_FEEDBACK 5
#define C_MOTOR_SET_ACCELERATION 6
#define C_MOTOR_SET_CONTROL_PID 7
// ctr <-> sim
#define C_MOTOR_GET_ASSOCIATED_DEVICE 8

// for the brake device
// ctr -> sim, to be ORed
#define C_BRAKE_SET_DAMPING_CONSTANT 2
// ctr <-> sim, to be ORed
#define C_BRAKE_GET_ASSOCIATED_DEVICE 4

// for the led device
// ctr -> sim
#define C_LED_SET 1

// for the display device
// ctr -> sim
#define C_DISPLAY_SET_COLOR 16
#define C_DISPLAY_SET_ALPHA 17
#define C_DISPLAY_SET_OPACITY 18
#define C_DISPLAY_SET_FONT 19
#define C_DISPLAY_ATTACH_CAMERA 20
#define C_DISPLAY_DETACH_CAMERA 21
#define C_DISPLAY_DRAW_PIXEL 32
#define C_DISPLAY_DRAW_LINE 33
#define C_DISPLAY_DRAW_TEXT 34
#define C_DISPLAY_DRAW_RECTANGLE 35
#define C_DISPLAY_DRAW_OVAL 36
#define C_DISPLAY_DRAW_POLYGON 37
#define C_DISPLAY_IMAGE_COPY 64
#define C_DISPLAY_IMAGE_PASTE 65
#define C_DISPLAY_IMAGE_SAVE 66
#define C_DISPLAY_IMAGE_LOAD 67
#define C_DISPLAY_IMAGE_DELETE 68
#define C_DISPLAY_IMAGE_GET_ALL 69

// for the camera device
// ctr -> sim
#define C_GPS_DATA 2

// for the lidar device
// ctr -> sim
#define C_LIDAR_SET_FREQUENCY 16
#define C_LIDAR_ENABLE_POINT_CLOUD 17
#define C_LIDAR_DISABLE_POINT_CLOUD 18

// for the pen device
// ctr-> sim
#define C_PEN_WRITE 1
#define C_PEN_DONT_WRITE 2
#define C_PEN_SET_INK_COLOR 4

// for the position sensor device
// sim -> ctr
#define C_POSITION_SENSOR_DATA 1
// ctr <-> sim
#define C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE 2

// for the radar device
// sim -> ctr
#define C_RADAR_DATA 1

// for the radio device
// ctr -> sim
#define C_RADIO_SEND 2
#define C_RADIO_SET_ADDRESS 3
#define C_RADIO_SET_FREQUENCY 4
#define C_RADIO_SET_CHANNEL 5
#define C_RADIO_SET_BITRATE 6
#define C_RADIO_SET_RX_SENSITIVITY 7
#define C_RADIO_SET_TX_POWER 8

// sim -> ctr
#define C_RADIO_RECEIVE 1

// connector device
#define C_CONNECTOR_GET_PRESENCE 1
#define C_CONNECTOR_LOCK 2
#define C_CONNECTOR_UNLOCK 3

// DistanceSensor device
#define C_DISTANCE_SENSOR_DATA 50

// Accelerometer device
#define C_ACCELEROMETER_DATA 51

// Compass device
#define C_COMPASS_DATA 52

// Gyro device
#define C_GYRO_DATA 53

// Inertial unit device
#define C_INERTIAL_UNIT_DATA 54

// LightSensor device
#define C_LIGHT_SENSOR_DATA 55

// TouchSensor device
#define C_TOUCH_SENSOR_DATA 40
#define C_TOUCH_SENSOR_DATA_3D 41

// Speaker device
// ctr -> sim
#define C_SPEAKER_PLAY_SOUND 1
#define C_SPEAKER_STOP 2
#define C_SPEAKER_SET_ENGINE 3
#define C_SPEAKER_SET_LANGUAGE 4
#define C_SPEAKER_SPEAK 5
// sim -> ctr
#define C_SPEAKER_SOUND_OVER 6
#define C_SPEAKER_SPEAK_OVER 7

// Propeller device
// ctr -> sim
#define C_PROPELLER_SET_TORQUE 1

// Microphone device
#define C_MICROPHONE_RECEIVE 1

#endif  // MESSAGES_H
