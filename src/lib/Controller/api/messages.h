/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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

// for the root device

// sim -> ctr
#define C_ROBOT_QUIT 1
#define C_ROBOT_JOYSTICK_CONFIG 2
#define C_ROBOT_WAIT_FOR_USER_INPUT_EVENT 3
#define C_ROBOT_TIME 10
#define C_ROBOT_BATTERY_VALUE 11
#define C_ROBOT_KEYBOARD_VALUE 12
#define C_ROBOT_JOYSTICK_VALUE 13
#define C_ROBOT_MOUSE_VALUE 14
#define C_ROBOT_WINDOW_SHOW 15
#define C_ROBOT_SIMULATION_CHANGE_MODE 16
#define C_ROBOT_DATA 17
#define C_ROBOT_MODEL 18
#define C_SUPERVISOR_ANIMATION_START_STATUS 19
#define C_SUPERVISOR_ANIMATION_STOP_STATUS 20
#define C_ROBOT_WINDOW_UPDATE 22
#define C_SUPERVISOR_MOVIE_STATUS 166

// ctr <-> sim
#define C_ROBOT_SET_BATTERY_SAMPLING_PERIOD 4
#define C_ROBOT_REMOTE_ON 5
#define C_ROBOT_REMOTE_OFF 6
#define C_ROBOT_SET_KEYBOARD_SAMPLING_PERIOD 7
#define C_ROBOT_SET_JOYSTICK_SAMPLING_PERIOD 8
#define C_ROBOT_SET_MOUSE_SAMPLING_PERIOD 9
#define C_ROBOT_SET_DATA 10
#define C_ROBOT_CLIENT_EXIT_NOTIFY 11
#define C_ROBOT_MOUSE_ENABLE_3D_POSITION 12
#define C_ROBOT_PIN 13
#define C_ROBOT_WWI_MESSAGE 21
#define C_DIFFERENTIAL_WHEELS_SET_SPEED 100
#define C_DIFFERENTIAL_WHEELS_ENCODERS_SET_SAMPLING_PERIOD 101
#define C_DIFFERENTIAL_WHEELS_ENCODERS_SET_VALUE 102
#define C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK 103
#define C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK_DURATION 104
#define C_ROBOT_SET_JOYSTICK_AUTO_CENTERING_GAIN 105
#define C_ROBOT_SET_JOYSTICK_RESISTANCE_GAIN 106
#define C_ROBOT_SET_JOYSTICK_FORCE_AXIS 107
#define C_SUPERVISOR_SAVE_WORLD 149
#define C_SUPERVISOR_EXPORT_IMAGE 150
#define C_SUPERVISOR_LOAD_WORLD 151
#define C_SUPERVISOR_RELOAD_WORLD 152
#define C_SUPERVISOR_NODE_GET_FROM_ROOT 153
#define C_SUPERVISOR_SET_LABEL 154
#define C_SUPERVISOR_SIMULATION_QUIT 155
#define C_SUPERVISOR_SIMULATION_RESET 156
#define C_SUPERVISOR_SIMULATION_CHANGE_MODE 160
#define C_SUPERVISOR_SIMULATION_RESET_PHYSICS 163
#define C_SUPERVISOR_START_MOVIE 164
#define C_SUPERVISOR_STOP_MOVIE 165
#define C_SUPERVISOR_START_ANIMATION 167
#define C_SUPERVISOR_STOP_ANIMATION 168
#define C_SUPERVISOR_NODE_GET_FROM_ID 169
#define C_SUPERVISOR_NODE_GET_FROM_DEF 170
#define C_SUPERVISOR_NODE_GET_SELECTED 171
#define C_SUPERVISOR_FIELD_GET_FROM_NAME 172
#define C_SUPERVISOR_FIELD_GET_VALUE 173
#define C_SUPERVISOR_FIELD_SET_VALUE 174
#define C_SUPERVISOR_FIELD_INSERT_VALUE 175
#define C_SUPERVISOR_FIELD_IMPORT_NODE_FROM_STRING 176
#define C_SUPERVISOR_FIELD_REMOVE_VALUE 177
#define C_SUPERVISOR_NODE_GET_POSITION 178
#define C_SUPERVISOR_NODE_GET_ORIENTATION 179
#define C_SUPERVISOR_NODE_GET_CENTER_OF_MASS 180
#define C_SUPERVISOR_NODE_GET_CONTACT_POINTS 181
#define C_SUPERVISOR_NODE_GET_STATIC_BALANCE 182
#define C_SUPERVISOR_NODE_GET_VELOCITY 183
#define C_SUPERVISOR_NODE_SET_VELOCITY 184
#define C_SUPERVISOR_NODE_RESET_PHYSICS 185
#define C_SUPERVISOR_NODE_RESTART_CONTROLLER 186
#define C_SUPERVISOR_NODE_REMOVE_NODE 187
#define C_SUPERVISOR_NODE_SET_VISIBILITY 188
#define C_SUPERVISOR_NODE_MOVE_VIEWPOINT 189
#define C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_IS_USED 190
#define C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_POSITION 191
#define C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION 192

// console
// ctr -> sim
#define C_CONSOLE_MESSAGE 20

// for the differential wheels device
// sim -> ctr
#define C_DIFFERENTIAL_WHEELS_GET_ENCODERS 100

// for the camera device
// ctr -> sim
#define C_CAMERA_SET_FOV 4
#define C_CAMERA_SET_FOCAL 5
// sim -> ctr
#define C_CAMERA_RECONFIGURE 6
#define C_CAMERA_SHARED_MEMORY 7
// ctr -> sim
#define C_CAMERA_SET_RECOGNITION_SAMPLING_PERIOD 8
#define C_CAMERA_GET_IMAGE 9
#define C_CAMERA_OBJECTS 10

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
