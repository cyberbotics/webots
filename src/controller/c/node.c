/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/nodes.h>

const int wb_NODE_NO_NODE = WB_NODE_NO_NODE;

// 3D rendering
const int wb_NODE_APPEARANCE = WB_NODE_APPEARANCE, wb_NODE_BACKGROUND = WB_NODE_BACKGROUND,
          wb_NODE_BILLBOARD = WB_NODE_BILLBOARD, wb_NODE_BOX = WB_NODE_BOX, wb_NODE_CAD_SHAPE = WB_NODE_CAD_SHAPE,
          wb_NODE_CAPSULE = WB_NODE_CAPSULE, wb_NODE_COLOR = WB_NODE_COLOR, wb_NODE_CONE = WB_NODE_CONE,
          wb_NODE_COORDINATE = WB_NODE_COORDINATE, wb_NODE_CYLINDER = WB_NODE_CYLINDER, wb_NODE_DIRECTIONAL_LIGHT,
          wb_NODE_ELEVATION_GRID = WB_NODE_ELEVATION_GRID, wb_NODE_FOG = WB_NODE_FOG, wb_NODE_GROUP = WB_NODE_GROUP,
          wb_NODE_IMAGE_TEXTURE = WB_NODE_IMAGE_TEXTURE, wb_NODE_INDEXED_FACE_SET = WB_NODE_INDEXED_FACE_SET,
          wb_NODE_INDEXED_LINE_SET = WB_NODE_INDEXED_LINE_SET, wb_NODE_MATERIAL = WB_NODE_MATERIAL, wb_NODE_MESH = WB_NODE_MESH,
          wb_NODE_MUSCLE = WB_NODE_MUSCLE, wb_NODE_NORMAL = WB_NODE_NORMAL, wb_NODE_PBR_APPEARANCE = WB_NODE_PBR_APPEARANCE,
          wb_NODE_PLANE = WB_NODE_PLANE, wb_NODE_POINT_LIGHT = WB_NODE_POINT_LIGHT, wb_NODE_POINT_SET = WB_NODE_POINT_SET,
          wb_NODE_POSE = WB_NODE_POSE, wb_NODE_SHAPE = WB_NODE_SHAPE, wb_NODE_SPHERE = WB_NODE_SPHERE,
          wb_NODE_SPOT_LIGHT = WB_NODE_SPOT_LIGHT, wb_NODE_TEXTURE_COORDINATE = WB_NODE_TEXTURE_COORDINATE,
          wb_NODE_TEXTURE_TRANSFORM = WB_NODE_TEXTURE_TRANSFORM, wb_NODE_TRANSFORM = WB_NODE_TRANSFORM,
          wb_NODE_VIEWPOINT = WB_NODE_VIEWPOINT;
// robots
int wb_NODE_ROBOT = WB_NODE_ROBOT;
// devices
const int wb_NODE_ACCELEROMETER = WB_NODE_ACCELEROMETER, wb_NODE_ALTIMETER = WB_NODE_ALTIMETER, wb_NODE_BRAKE = WB_NODE_BRAKE,
          wb_NODE_CAMERA = WB_NODE_CAMERA, wb_NODE_COMPASS = WB_NODE_COMPASS, wb_NODE_CONNECTOR = WB_NODE_CONNECTOR,
          wb_NODE_DISPLAY = WB_NODE_DISPLAY, wb_NODE_DISTANCE_SENSOR = WB_NODE_DISTANCE_SENSOR,
          wb_NODE_EMITTER = WB_NODE_EMITTER, wb_NODE_GPS = WB_NODE_GPS, wb_NODE_GYRO = WB_NODE_GYRO,
          wb_NODE_INERTIAL_UNIT = WB_NODE_INERTIAL_UNIT, wb_NODE_LED = WB_NODE_LED, wb_NODE_LIDAR = WB_NODE_LIDAR,
          wb_NODE_LIGHT_SENSOR = WB_NODE_LIGHT_SENSOR, wb_NODE_LINEAR_MOTOR = WB_NODE_LINEAR_MOTOR, wb_NODE_PEN = WB_NODE_PEN,
          wb_NODE_POSITION_SENSOR = WB_NODE_POSITION_SENSOR, wb_NODE_PROPELLER = WB_NODE_PROPELLER,
          wb_NODE_RADAR = WB_NODE_RADAR, wb_NODE_RANGE_FINDER = WB_NODE_RANGE_FINDER, wb_NODE_RECEIVER = WB_NODE_RECEIVER,
          wb_NODE_ROTATIONAL_MOTOR = WB_NODE_ROTATIONAL_MOTOR, wb_NODE_SKIN = WB_NODE_SKIN, wb_NODE_SPEAKER = WB_NODE_SPEAKER,
          wb_NODE_TOUCH_SENSOR = WB_NODE_TOUCH_SENSOR, wb_NODE_VACUUM_GRIPPER = WB_NODE_VACUUM_GRIPPER;
// misc
const int wb_NODE_BALL_JOINT = WB_NODE_BALL_JOINT, wb_NODE_BALL_JOINT_PARAMETERS = WB_NODE_BALL_JOINT_PARAMETERS,
          wb_NODE_CHARGER = WB_NODE_CHARGER, wb_NODE_CONTACT_PROPERTIES = WB_NODE_CONTACT_PROPERTIES,
          wb_NODE_DAMPING = WB_NODE_DAMPING, wb_NODE_FLUID = WB_NODE_FLUID, wb_NODE_FOCUS = WB_NODE_FOCUS,
          wb_NODE_HINGE_JOINT = WB_NODE_HINGE_JOINT, wb_NODE_HINGE_JOINT_PARAMETERS = WB_NODE_HINGE_JOINT_PARAMETERS,
          wb_NODE_HINGE_2_JOINT = WB_NODE_HINGE_2_JOINT, wb_NODE_IMMERSION_PROPERTIES = WB_NODE_IMMERSION_PROPERTIES,
          wb_NODE_JOINT_PARAMETERS = WB_NODE_JOINT_PARAMETERS, wb_NODE_LENS = WB_NODE_LENS,
          wb_NODE_LENS_FLARE = WB_NODE_LENS_FLARE, wb_NODE_PHYSICS = WB_NODE_PHYSICS, wb_NODE_RECOGNITION = WB_NODE_RECOGNITION,
          wb_NODE_SLIDER_JOINT = WB_NODE_SLIDER_JOINT, wb_NODE_SLOT = WB_NODE_SLOT, wb_NODE_SOLID = WB_NODE_SOLID,
          wb_NODE_SOLID_REFERENCE = WB_NODE_SOLID_REFERENCE, wb_NODE_TRACK = WB_NODE_TRACK,
          wb_NODE_TRACK_WHEEL = WB_NODE_TRACK_WHEEL, wb_NODE_WORLD_INFO = WB_NODE_WORLD_INFO, wb_NODE_ZOOM = WB_NODE_ZOOM;
// experimental
const int wb_NODE_MICROPHONE = WB_NODE_MICROPHONE, wb_NODE_RADIO = WB_NODE_RADIO;

const char *wb_node_get_name(WbNodeType t) {
  switch (t) {
    case WB_NODE_ACCELEROMETER:
      return "Accelerometer";
    case WB_NODE_ALTIMETER:
      return "Altimeter";
    case WB_NODE_APPEARANCE:
      return "Appearance";
    case WB_NODE_BACKGROUND:
      return "Background";
    case WB_NODE_BALL_JOINT:
      return "BallJoint";
    case WB_NODE_BALL_JOINT_PARAMETERS:
      return "BallJointParameters";
    case WB_NODE_BILLBOARD:
      return "Billboard";
    case WB_NODE_BOX:
      return "Box";
    case WB_NODE_BRAKE:
      return "Brake";
    case WB_NODE_CAD_SHAPE:
      return "CadShape";
    case WB_NODE_CAMERA:
      return "Camera";
    case WB_NODE_CAPSULE:
      return "Capsule";
    case WB_NODE_CHARGER:
      return "Charger";
    case WB_NODE_COLOR:
      return "Color";
    case WB_NODE_COMPASS:
      return "Compass";
    case WB_NODE_CONE:
      return "Cone";
    case WB_NODE_CONNECTOR:
      return "Connector";
    case WB_NODE_CONTACT_PROPERTIES:
      return "ContactProperties";
    case WB_NODE_COORDINATE:
      return "Coordinate";
    case WB_NODE_CYLINDER:
      return "Cylinder";
    case WB_NODE_DAMPING:
      return "Damping";
    case WB_NODE_DIRECTIONAL_LIGHT:
      return "DirectionalLight";
    case WB_NODE_DISPLAY:
      return "Display";
    case WB_NODE_DISTANCE_SENSOR:
      return "DistanceSensor";
    case WB_NODE_ELEVATION_GRID:
      return "ElevationGrid";
    case WB_NODE_EMITTER:
      return "Emitter";
    case WB_NODE_FLUID:
      return "Fluid";
    case WB_NODE_FOCUS:
      return "Focus";
    case WB_NODE_FOG:
      return "Fog";
    case WB_NODE_GPS:
      return "GPS";
    case WB_NODE_GROUP:
      return "Group";
    case WB_NODE_GYRO:
      return "Gyro";
    case WB_NODE_HINGE_2_JOINT:
      return "Hinge2Joint";
    case WB_NODE_HINGE_JOINT:
      return "HingeJoint";
    case WB_NODE_HINGE_JOINT_PARAMETERS:
      return "HingeJointParameters";
    case WB_NODE_IMAGE_TEXTURE:
      return "ImageTexture";
    case WB_NODE_IMMERSION_PROPERTIES:
      return "ImmersionProperties";
    case WB_NODE_INDEXED_FACE_SET:
      return "IndexedFaceSet";
    case WB_NODE_INDEXED_LINE_SET:
      return "IndexedLineSet";
    case WB_NODE_INERTIAL_UNIT:
      return "InertialUnit";
    case WB_NODE_JOINT_PARAMETERS:
      return "JointParameters";
    case WB_NODE_LED:
      return "LED";
    case WB_NODE_LENS:
      return "Lens";
    case WB_NODE_LENS_FLARE:
      return "LensFlare";
    case WB_NODE_LIDAR:
      return "Lidar";
    case WB_NODE_LIGHT_SENSOR:
      return "LightSensor";
    case WB_NODE_LINEAR_MOTOR:
      return "LinearMotor";
    case WB_NODE_MATERIAL:
      return "Material";
    case WB_NODE_MESH:
      return "Mesh";
    case WB_NODE_MICROPHONE:
      return "Microphone";
    case WB_NODE_MUSCLE:
      return "Muscle";
    case WB_NODE_NORMAL:
      return "Normal";
    case WB_NODE_PBR_APPEARANCE:
      return "PBRAppearance";
    case WB_NODE_PEN:
      return "Pen";
    case WB_NODE_PHYSICS:
      return "Physics";
    case WB_NODE_PLANE:
      return "Plane";
    case WB_NODE_POINT_LIGHT:
      return "PointLight";
    case WB_NODE_POINT_SET:
      return "PointSet";
    case WB_NODE_POSITION_SENSOR:
      return "PositionSensor";
    case WB_NODE_POSE:
      return "Pose";
    case WB_NODE_PROPELLER:
      return "Propeller";
    case WB_NODE_RADAR:
      return "Radar";
    case WB_NODE_RADIO:
      return "Radio";
    case WB_NODE_RANGE_FINDER:
      return "RangeFinder";
    case WB_NODE_RECEIVER:
      return "Receiver";
    case WB_NODE_RECOGNITION:
      return "Recognition";
    case WB_NODE_ROBOT:
      return "Robot";
    case WB_NODE_ROTATIONAL_MOTOR:
      return "RotationalMotor";
    case WB_NODE_SHAPE:
      return "Shape";
    case WB_NODE_SKIN:
      return "Skin";
    case WB_NODE_SLIDER_JOINT:
      return "SliderJoint";
    case WB_NODE_SLOT:
      return "Slot";
    case WB_NODE_SOLID:
      return "Solid";
    case WB_NODE_SOLID_REFERENCE:
      return "SolidReference";
    case WB_NODE_SPEAKER:
      return "Speaker";
    case WB_NODE_SPHERE:
      return "Sphere";
    case WB_NODE_SPOT_LIGHT:
      return "SpotLight";
    case WB_NODE_TEXTURE_COORDINATE:
      return "TextureCoordinate";
    case WB_NODE_TEXTURE_TRANSFORM:
      return "TextureTransform";
    case WB_NODE_TOUCH_SENSOR:
      return "TouchSensor";
    case WB_NODE_TRACK:
      return "Track";
    case WB_NODE_TRACK_WHEEL:
      return "TrackWheel";
    case WB_NODE_TRANSFORM:
      return "Transform";
    case WB_NODE_VACUUM_GRIPPER:
      return "VacuumGripper";
    case WB_NODE_VIEWPOINT:
      return "Viewpoint";
    case WB_NODE_WORLD_INFO:
      return "WorldInfo";
    case WB_NODE_ZOOM:
      return "Zoom";
    default:
      return 0;
  }
}
