# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes
from .wb import wb
from .constants import constant
from .field import Field
from .proto import Proto
import struct
import typing


class ContactPoint:
    def __init__(self, point):
        self.point = point[0:3]
        self.node_id = point[3]

    def getPoint(self):
        return self.point

    def getNodeId(self):
        return self.node_id


class Node:
    pass


class Node:
    wb.wb_supervisor_node_get_root.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_selected.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_from_def.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_proto.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_field.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_field_by_index.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_base_node_field.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_base_node_field_by_index.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_self.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_from_device.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_from_id.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_parent_node.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_from_proto_def.restype = ctypes.c_void_p
    wb.wb_supervisor_node_get_type_name.restype = ctypes.c_char_p
    wb.wb_supervisor_node_get_base_type_name.restype = ctypes.c_char_p
    wb.wb_supervisor_node_export_string.restype = ctypes.c_char_p
    wb.wb_supervisor_node_get_def.restype = ctypes.c_char_p
    wb.wb_supervisor_node_get_position.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_node_get_orientation.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_node_get_pose.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_node_get_center_of_mass.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_node_get_contact_points.restype = ctypes.POINTER(ctypes.c_ubyte)
    wb.wb_supervisor_node_get_velocity.restype = ctypes.POINTER(ctypes.c_double)

    def __init__(self, DEF: typing.Optional[str] = None, tag: typing.Optional[int] = None, id: typing.Optional[int] = None,
                 selected: typing.Optional[bool] = None, ref: typing.Optional[int] = None):
        if ref is None:
            if id is None:
                if tag is None:
                    if DEF is None:
                        if selected is None:
                            ref = wb.wb_supervisor_node_get_root()
                        else:
                            ref = wb.wb_supervisor_node_get_selected()
                    else:
                        ref = wb.wb_supervisor_node_get_from_def(str.encode(DEF))
                else:
                    if tag == 0:
                        ref = wb.wb_supervisor_node_get_self()
                    else:
                        ref = wb.wb_supervisor_node_get_from_device(tag)
            else:
                ref = wb.wb_supervisor_node_get_from_id(id)
        self._ref = ctypes.c_void_p(ref)

    def getDef(self) -> str:
        return self.DEF

    def getId(self) -> int:
        return self.id

    def getParentNode(self) -> Node:
        node = wb.wb_supervisor_node_get_parent_node(self._ref)
        return Node(ref=node) if node else None

    def isProto(self) -> bool:
        return wb.wb_supervisor_node_is_proto(self._ref) != 0

    def getProto(self) -> Proto:
        proto = wb.wb_supervisor_node_get_proto(self._ref)
        return Proto(proto) if proto else None

    def getFromProtoDef(self, DEF: str) -> Node:
        node = wb.wb_supervisor_node_get_from_proto_def(self._ref, str.encode(DEF))
        return Node(ref=node) if node else None

    def getType(self) -> int:
        return self.type

    def getTypeName(self) -> str:
        return self.type_name

    def getBaseTypeName(self) -> str:
        return self.base_type_name

    def remove(self):
        wb.wb_supervisor_node_remove(self._ref)

    def exportString(self):
        return wb.wb_supervisor_node_export_string(self._ref).decode()

    def getField(self, fieldName: str) -> Field:
        field = wb.wb_supervisor_node_get_field(self._ref, str.encode(fieldName))
        return Field(field) if field else None

    def getFieldByIndex(self, index: int) -> Field:
        field = wb.wb_supervisor_node_get_field_by_index(self._ref, index)
        return Field(field) if field else None

    def getNumberOfFields(self) -> int:
        return self.number_of_fields

    def getBaseNodeField(self, fieldName: str) -> Field:
        field = wb.wb_supervisor_node_get_base_node_field(self._ref, str.encode(fieldName))
        return Field(field) if field else None

    def getBaseNodeFieldByIndex(self, index: int) -> Field:
        field = wb.wb_supervisor_node_get_base_node_field_by_index(self._ref, index)
        return Field(field) if field else None

    def getNumberOfBaseNodeFields(self) -> int:
        return self.number_of_base_node_fields

    def getPosition(self) -> typing.List[float]:
        p = wb.wb_supervisor_node_get_position(self._ref)
        return [p[0], p[1], p[2]]

    def getOrientation(self) -> typing.List[float]:
        o = wb.wb_supervisor_node_get_orientation(self._ref)
        return [o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7], o[8]]

    def getPose(self, fromNode: Node = None) -> typing.List[float]:
        fromNodeRef = fromNode._ref if fromNode else None
        p = wb.wb_supervisor_node_get_pose(self._ref, fromNodeRef)
        return [p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]]

    def enablePoseTracking(self, samplingPeriod: int, fromNode: Node = None):
        fromNodeRef = fromNode._ref if fromNode else None
        wb.wb_supervisor_node_enable_pose_tracking(self._ref, samplingPeriod, fromNodeRef)

    def disablePoseTracking(self, fromNode: Node = None):
        fromNodeRef = fromNode._ref if fromNode else None
        wb.wb_supervisor_node_disable_pose_tracking(self._ref, fromNodeRef)

    def getCenterOfMass(self) -> typing.List[float]:
        c = wb.wb_supervisor_node_get_center_of_mass(self._ref)
        return [c[0], c[1], c[2]]

    def getContactPoints(self, includeDescendants: bool = False) -> typing.List[ContactPoint]:
        size = ctypes.c_int(0)
        p = wb.wb_supervisor_node_get_contact_points(self._ref, 1 if includeDescendants else 0, ctypes.byref(size))
        format_size = 32  # the C compiler adds some padding to the struct, so that its actual size is not 28 as it seems
        points = bytes(p[0:format_size * size.value])
        contact_points = []
        for i in range(size.value):
            contact_points.append(ContactPoint(struct.unpack_from('3di', points, format_size * i)))
        return contact_points

    def enableContactPointsTracking(self, samplingPeriod: int, includeDescendants: bool = False):
        wb.wb_supervisor_node_enable_contact_points_tracking(self._ref, samplingPeriod, 1 if includeDescendants else 0)

    def disableContactPointsTracking(self, includeDescendants: bool = False):
        # includeDescendants is kept for backwards compatibility, but should not be used in new code
        wb.wb_supervisor_node_disable_contact_points_tracking(self._ref)

    def getStaticBalance(self) -> bool:
        return wb.wb_supervisor_node_get_static_balance(self._ref) != 0

    def getVelocity(self) -> typing.List[float]:
        v = wb.wb_supervisor_node_get_velocity(self._ref)
        return [v[0], v[1], v[2], v[3], v[4], v[5]]

    def setVelocity(self, velocity: typing.List[float]):
        wb.wb_supervisor_node_set_velocity(self._ref, (ctypes.c_double * 6)(*velocity))

    def saveState(self, stateName: str):
        wb.wb_supervisor_node_save_state(self._ref, str.encode(stateName))

    def loadState(self, stateName: str):
        wb.wb_supervisor_node_load_state(self._ref, str.encode(stateName))

    def resetPhysics(self):
        wb.wb_supervisor_node_reset_physics(self._ref)

    def setJointPosition(self, position: float, index: int = 1):
        wb.wb_supervisor_node_set_joint_position(self._ref, ctypes.c_double(position), index)

    def restartController(self):
        wb.wb_supervisor_node_restart_controller(self._ref)

    def moveViewpoint(self):
        wb.wb_supervisor_node_move_viewpoint(self._ref)

    def setVisibility(self, fromNode: Node, visible: bool):
        wb.wb_supervisor_node_set_visibility(self._ref, fromNode._ref, 1 if visible else 0)

    def addForce(self, force: typing.List[float], relative: bool):
        wb.wb_supervisor_node_add_force(self._ref, (ctypes.c_double * 3)(*force), 1 if relative else 0)

    def addForceWithOffset(self, force: typing.List[float], offset: typing.List[float], relative: bool):
        wb.wb_supervisor_node_add_force(self._ref, (ctypes.c_double * 3)(*force), (ctypes.c_double * 3)(*offset),
                                        1 if relative else 0)

    def addTorque(self, torque: typing.List[float], relative: bool):
        wb.wb_supervisor_node_add_force(self._ref, (ctypes.c_double * 4)(*torque), 1 if relative else 0)

    @property
    def DEF(self) -> str:
        return wb.wb_supervisor_node_get_def(self._ref).decode()

    @property
    def id(self) -> int:
        return wb.wb_supervisor_node_get_id(self._ref)

    @property
    def type(self) -> int:
        return wb.wb_supervisor_node_get_type(self._ref)

    @property
    def type_name(self) -> str:
        return wb.wb_supervisor_node_get_type_name(self._ref).decode()

    @property
    def base_type_name(self) -> str:
        return wb.wb_supervisor_node_get_base_type_name(self._ref).decode()

    @property
    def number_of_fields(self) -> int:
        return wb.wb_supervisor_node_get_number_of_fields(self._ref)

    @property
    def number_of_base_node_fields(self) -> int:
        return wb.wb_supervisor_node_get_number_of_base_node_fields(self._ref)


Node.NO_NODE = constant('NODE_NO_NODE')
Node.APPEARANCE = constant('NODE_APPEARANCE')
Node.BACKGROUND = constant('NODE_BACKGROUND')
Node.BILLBOARD = constant('NODE_BILLBOARD')
Node.BOX = constant('NODE_BOX')
Node.CAD_SHAPE = constant('NODE_CAD_SHAPE')
Node.CAPSULE = constant('NODE_CAPSULE')
Node.COLOR = constant('NODE_COLOR')
Node.CONE = constant('NODE_CONE')
Node.COORDINATE = constant('NODE_COORDINATE')
Node.CYLINDER = constant('NODE_CYLINDER')
Node.DIRECTIONAL_LIGHT = constant('NODE_DIRECTIONAL_LIGHT')
Node.ELEVATION_GRID = constant('NODE_ELEVATION_GRID')
Node.FOG = constant('NODE_FOG')
Node.GROUP = constant('NODE_GROUP')
Node.IMAGE_TEXTURE = constant('NODE_IMAGE_TEXTURE')
Node.INDEXED_FACE_SET = constant('NODE_INDEXED_FACE_SET')
Node.INDEXED_LINE_SET = constant('NODE_INDEXED_LINE_SET')
Node.MATERIAL = constant('NODE_MATERIAL')
Node.MESH = constant('NODE_MESH')
Node.MUSCLE = constant('NODE_MUSCLE')
Node.NORMAL = constant('NODE_NORMAL')
Node.PBR_APPEARANCE = constant('NODE_PBR_APPEARANCE')
Node.PLANE = constant('NODE_PLANE')
Node.POINT_LIGHT = constant('NODE_POINT_LIGHT')
Node.POINT_SET = constant('NODE_POINT_SET')
Node.SHAPE = constant('NODE_SHAPE')
Node.SPHERE = constant('NODE_SPHERE')
Node.SPOT_LIGHT = constant('NODE_SPOT_LIGHT')
Node.TEXTURE_COORDINATE = constant('NODE_TEXTURE_COORDINATE')
Node.TEXTURE_TRANSFORM = constant('NODE_TEXTURE_TRANSFORM')
Node.TRANSFORM = constant('NODE_TRANSFORM')
Node.VIEWPOINT = constant('NODE_VIEWPOINT')
Node.ROBOT = constant('NODE_ROBOT')
Node.ACCELEROMETER = constant('NODE_ACCELEROMETER')
Node.ALTIMETER = constant('NODE_ALTIMETER')
Node.BRAKE = constant('NODE_BRAKE')
Node.CAMERA = constant('NODE_CAMERA')
Node.COMPASS = constant('NODE_COMPASS')
Node.CONNECTOR = constant('NODE_CONNECTOR')
Node.DISPLAY = constant('NODE_DISPLAY')
Node.DISTANCE_SENSOR = constant('NODE_DISTANCE_SENSOR')
Node.EMITTER = constant('NODE_EMITTER')
Node.GPS = constant('NODE_GPS')
Node.GYRO = constant('NODE_GYRO')
Node.INERTIAL_UNIT = constant('NODE_INERTIAL_UNIT')
Node.LED = constant('NODE_LED')
Node.LIDAR = constant('NODE_LIDAR')
Node.LIGHT_SENSOR = constant('NODE_LIGHT_SENSOR')
Node.LINEAR_MOTOR = constant('NODE_LINEAR_MOTOR')
Node.PEN = constant('NODE_PEN')
Node.POSITION_SENSOR = constant('NODE_POSITION_SENSOR')
Node.RADAR = constant('NODE_RADAR')
Node.RANGE_FINDER = constant('NODE_RANGE_FINDER')
Node.RECEIVER = constant('NODE_RECEIVER')
Node.ROTATIONAL_MOTOR = constant('NODE_ROTATIONAL_MOTOR')
Node.SKIN = constant('NODE_SKIN')
Node.SPEAKER = constant('NODE_SPEAKER')
Node.TOUCH_SENSOR = constant('NODE_TOUCH_SENSOR')
Node.VACUUM_GRIPPER = constant('NODE_VACUUM_GRIPPER')
Node.BALL_JOINT = constant('NODE_BALL_JOINT')
Node.BALL_JOINT_PARAMETERS = constant('NODE_BALL_JOINT_PARAMETERS')
Node.CHARGER = constant('NODE_CHARGER')
Node.CONTACT_PROPERTIES = constant('NODE_CONTACT_PROPERTIES')
Node.DAMPING = constant('NODE_DAMPING')
Node.FLUID = constant('NODE_FLUID')
Node.FOCUS = constant('NODE_FOCUS')
Node.HINGE_JOINT = constant('NODE_HINGE_JOINT')
Node.HINGE_JOINT_PARAMETERS = constant('NODE_HINGE_JOINT_PARAMETERS')
Node.HINGE_2_JOINT = constant('NODE_HINGE_2_JOINT')
Node.IMMERSION_PROPERTIES = constant('NODE_IMMERSION_PROPERTIES')
Node.JOINT_PARAMETERS = constant('NODE_JOINT_PARAMETERS')
Node.LENS = constant('NODE_LENS')
Node.LENS_FLARE = constant('NODE_LENS_FLARE')
Node.PHYSICS = constant('NODE_PHYSICS')
Node.RECOGNITION = constant('NODE_RECOGNITION')
Node.SLIDER_JOINT = constant('NODE_SLIDER_JOINT')
Node.SLOT = constant('NODE_SLOT')
Node.SOLID = constant('NODE_SOLID')
Node.SOLID_REFERENCE = constant('NODE_SOLID_REFERENCE')
Node.TRACK = constant('NODE_TRACK')
Node.TRACK_WHEEL = constant('NODE_TRACK_WHEEL')
Node.WORLD_INFO = constant('NODE_WORLD_INFO')
Node.ZOOM = constant('NODE_ZOOM')
Node.MICROPHONE = constant('NODE_MICROPHONE')
Node.RADIO = constant('NODE_RADIO')

wb.wb_supervisor_field_get_mf_node.restype = ctypes.c_void_p
wb.wb_supervisor_field_get_sf_node.restype = ctypes.c_void_p


def getSFNode(self) -> Node:
    node = wb.wb_supervisor_field_get_sf_node(self._ref)
    return Node(ref=node) if node else None


def getMFNode(self, index: int) -> Node:
    node = wb.wb_supervisor_field_get_mf_node(self._ref, index)
    return Node(ref=node) if node else None


Field.getSFNode = getSFNode
Field.getMFNode = getMFNode
