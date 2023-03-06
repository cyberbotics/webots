# Copyright 1996-2023 Cyberbotics Ltd.
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

from vehicle import Driver
import ctypes
import os
import sys


class Car(Driver):
    # types
    TRACTION = 0
    PROPULSION = 1
    FOUR_BY_FOUR = 2
    # engine types
    COMBUSTION_ENGINE = 0
    ELECTRIC_ENGINE = 1
    PARALLEL_HYBRID_ENGINE = 2
    POWER_SPLIT_HYBRID_ENGINE = 3
    # wheels
    WHEEL_FRONT_RIGHT = 0
    WHEEL_FRONT_LEFT = 1
    WHEEL_REAR_RIGHT = 2
    WHEEL_REAR_LEFT = 3

    def __init__(self):
        super().__init__()
        if sys.platform == 'linux' or sys.platform == 'linux2':
            path = os.path.join('lib', 'controller', 'libcar.so')
        elif sys.platform == 'win32':
            path = os.path.join('lib', 'controller', 'car.dll')
        elif sys.platform == 'darwin':
            path = os.path.join('Contents', 'MacOS', 'lib', 'controller', 'libcar.dylib')
        self.api = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], path))
        self.api.wbu_car_get_front_wheel_radius.restype = ctypes.c_double
        self.api.wbu_car_get_indicator_period.restype = ctypes.c_double
        self.api.wbu_car_get_left_steering_angle.restype = ctypes.c_double
        self.api.wbu_car_get_rear_wheel_radius.restype = ctypes.c_double
        self.api.wbu_car_get_right_steering_angle.restype = ctypes.c_double
        self.api.wbu_car_get_track_front.restype = ctypes.c_double
        self.api.wbu_car_get_track_rear.restype = ctypes.c_double
        self.api.wbu_car_get_wheelbase.restype = ctypes.c_double
        self.api.wbu_car_get_wheel_encoder.restype = ctypes.c_double
        self.api.wbu_car_get_wheel_speed.restype = ctypes.c_double

        self.api.wbu_car_init()

    def __del__(self):
        self.api.wbu_car_cleanup()
        super().__del__()

    def enableIndicatorAutoDisabling(self, enable: bool):
        self.api.wbu_car_enable_indicator_auto_disabling(1 if enable else 0)

    def enableLimitedSlipDifferential(self, enable: bool):
        self.api.wbu_car_enable_limited_slip_differential(1 if enable else 0)

    def getBackwardsLights(self) -> bool:
        return self.backwards_lights

    def getBrakeLights(self) -> bool:
        return self.brake_lights

    def getEngineType(self) -> int:
        return self.engine_type

    def getFrontWheelRadius(self):
        return self.front_wheel_radius

    def getIndicatorPeriod(self) -> float:
        return self.indicator_period

    def getLeftSteeringAngle(self):
        return self.left_steering_angle

    def getRearWheelRadius(self):
        return self.rear_wheel_radius

    def getRightSteeringAngle(self):
        return self.right_steering_angle

    def getTrackFront(self):
        return self.track_front

    def getTrackRear(self):
        return self.track_rear

    def getType(self) -> int:
        return self.type

    def getWheelbase(self):
        return self.wheelbase

    def getWheelEncoder(self, wheel_index):
        return self.api.wbu_car_get_wheel_encoder(wheel_index)

    def getWheelSpeed(self, wheel_index):
        return self.api.wbu_car_get_wheel_speed(wheel_index)

    def setIndicatorPeriod(self, period: float):
        self.indicator_period = period

    def setLeftSteeringAngle(self, angle):
        self.left_steering_angle = angle

    def setRightSteeringAngle(self, angle):
        self.right_steering_angle = angle

    @property
    def backwards_lights(self) -> bool:
        return self.api.wbu_car_get_backwards_lights() != 0

    @property
    def brake_lights(self) -> bool:
        return self.api.wbu_car_get_brake_lights() != 0

    @property
    def engine_type(self) -> int:
        return self.api.wbu_car_get_engine_type()

    @property
    def front_wheel_radius(self) -> float:
        return self.api.wbu_car_get_front_wheel_radius()

    @property
    def indicator_period(self) -> float:
        return self.api.wbu_car_get_indicator_period()

    @indicator_period.setter
    def indicator_period(self, period: float):
        self.api.wbu_car_set_indicator_period(ctypes.c_double(period))

    @property
    def left_steering_angle(self) -> float:
        return self.api.wbu_car_get_left_steering_angle()

    @left_steering_angle.setter
    def left_steering_angle(self, angle: float):
        self.api.wbu_car_set_left_steering_angle(ctypes.c_double(angle))

    @property
    def rear_wheel_radius(self) -> float:
        return self.api.wbu_car_get_rear_wheel_radius()

    @property
    def right_steering_angle(self) -> float:
        return self.api.wbu_car_get_right_steering_angle()

    @right_steering_angle.setter
    def right_steering_angle(self, angle: float):
        self.api.wbu_car_set_right_steering_angle(ctypes.c_double(angle))

    @property
    def track_front(self) -> float:
        return self.api.wbu_car_get_track_front()

    @property
    def track_rear(self) -> float:
        return self.api.wbu_car_get_track_rear()

    @property
    def type(self) -> int:
        return self.api.wbu_car_get_type()

    @property
    def wheelbase(self) -> float:
        return self.api.wbu_car_get_wheelbase()
