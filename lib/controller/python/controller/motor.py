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
from .constants import constant
from .device import Device
from .wb import wb
from typing import Union


class Motor(Device):
    wb.wb_motor_get_target_position.restype = ctypes.c_double
    wb.wb_motor_get_max_position.restype = ctypes.c_double
    wb.wb_motor_get_min_position.restype = ctypes.c_double
    wb.wb_motor_get_velocity.restype = ctypes.c_double
    wb.wb_motor_get_max_velocity.restype = ctypes.c_double
    wb.wb_motor_get_acceleration.restype = ctypes.c_double
    wb.wb_motor_get_available_force.restype = ctypes.c_double
    wb.wb_motor_get_available_torque.restype = ctypes.c_double
    wb.wb_motor_get_max_force.restype = ctypes.c_double
    wb.wb_motor_get_max_torque.restype = ctypes.c_double
    wb.wb_motor_get_multiplier.restype = ctypes.c_double
    wb.wb_motor_get_force_feedback.restype = ctypes.c_double
    wb.wb_motor_get_torque_feedback.restype = ctypes.c_double

    ROTATIONAL = constant('ROTATIONAL')
    LINEAR = constant('LINEAR')

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def setPosition(self, position: float):
        self.target_position = position

    def setVelocity(self, velocity: float):
        self.target_velocity = velocity

    def setAcceleration(self, acceleration: float):
        self.target_acceleration = acceleration

    def setAvailableForce(self, force: float):
        self.available_force = force

    def setAvailableTorque(self, torque: float):
        self.available_torque = torque

    def setControlPID(self, p: float, i: float, d: float):
        wb.wb_motor_set_control_pid(self._tag, ctypes.c_double(p), ctypes.c_double(i), ctypes.c_double(d))

    def getTargetPosition(self) -> float:
        return self.target_position

    def getMinPosition(self) -> float:
        return self.min_position

    def getMaxPosition(self) -> float:
        return self.max_position

    def getVelocity(self) -> float:
        return self.target_velocity

    def getMaxVelocity(self) -> float:
        return self.max_velocity

    def getAcceleration(self) -> float:
        return self.target_acceleration

    def getAvailableForce(self) -> float:
        return self.available_force

    def getMaxForce(self) -> float:
        return self.max_force

    def getAvailableTorque(self) -> float:
        return self.available_torque

    def getMaxTorque(self) -> float:
        return self.max_torque

    def getMultiplier(self) -> float:
        return self.multiplier

    def enableForceFeedback(self, sampling_period: int):
        wb.wb_motor_enable_force_feedback(self._tag, sampling_period)

    def disableForceFeedback(self):
        wb.wb_motor_disable_force_feedback(self._tag)

    def getForceFeedbackSamplingPeriod(self) -> int:
        return self.force_feedback_sampling_period

    def getForceFeedback(self) -> float:
        return self.force_feedback

    def enableTorqueFeedback(self, sampling_period: int):
        wb.wb_motor_enable_torque_feedback(self._tag, sampling_period)

    def disableTorqueFeedback(self):
        wb.wb_motor_disable_torque_feedback(self._tag)

    def getTorqueFeedbackSamplingPeriod(self) -> int:
        return self.torque_feedback_sampling_period

    def getTorqueFeedback(self) -> float:
        return self.torque_feedback

    def setForce(self, force: float):
        wb.wb_motor_set_force(self._tag, ctypes.c_double(force))

    def setTorque(self, torque: float):
        wb.wb_motor_set_torque(self._tag, ctypes.c_double(torque))

    def getType(self) -> int:
        return wb.wb_motor_get_type(self._tag)

    def getBrake(self):
        return self.brake

    def getPositionSensor(self):
        return self.position_sensor

    @property
    def brake(self):
        from .brake import Brake
        tag = wb.wb_motor_get_brake(self._tag)
        return None if tag == 0 else Brake(tag)

    @property
    def position_sensor(self):
        from .position_sensor import PositionSensor
        tag = wb.wb_motor_get_position_sensor(self._tag)
        return None if tag == 0 else PositionSensor(tag)

    @property
    def force_feedback_sampling_period(self) -> int:
        return wb.wb_motor_get_force_feedback_sampling_period(self._tag)

    @force_feedback_sampling_period.setter
    def force_feedback_sampling_period(self, sampling_period):
        wb.wb_motor_enable_force_feedback(self._tag, sampling_period)

    @property
    def torque_feedback_sampling_period(self) -> int:
        return wb.wb_motor_get_torque_feedback_sampling_period(self._tag)

    @torque_feedback_sampling_period.setter
    def torque_feedback_sampling_period(self, sampling_period):
        wb.wb_motor_enable_torque_feedback(self._tag, sampling_period)

    @property
    def max_position(self) -> float:
        return wb.wb_motor_get_max_position(self._tag)

    @property
    def min_position(self) -> float:
        return wb.wb_motor_get_min_position(self._tag)

    @property
    def max_velocity(self) -> float:
        return wb.wb_motor_get_max_velocity(self._tag)

    @property
    def target_position(self) -> float:
        return wb.wb_motor_get_target_position(self._tag)

    @target_position.setter
    def target_position(self, position: float):
        wb.wb_motor_set_position(self._tag, ctypes.c_double(position))

    @property
    def target_velocity(self) -> float:
        return wb.wb_motor_get_velocity(self._tag)

    @target_velocity.setter
    def target_velocity(self, velocity: float):
        wb.wb_motor_set_velocity(self._tag, ctypes.c_double(velocity))

    @property
    def available_force(self) -> float:
        return wb.wb_motor_get_available_force(self._tag)

    @available_force.setter
    def available_force(self, force: float):
        wb.wb_motor_set_available_force(self._tag, ctypes.c_double(force))

    @property
    def max_force(self) -> float:
        return wb.wb_motor_get_max_force(self._tag)

    @property
    def available_torque(self) -> float:
        return wb.wb_motor_get_available_torque(self._tag)

    @available_torque.setter
    def available_torque(self, torque: float):
        wb.wb_motor_set_available_torque(self._tag, ctypes.c_double(torque))

    @property
    def max_torque(self) -> float:
        return wb.wb_motor_get_max_torque(self._tag)

    @property
    def target_acceleration(self) -> float:
        return wb.wb_motor_get_acceleration(self._tag)

    @target_acceleration.setter
    def target_acceleration(self, acceleration: float):
        wb.wb_motor_set_acceleration(self._tag, ctypes.c_double(acceleration))

    force = property(fset=setForce)
    torque = property(fset=setTorque)

    @property
    def multiplier(self) -> float:
        return wb.wb_motor_get_multiplier(self._tag)

    @property
    def force_feedback(self) -> float:
        return wb.wb_motor_get_force_feedback(self._tag)

    @property
    def torque_feedback(self) -> float:
        return wb.wb_motor_get_torque_feedback(self._tag)

    @property
    def type(self) -> int:
        return wb.wb_motor_get_type(self._tag)
