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

from controller import Supervisor
import ctypes
import os
import sys


class Driver(Supervisor):
    INDICATOR_OFF = 0
    INDICATOR_RIGHT = 1
    INDICATOR_LEFT = 2
    SPEED = 0
    TORQUE = 1
    # wiper modes
    DOWN = 0
    SLOW = 1
    NORMAL = 2
    FAST = 3

    api = None

    @staticmethod
    def loadApi() -> ctypes.cdll:
        if sys.platform == 'linux' or sys.platform == 'linux2':
            path = os.path.join('lib', 'controller')
            car = 'libcar.so'
            driver = 'libdriver.so'
        elif sys.platform == 'win32':
            path = os.path.join('lib', 'controller')
            car = 'car.dll'
            driver = 'driver.dll'
        elif sys.platform == 'darwin':
            path = os.path.join('Contents', 'lib', 'controller')
            car = 'libcar.dylib'
            driver = 'libdriver.dylib'
        ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], path, car))
        Driver.api = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], path, driver))

    def __init__(self):
        super().__init__()
        if not Driver.api:
            Driver.loadApi()
        Driver.api.wbu_driver_get_brake_intensity.restype = ctypes.c_double
        Driver.api.wbu_driver_get_current_speed.restype = ctypes.c_double
        Driver.api.wbu_driver_get_rpm.restype = ctypes.c_double
        Driver.api.wbu_driver_get_steering_angle.restype = ctypes.c_double
        Driver.api.wbu_driver_get_target_cruising_speed.restype = ctypes.c_double
        Driver.api.wbu_driver_get_throttle.restype = ctypes.c_double
        Driver.api.wbu_driver_init()

    def __del__(self):
        Driver.api.wbu_driver_cleanup()
        super().__del__()

    def getAntifogLights(self) -> bool:
        return self.antifog_lights

    def getBrakeIntensity(self) -> float:
        return self.brake_intensity

    def getControlMode(self) -> int:
        return self.control_mode

    def getCurrentSpeed(self) -> float:
        return self.current_speed

    def getDippedBeams(self) -> bool:
        return self.dipped_beams

    def getGear(self):
        return self.gear

    def getGearNumber(self):
        return self.gear_number

    def getHazardFlashers(self) -> bool:
        return self.hazard_flashers

    def getIndicator(self) -> int:
        return self.indicator

    def getRpm(self) -> float:
        return self.rpm

    def getSteeringAngle(self) -> float:
        return self.steering_angle

    def getTargetCruisingSpeed(self) -> float:
        return self.target_cruising_speed

    def getThrottle(self) -> float:
        return self.throttle

    def getWiperMode(self) -> int:
        return Driver.api.wbu_driver_get_wiper_mode()

    def setAntifogLights(self, state: bool):
        return self.antifog_lights

    def setBrakeIntensity(self, brakeIntensity: float):
        self.brake_intensity = brakeIntensity

    def setCruisingSpeed(self, cruisingSpeed: float):
        self.target_cruising_speed = cruisingSpeed

    def setDippedBeams(self, state: bool):
        self.dipped_beams = state

    def setGear(self, gear):
        self.gear = gear

    def setHazardFlashers(self, hazardFlasher: bool):
        self.hazard_flashers = hazardFlasher

    def setIndicator(self, indicator: int):
        self.indicator = indicator

    def setSteeringAngle(self, steeringAngle: float):
        self.steering_angle = steeringAngle

    def setThrottle(self, throttle: float):
        self.throttle = throttle

    def setWiperMode(self, mode: int):
        self.wiper_mode = mode

    def step(self):
        return Driver.api.wbu_driver_step()

    @property
    def antifog_lights(self) -> bool:
        return Driver.api.wbu_driver_get_antifog_lights() != 0

    @antifog_lights.setter
    def antifog_lights(self, antifog_lights: bool):
        Driver.api.wbu_driver_set_antifog_lights(1 if antifog_lights else 0)

    @property
    def brake_intensity(self) -> float:
        return Driver.api.wbu_driver_get_brake_intensity()

    @brake_intensity.setter
    def brake_intensity(self, brake_intensity: float):
        Driver.api.wbu_driver_set_brake_intensity(ctypes.c_double(brake_intensity))

    @property
    def control_mode(self) -> int:
        m = Driver.api.wbu_driver_get_control_mode()
        return None if m == -1 else m

    @property
    def dipped_beams(self) -> bool:
        return Driver.api.wbu_driver_get_dipped_beams() != 0

    @dipped_beams.setter
    def dipped_beams(self, dipped_beams: bool):
        Driver.api.wbu_driver_set_dipped_beams(1 if dipped_beams else 0)

    @property
    def gear(self) -> int:
        return Driver.api.wbu_driver_get_gear()

    @gear.setter
    def gear(self, gear: int):
        Driver.api.wbu_driver_set_gear(gear)

    @property
    def gear_number(self) -> int:
        return Driver.api.wbu_driver_get_gear_number()

    @property
    def hazard_flashers(self) -> bool:
        return Driver.api.wbu_driver_get_hazard_flashers() != 0

    @hazard_flashers.setter
    def hazard_flashers(self, hazard_flashers: bool):
        Driver.api.wbu_driver_set_hazard_flashers(1 if hazard_flashers else 0)

    @property
    def indicator(self) -> int:
        return Driver.api.wbu_driver_get_indicator()

    @indicator.setter
    def indicator(self, indicator: int):
        Driver.api.wb_driver_set_indicator(indicator)

    @property
    def rpm(self) -> float:
        return Driver.api.wbu_driver_get_rpm()

    @property
    def steering_angle(self) -> float:
        return Driver.api.wbu_driver_get_steering_angle()

    @steering_angle.setter
    def steering_angle(self, value: float):
        Driver.api.wbu_driver_set_steering_angle(ctypes.c_double(value))

    @property
    def current_speed(self) -> float:
        return Driver.api.wbu_driver_get_current_speed()

    @property
    def target_cruising_speed(self) -> float:
        return Driver.api.wbu_driver_get_target_cruising_speed()

    @target_cruising_speed.setter
    def target_cruising_speed(self, target_cruising_speed: float):
        Driver.api.wbu_driver_set_cruising_speed(ctypes.c_double(target_cruising_speed))

    @property
    def throttle(self) -> float:
        return Driver.api.wbu_driver_get_throttle()

    @throttle.setter
    def throttle(self, throttle: float):
        Driver.api.wbu_driver_set_throttle(ctypes.c_double(throttle))

    @property
    def wiper_mode(self) -> int:
        return Driver.api.wbu_driver_get_wiper_mode()

    @wiper_mode.setter
    def wiper_mode(self, mode):
        Driver.api.wbu_driver_set_wiper_mode(mode)

    # private function for webots_ros2 to identify robots that can use libdriver
    @staticmethod
    def isInitialisationPossible() -> bool:
        if not Driver.api:
            Driver.loadApi()
        Driver.api.wbu_driver_initialization_is_possible.restype = ctypes.c_bool
        return Driver.api.wbu_driver_initialization_is_possible()
