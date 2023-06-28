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

"""Vehicle and TrafficLight classes."""

import math
import random

hiddenPosition = 10000

CAR_MODEL = [
    'BmwX5Simple',
    'CitroenCZeroSimple',
    'ToyotaPriusSimple',
    'LincolnMKZSimple',
    'RangeRoverSportSVRSimple',
    'TeslaModel3Simple',
    'MercedesBenzSprinterSimple'
]

BUS_MODEL = ['BusSimple']

TRUCK_MODEL = ['TruckSimple']

TRAILER_MODEL = [
    'TruckTrailerSimple',
    'TruckTankSimple'
]

MOTORCYCLE_MODEL = [  # Motorcycle is a fixed keyword in SUMO (this is why "two wheelers" is not used here).
    'ScooterSimple',
    'MotorbikeSimple'
]

WHEEL_RADIUS = {
    'BmwX5Simple': 0.374,
    'CitroenCZeroSimple': 0.285,
    'ToyotaPriusSimple': 0.317,
    'LincolnMKZSimple': 0.358,
    'RangeRoverSportSVRSimple': 0.358,
    'TeslaModel3Simple': 0.36,
    'MercedesBenzSprinterSimple': 0.4,
    'BusSimple': 0.56,
    'TruckSimple': 0.5,
    'ScooterSimple': 0.23,
    'MotorbikeSimple': 0.25
}

VEHICLE_COLORS = [
    '0.43 0.11 0.1',
    '0.85 0.85 0.05',
    '0.10 0.15 0.18',
    '0.14 0.29 0.16',
    '0.18 0.28 0.64',
    '0.62 0.62 0.62',
    '0.72 0.52 0.18',
    '0.18 0.50 0.72'
]

COLOR_PAIRS = [  # Good looking pairs of colors for the motorcycle colors
    ['0.43 0.11 0.1', '0.694118 0.435294 0.435294'],
    ['0.0235294 0.352941 0.603922', '0.0784314 0.254902 0.352941'],
    ['0.231373 0.603922 0.592157', '0.223529 0.356863 0.317647'],
    ['0.564706 0.435294 0.054902', '0.458824 0.286275 0.0705882'],
    ['0.141176 0.345098 0.0784314', '0.223529 0.254902 0.164706'],
    ['0.254902 0.0156863 0.105882', '0.309804 0.2 0.235294']
]

PEDESTRIAN_COLORS = [  # Politically correct colors for the pedestrian
    ['0.403922 0.329412 0.254902', '0.215686 0.309804 0.478431', '0.996078 0.764706 0.67451', '0.266667 0.266667 0.266667'],
    ['0.396078 0.396078 0.686275', '0.278431 0.278431 0.278431', '0.686275 0.588235 0.517647', '0.470588 0.117647 0.117647'],
    ['0.329412 0.145098 0.145098', '0.172549 0.172549 0.172549', '0.407843 0.305882 0.219608', '0.137255 0.137255 0.137255'],
    ['0.678431 0.678431 0.678431', '0.235294 0.305882 0.415686', '0.6 0.552941 0.372549', '0.160784 0.160784 0.223529'],
    ['0.596078 0.678431 0.564706', '0.113725 0.137255 0.141176', '0.27451 0.207843 0.168627', '0.121569 0.223529 0.0117647']
]

TRAILER_COLORS = [
    '0.66 0.66 0.66',
    '0.85 0.85 0.85',
    '0.07 0.27 0.12'
]

TRAILER_TEXTURES = [
    'truck_trailer_webots.jpg',
    'truck_trailer_red.jpg'
]


class Vehicle:
    """This class defines a vehicle controlled by SUMO."""

    def __init__(self, node):
        """Initialize and get the required fields from the vehicle node."""
        if node.getTypeName() in ['Solid']:
            self.car_node = node.getField('children').getMFNode(0)
        else:
            self.car_node = node
        self.node = node
        self.translation = self.node.getField("translation")
        self.rotation = self.node.getField("rotation")
        self.name = self.node.getField("name")
        self.inUse = False
        self.currentID = ""
        self.currentPos = [hiddenPosition, 0, 0.5]
        self.currentRot = [0, 0, 1, 0]
        self.currentAngles = [0, 0, 0]
        self.targetPos = [hiddenPosition, 0, 0.5]
        self.targetRot = [0, 0, 1, 0]
        self.currentAngles = [0, 0, 0]
        self.roll = 0
        self.pitch = 0
        self.speed = 0
        self.type = self.car_node.getTypeName()
        self.currentLane = None
        self.currentRoad = None
        self.laneChangeStartTime = None
        self.laneChangeDistance = 0
        self.wheelRadius = WHEEL_RADIUS[self.type]
        if self.type in CAR_MODEL:
            self.vehicleClass = 'car'
        elif self.type in BUS_MODEL:
            self.vehicleClass = 'bus'
        elif self.type in TRUCK_MODEL:
            if self.car_node.getField("trailer").getSFNode() is None:
                self.vehicleClass = 'truck'
            else:
                self.vehicleClass = 'trailer'
        elif self.type in MOTORCYCLE_MODEL:
            self.vehicleClass = 'motorcycle'
        else:
            print("Vehicle type not supported: " + self.type)
        self.wheelsAngularVelocity = []
        if self.type in MOTORCYCLE_MODEL:
            self.wheelsAngularVelocity.append(self.car_node.getField("frontWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(self.car_node.getField("rearWheelAngularVelocity"))
        else:
            self.wheelsAngularVelocity.append(self.car_node.getField("frontRightWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(self.car_node.getField("frontLeftWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(self.car_node.getField("rearRightWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(self.car_node.getField("rearLeftWheelAngularVelocity"))
        if self.vehicleClass == 'trailer':
            trailerNode = self.car_node.getField("trailer").getSFNode()
            self.wheelsAngularVelocity.append(trailerNode.getField("frontLeftWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(trailerNode.getField("frontRightWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(trailerNode.getField("centerLeftWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(trailerNode.getField("centerRightWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(trailerNode.getField("rearLeftWheelAngularVelocity"))
            self.wheelsAngularVelocity.append(trailerNode.getField("rearRightWheelAngularVelocity"))

    @staticmethod
    def generate_vehicle_string(index, vehicleClass):
        """Generate a string representing the node of the corresponding vehicle class."""
        # randomly select the model
        if vehicleClass == 'bus':
            model = BUS_MODEL[0]
            vehicleLength = -4.6
        elif vehicleClass == 'truck' or vehicleClass == 'trailer':
            model = TRUCK_MODEL[0]
            vehicleLength = -5.2
        elif vehicleClass == 'motorcycle':
            modelIndex = math.trunc(random.uniform(0, len(MOTORCYCLE_MODEL)))
            model = MOTORCYCLE_MODEL[modelIndex]
            colorPairs = random.choice(COLOR_PAIRS)
            colorPedestrian = random.choice(PEDESTRIAN_COLORS)
            vehicleLength = -1.7
        else:
            modelIndex = math.trunc(random.uniform(0, len(CAR_MODEL)))
            model = CAR_MODEL[modelIndex]
            vehicleLength = -2.85
        color = random.choice(VEHICLE_COLORS)
        defName = "SUMO_VEHICLE%d" % index
        vehicleString = "DEF " + defName + " Solid" + " {\n"
        vehicleString += "  translation 10000 0 0.5\n children [\n" + model + "{\n"
        vehicleString += "  translation " + str(vehicleLength) + " 0 0\n"
        if vehicleClass == 'motorcycle':
            vehicleString += "  primaryColor " + colorPairs[0] + "\n"
            vehicleString += "  secondaryColor " + colorPairs[1] + "\n"
            vehicleString += "  recognitionColors [ " + colorPairs[0] + ", " + colorPairs[1] + " ]\n"
            if model == "ScooterSimple":
                vehicleString += "  driver ScooterDriver {\n"
            else:
                vehicleString += "  driver MotorbikeDriver {\n"
            vehicleString += "    shirtColor " + colorPedestrian[0] + "\n"
            vehicleString += "    pantsColor " + colorPedestrian[1] + "\n"
            vehicleString += "    skinColor " + colorPedestrian[2] + "\n"
            vehicleString += "    helmetColor " + colorPedestrian[3] + "\n"
            vehicleString += "  }\n"
        else:
            vehicleString += "  color " + color + "\n"
            vehicleString += "  recognitionColors [ " + color + " ]\n"
            if vehicleClass == 'trailer':
                trailerModel = random.choice(TRAILER_MODEL)
                vehicleString += "  trailer " + trailerModel + "{\n"
                if trailerModel == 'TruckTrailerSimple':
                    vehicleString += "    appearance PBRAppearance { metalness 0 roughness 0.4 baseColorMap ImageTexture { "
                    vehicleString += "url [ \"webots://projects/vehicles/protos/generic/textures/" + \
                        random.choice(TRAILER_TEXTURES) + "\" ] } }"
                else:
                    vehicleString += "    color " + random.choice(TRAILER_COLORS) + "\n"
                vehicleString += "  }\n"
            elif vehicleClass == 'truck':
                vehicleString += "  trailer NULL\n"
        vehicleString += "  }\n ]\n}\n"
        return vehicleString, defName

    @staticmethod
    def get_corresponding_vehicle_class(vehicleClass):
        """Return the string matching the vehicle class (SUMO to Webots vehicle class conversion)."""
        if vehicleClass == 'bus':
            return 'bus'
        elif vehicleClass == 'truck':
            return 'truck'
        elif vehicleClass == 'trailer':
            return 'trailer'
        elif vehicleClass == 'motorcycle':
            return 'motorcycle'
        else:
            return 'car'

    @staticmethod
    def get_car_models_list():
        """Get the string list of car models."""
        return CAR_MODEL

    @staticmethod
    def get_bus_models_list():
        """Get the string list of bus models."""
        return BUS_MODEL

    @staticmethod
    def get_truck_models_list():
        """Get the string list of truck models."""
        return TRUCK_MODEL

    @staticmethod
    def get_motorcycle_models_list():
        """Get the string list of motorcycle models."""
        return MOTORCYCLE_MODEL


class TrafficLight:
    """Class that represents a traffic light in Webots."""

    def __init__(self):
        self.lightNumber = 0  # In SUMO a traffic light is a set of real traffic lights
        self.previousState = ""
        self.LED = {}
        self.trafficLightRecognitionColors = {}
