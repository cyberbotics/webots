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

"""SumoSupervisor class inheriting from Supervisor."""

from controller import Supervisor, Node
from Objects import Vehicle, TrafficLight
from WebotsVehicle import WebotsVehicle

import os
import sys
import math

hiddenPosition = 10000


def rotation_from_yaw_pitch_roll(yaw, pitch, roll):
    """Compute the axis-angle rotation from the yaw pitch roll angles"""
    rotation = [0, 0, 1, 0]
    # construct rotation matrix
    # a b c
    # d e f
    # g h i
    a = math.cos(roll) * math.cos(yaw)
    b = -math.sin(roll)
    c = math.cos(roll) * math.sin(yaw)
    d = math.sin(roll) * math.cos(yaw) * math.cos(pitch) + math.sin(yaw) * math.sin(pitch)
    e = math.cos(roll) * math.cos(pitch)
    f = math.sin(roll) * math.sin(yaw) * math.cos(pitch) - math.cos(yaw) * math.sin(pitch)
    g = math.sin(roll) * math.cos(yaw) * math.sin(pitch) - math.sin(yaw) * math.cos(pitch)
    h = math.cos(roll) * math.sin(pitch)
    i = math.sin(roll) * math.sin(yaw) * math.sin(pitch) + math.cos(yaw) * math.cos(pitch)
    # convert it to rotation vector
    cosAngle = 0.5 * (a + e + i - 1.0)
    if math.fabs(cosAngle) > 1:
        return rotation
    else:
        rotation[0] = b - d
        rotation[1] = f - h
        rotation[2] = g - c
        rotation[3] = math.acos(cosAngle)
        # normalize vector
        length = math.sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2])
        if length != 0:
            rotation[0] = rotation[0] / length
            rotation[1] = rotation[1] / length
            rotation[2] = rotation[2] / length
        if rotation[0] == 0 and rotation[1] == 0 and rotation[2] == 0:
            return [0, 0, 1, 0]
        else:
            return rotation


class SumoSupervisor (Supervisor):
    """This is the main class that implements the actual interface."""

    def get_viewpoint_position_field(self):
        """Look for the 'position' field of the Viewpoint node."""
        children = self.getRoot().getField('children')
        number = children.getCount()
        for i in range(0, number):
            node = children.getMFNode(i)
            if node.getType() == Node.VIEWPOINT:
                return node.getField('position')
        return None

    def get_initial_vehicles(self):
        """Get all the vehicles (both controlled by SUMO and Webots) already present in the world."""
        for i in range(0, self.vehiclesLimit):
            defName = "SUMO_VEHICLE%d" % self.vehicleNumber
            node = self.getFromDef(defName)
            if node:
                self.vehicles[i] = Vehicle(node)
                self.vehicles[i].name.setSFString("SUMO vehicle %i" % self.vehicleNumber)
                self.vehicleNumber += 1
            else:
                break
        for i in range(0, self.vehiclesLimit):
            defName = "WEBOTS_VEHICLE%d" % self.webotsVehicleNumber
            node = self.getFromDef(defName)
            if node:
                self.webotsVehicles[i] = WebotsVehicle(node, self.webotsVehicleNumber)
                self.webotsVehicleNumber += 1
            else:
                break

    def generate_new_vehicle(self, vehicleClass):
        """Generate and import a new vehicle that will be controlled by SUMO."""
        # load the new vehicle
        vehicleString, defName = Vehicle.generate_vehicle_string(self.vehicleNumber, vehicleClass)
        self.rootChildren.importMFNodeFromString(-1, vehicleString)
        nodeRef = self.getFromDef(defName)
        if nodeRef:
            self.vehicles[self.vehicleNumber] = Vehicle(nodeRef)
            self.vehicleNumber += 1

    def get_vehicle_index(self, id, generateIfneeded=True):
        """Look for the vehicle index corresponding to this id (and optionnaly create it if required)."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].currentID == id:
                # the vehicle was already here at last step
                return i
        if not generateIfneeded:
            return -1
        # the vehicle was not present last step
        # check if a corresponding vehicle is already in the simulation
        node = self.getFromDef(id)
        if node and (node.getTypeName() in Vehicle.get_car_models_list() or
                     node.getTypeName() in Vehicle.get_bus_models_list() or
                     node.getTypeName() in Vehicle.get_truck_models_list() or
                     node.getTypeName() in Vehicle.get_motorcycle_models_list()):
            self.vehicles[self.vehicleNumber] = Vehicle(node)
            self.vehicles[self.vehicleNumber].currentID = id
            self.vehicleNumber += 1
            return self.vehicleNumber - 1
        # check if a vehicle is available
        vehicleClass = self.get_vehicle_class(id)
        for i in range(0, self.vehicleNumber):
            if not self.vehicles[i].inUse and self.vehicles[i].vehicleClass == vehicleClass:
                # if a vehicle is available assign it to this id
                self.vehicles[i].currentID = id
                self.vehicles[i].name.setSFString(id)
                return i
        # no vehicle available => generate a new one if limit is not reached
        if self.vehicleNumber < self.vehiclesLimit:
            vehicleClass = self.get_vehicle_class(id)
            self.generate_new_vehicle(vehicleClass)
            return self.vehicleNumber - 1
        return -1

    def get_vehicle_class(self, id):
        """Get the class of the vehicle associated to this id."""
        if id in self.vehiclesClass:
            return self.vehiclesClass[id]
        vehicleClass = Vehicle.get_corresponding_vehicle_class(self.traci.vehicle.getVehicleClass(id))
        self.vehiclesClass[id] = vehicleClass
        return vehicleClass

    def disable_unused_vehicles(self, IdList):
        """Check for all the vehicles currently used if they need to be disabled."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].inUse and self.vehicles[i].currentID not in IdList:
                self.vehicles[i].inUse = False
                self.vehicles[i].name.setSFString("SUMO vehicle %i" % i)
                self.vehicles[i].currentLane = None
                self.vehicles[i].currentRoad = None
                self.vehicles[i].laneChangeStartTime = None
                self.vehicles[i].laneChangeDistance = 0

    def hide_unused_vehicles(self):
        """Hide all the newly unused vehicles."""
        for i in range(0, self.vehicleNumber):
            if not self.vehicles[i].inUse:
                if self.vehicles[i].targetPos[0] != hiddenPosition:
                    self.vehicles[i].targetPos = [hiddenPosition, i * 10, 0.5]
                    self.vehicles[i].currentPos = [hiddenPosition, i * 10, 0.5]
                    self.vehicles[i].currentRot = [0, 0, 1, 0]
                    self.vehicles[i].targetRot = [0, 0, 1, 0]
                    self.vehicles[i].currentAngles = [0, 0, 0]
                    self.vehicles[i].targetAngles = [0, 0, 0]
                    self.vehicles[i].translation.setSFVec3f([hiddenPosition, i * 10, 0.5])
                    self.vehicles[i].node.setVelocity([0, 0, 0, 0, 0, 0])
                    for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                        wheelAngularVelocity.setSFVec3f([0, 0, 0])

    def stop_all_vehicles(self):
        """Stop all the vehicles (to be called when controller exits)."""
        for i in range(0, self.vehicleNumber):
            self.vehicles[i].node.setVelocity([0, 0, 0, 0, 0, 0])
            for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                wheelAngularVelocity.setSFVec3f([0, 0, 0])

    def get_vehicles_position(self, id, subscriptionResult, step, xOffset, yOffset,
                              maximumLateralSpeed, maximumAngularSpeed, laneChangeDelay):
        """Compute the new desired position and orientation for all the vehicles controlled by SUMO."""
        if not subscriptionResult:
            return
        height = 0.4
        roll = 0.0
        pitch = 0.0
        sumoPos = subscriptionResult[self.traci.constants.VAR_POSITION]
        sumoAngle = subscriptionResult[self.traci.constants.VAR_ANGLE]
        pos = [sumoPos[0] + xOffset, sumoPos[1] + yOffset, height]
        angle = -math.pi * sumoAngle / 180
        dx = math.cos(angle)
        dz = -math.sin(angle)
        yaw = -math.atan2(dx, dz)
        # correct position (origin of the car is not the same in Webots / sumo)
        vehicleLength = subscriptionResult[self.traci.constants.VAR_LENGTH]
        pos[0] += 0.5 * vehicleLength * math.sin(angle)
        pos[1] -= 0.5 * vehicleLength * math.cos(angle)
        # if needed check the vehicle is in the visibility radius
        if self.radius > 0:
            viewpointPosition = self.viewpointPosition.getSFVec3f()
            xDiff = viewpointPosition[0] - pos[0]
            yDiff = viewpointPosition[1] - pos[1]
            zDiff = viewpointPosition[2]
            distance = math.sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff)
            if distance > self.radius:
                index = self.get_vehicle_index(id, generateIfneeded=False)
                if index >= 0:
                    self.vehicles[index].inUse = False
                    self.vehicles[index].currentID = ""
                    self.vehicles[index].name.setSFString("SUMO vehicle %i" % index)
                return
        index = self.get_vehicle_index(id)
        if index >= 0:
            vehicle = self.vehicles[index]
            height = vehicle.wheelRadius
            if self.enableHeight:
                roadID = subscriptionResult[self.traci.constants.VAR_ROAD_ID]
                roadPos = subscriptionResult[self.traci.constants.VAR_LANEPOSITION]
                if roadID.startswith(':'):
                    # this is a lane change it does not contains edge information
                    # in that case, use previous height, roll and pitch
                    height = vehicle.currentPos[2]
                    roll = vehicle.roll
                    pitch = vehicle.pitch
                else:
                    tags = roadID.split('_')
                    del tags[0]  # remove the first one which is the 'id' of the road
                    for tag in tags:
                        if tag.startswith('height'):
                            height = height + float(tag.split('height', 1)[1])
                        elif tag.startswith('roll'):
                            roll = float(tag.split('roll', 1)[1])
                        elif tag.startswith('pitch'):
                            pitch = float(tag.split('pitch', 1)[1])
                    vehicle.pitch = pitch
                    vehicle.roll = roll
                    # ajust height according to the pitch
                    if pitch != 0:
                        height += (roadPos - 0.5 * vehicleLength) * math.sin(pitch)
                    # ajust height according to the roll and lateral position of the vehicle
                    if roll != 0.0:
                        laneIndex = subscriptionResult[self.traci.constants.VAR_LANE_INDEX]
                        laneID = subscriptionResult[self.traci.constants.VAR_LANE_ID]
                        laneWidth = self.traci.lane.getWidth(laneID)
                        edge = self.net.getEdge(roadID)
                        numberOfLane = edge.getLaneNumber()
                        # compute lateral distance from the center of the lane
                        distance = math.fabs((laneIndex - numberOfLane / 2) + 0.5) * laneWidth
                        if laneIndex >= (numberOfLane / 2):
                            height = height - distance * math.sin(roll)
                        else:
                            height = height + distance * math.sin(roll)
            pos[2] = height
            if vehicle.inUse:
                # TODO: once the lane change model of SUMO has been improved
                #       (sub-lane model currently in development phase) we will be able to remove this corrections

                # compute longitudinal (x) and lateral (y) displacement
                diffX = pos[0] - vehicle.targetPos[0]
                diffY = pos[1] - vehicle.targetPos[1]
                x1 = math.cos(-angle) * diffX - math.sin(-angle) * diffY
                y1 = math.sin(-angle) * diffX + math.cos(-angle) * diffY
                # check for lane change
                if (vehicle.currentRoad is not None and
                        vehicle.currentRoad == subscriptionResult[self.traci.constants.VAR_ROAD_ID] and
                        vehicle.currentLane is not None and
                        vehicle.currentLane != subscriptionResult[self.traci.constants.VAR_LANE_INDEX]):
                    vehicle.laneChangeStartTime = self.getTime()
                    vehicle.laneChangeDistance = x1
                x2 = x1
                # artificially add an angle depending on the lateral speed
                artificialAngle = 0
                if y1 > 0.0001:  # don't add the angle if speed is very small as atan2(0.0, 0.0) is unstable
                    # the '0.15' factor was found empirically and should not depend on the simulation
                    artificialAngle = 0.15 * math.atan2(x1, y1)
                if (vehicle.laneChangeStartTime is not None and
                        vehicle.laneChangeStartTime > self.getTime() - laneChangeDelay):  # lane change case
                    ratio = (self.getTime() - vehicle.laneChangeStartTime) / laneChangeDelay
                    ratio = (0.5 + 0.5 * math.sin((ratio - 0.5) * math.pi))
                    p = vehicle.laneChangeDistance * ratio
                    x2 = x1 - (vehicle.laneChangeDistance - p)
                    artificialAngle = math.atan2(-x2, y1)
                # limit lateral speed
                threshold = 0.001 * step * maximumLateralSpeed
                x2 = min(max(x2, -threshold), threshold)
                x3 = math.cos(angle) * x2 - math.sin(angle) * y1
                y3 = math.sin(angle) * x2 + math.cos(angle) * y1
                pos = [x3 + vehicle.targetPos[0], y3 + vehicle.targetPos[1], pos[2]]
                diffYaw = yaw - vehicle.targetAngles[2] - artificialAngle
                # limit angular speed
                diffYaw = (diffYaw + 2 * math.pi) % (2 * math.pi)
                if (diffYaw > math.pi):
                    diffYaw -= 2 * math.pi
                threshold = 0.001 * step * maximumAngularSpeed
                diffYaw = min(max(diffYaw, -threshold), threshold)
                yaw = diffYaw + vehicle.targetAngles[2]
                # tilt motorcycle depending on the angluar speed
                if vehicle.type in Vehicle.get_motorcycle_models_list():
                    threshold = 0.001 * step * maximumLateralSpeed
                    roll -= min(max(diffYaw / (0.001 * step), -0.2), 0.2)
            rot = rotation_from_yaw_pitch_roll(yaw, pitch, roll)
            if not vehicle.inUse:
                # this vehicle was previously not used, move it directly to the correct initial location
                vehicle.inUse = True
                vehicle.currentPos = pos
                vehicle.currentRot = rot
                vehicle.currentAngles = [roll, pitch, yaw]
            else:
                vehicle.currentPos = vehicle.targetPos
                vehicle.currentRot = vehicle.targetRot
                vehicle.currentAngles = vehicle.targetAngles
            # update target and wheels speed
            vehicle.targetPos = pos
            vehicle.targetRot = rot
            vehicle.targetAngles = [roll, pitch, yaw]
            if self.traci.constants.VAR_SPEED in subscriptionResult:
                vehicle.speed = subscriptionResult[self.traci.constants.VAR_SPEED]
            vehicle.currentRoad = subscriptionResult[self.traci.constants.VAR_ROAD_ID]
            vehicle.currentLane = subscriptionResult[self.traci.constants.VAR_LANE_INDEX]

    def update_vehicles_position_and_velocity(self, step, rotateWheels):
        """Update the actual position (using angular and linear velocities) of all the vehicles in Webots."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].inUse:
                self.vehicles[i].translation.setSFVec3f(self.vehicles[i].currentPos)
                self.vehicles[i].rotation.setSFRotation(self.vehicles[i].currentRot)
                velocity = []
                velocity.append(self.vehicles[i].targetPos[0] - self.vehicles[i].currentPos[0])
                velocity.append(self.vehicles[i].targetPos[1] - self.vehicles[i].currentPos[1])
                velocity.append(self.vehicles[i].targetPos[2] - self.vehicles[i].currentPos[2])
                for j in range(0, 3):
                    diffAngle = self.vehicles[i].currentAngles[j] - self.vehicles[i].targetAngles[j]
                    diffAngle = (diffAngle + 2 * math.pi) % (2 * math.pi)
                    if (diffAngle > math.pi):
                        diffAngle -= 2 * math.pi
                    velocity.append(diffAngle)
                velocity[:] = [1000 * x / step for x in velocity]
                self.vehicles[i].node.setVelocity(velocity)
                if rotateWheels:
                    angularVelocity = [0, self.vehicles[i].speed / self.vehicles[i].wheelRadius, 0]
                    for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                        wheelAngularVelocity.setSFVec3f(angularVelocity)

    def update_webots_vehicles(self, xOffset, yOffset):
        """Update the position of all the vehicles controlled by Webots in SUMO."""
        for i in range(0, self.webotsVehicleNumber):
            if self.webotsVehicles[i].is_on_road(xOffset, yOffset, self.maxWebotsVehicleDistanceToLane, self.net):
                self.webotsVehicles[i].update_position(self.getTime(), self.net, self.traci, self.sumolib, xOffset, yOffset)
            else:
                # the controlled vehicle is not on any road
                # => we remove it from sumo network
                if self.webotsVehicles[i].name in self.traci.vehicle.getIDList():
                    self.traci.vehicle.remove(self.webotsVehicles[i].name)

    def get_traffic_light(self, IDlist):
        """Get the state of all the traffic lights controlled by SUMO."""
        self.trafficLightNumber = len(IDlist)
        self.trafficLights = {}
        LEDNames = []
        for i in range(0, self.getNumberOfDevices()):
            device = self.getDeviceByIndex(i)
            if device.getNodeType() == Node.LED:
                LEDNames.append(device.getName())
        for i in range(0, self.trafficLightNumber):
            id = IDlist[i]
            self.trafficLights[id] = TrafficLight()
            self.trafficLights[id].lightNumber = len(self.traci.trafficlight.getRedYellowGreenState(id))
            for j in range(0, self.trafficLights[id].lightNumber):
                trafficLightNode = self.getFromDef("TLS_" + id + "_" + str(j))
                if trafficLightNode is not None:
                    self.trafficLights[id].trafficLightRecognitionColors[j] = trafficLightNode.getField('recognitionColors')
                ledName = id + "_" + str(j) + "_"
                if ledName + 'r' in LEDNames:
                    self.trafficLights[id].LED[3 * j + 0] = self.getDevice(ledName + 'r')
                else:
                    self.trafficLights[id].LED[3 * j + 0] = None
                if ledName + 'y' in LEDNames:
                    self.trafficLights[id].LED[3 * j + 1] = self.getDevice(ledName + 'y')
                else:
                    self.trafficLights[id].LED[3 * j + 1] = None
                if ledName + 'g' in LEDNames:
                    self.trafficLights[id].LED[3 * j + 2] = self.getDevice(ledName + 'g')
                else:
                    self.trafficLights[id].LED[3 * j + 2] = None

    def update_traffic_light_state(self, id, states):
        """Update the traffic lights state in Webots."""
        # update light LED state if traffic light state has changed
        currentState = states[self.traci.constants.TL_RED_YELLOW_GREEN_STATE]
        if self.trafficLights[id].previousState != currentState:
            self.trafficLights[id].previousState = currentState
            for j in range(0, self.trafficLights[id].lightNumber):
                # Update red LED if it exists
                if self.trafficLights[id].LED[3 * j + 0]:
                    if currentState[j] == 'r' or currentState[j] == 'R':
                        self.trafficLights[id].LED[3 * j + 0].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [1, 0, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 0].set(0)
                # Update yellow LED if it exists
                if self.trafficLights[id].LED[3 * j + 1]:
                    if currentState[j] == 'y' or currentState[j] == 'Y':
                        self.trafficLights[id].LED[3 * j + 1].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [1, 0.5, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 1].set(0)
                # Update green LED if it exists
                if self.trafficLights[id].LED[3 * j + 2]:
                    if currentState[j] == 'g' or currentState[j] == 'G':
                        self.trafficLights[id].LED[3 * j + 2].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [0, 1, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 2].set(0)

    def run(self, port, disableTrafficLight, directory, step, rotateWheels,
            maxVehicles, radius, enableHeight, useDisplay, displayRefreshRate,
            displayZoom, displayFitSize, maximumLateralSpeed, maximumAngularSpeed,
            laneChangeDelay, traci, sumolib):
        """Main loop function."""
        try:
            print('Connect to SUMO... This operation may take a few seconds.')
            self.step(step)
            traci.init(port, numRetries=20)
        except Exception:
            sys.exit('Unable to connect to SUMO, please make sure any previous instance of SUMO is closed.\n You can try'
                     ' changing SUMO port using the "--port" argument.')

        self.traci = traci
        self.sumolib = sumolib
        self.radius = radius
        self.enableHeight = enableHeight
        self.sumoClosed = False
        self.temporaryDirectory = directory
        self.rootChildren = self.getRoot().getField('children')
        self.viewpointPosition = self.get_viewpoint_position_field()
        self.maxWebotsVehicleDistanceToLane = 15
        self.webotsVehicleNumber = 0
        self.webotsVehicles = {}
        self.vehicleNumber = 0
        self.vehicles = {}
        self.vehiclesLimit = maxVehicles
        self.vehiclesClass = {}

        directory = os.path.normpath(directory)

        # for backward compatibility
        if self.traci.constants.TRACI_VERSION <= 15:
            self.traci.trafficlight = self.traci.trafficlights

        # get sumo vehicles already present in the world
        self.get_initial_vehicles()

        # parse the net and get the offsets
        self.net = sumolib.net.readNet(os.path.join(directory, 'sumo.net.xml'))
        xOffset = -self.net.getLocationOffset()[0]
        yOffset = -self.net.getLocationOffset()[1]

        # Load plugin to the generic SUMO Supervisor (if any)
        self.usePlugin = False
        if os.path.exists(os.path.join(directory, 'plugin.py')):
            self.usePlugin = True
            sys.path.append(directory)
            import plugin
            sumoSupervisorPlugin = plugin.SumoSupervisorPlugin(self, self.traci, self.net)

        # Get all the LEDs of the traffic lights
        if not disableTrafficLight:
            trafficLightsList = self.traci.trafficlight.getIDList()
            self.get_traffic_light(trafficLightsList)
            for id in trafficLightsList:
                # subscribe to traffic lights state
                self.traci.trafficlight.subscribe(id, [self.traci.constants.TL_RED_YELLOW_GREEN_STATE])

        # Subscribe to new vehicles entering the simulation
        self.traci.simulation.subscribe([
            self.traci.constants.VAR_DEPARTED_VEHICLES_IDS,
            self.traci.constants.VAR_MIN_EXPECTED_VEHICLES
        ])

        # Create the vehicle variable subscription list
        self.vehicleVariableList = [
            self.traci.constants.VAR_POSITION,
            self.traci.constants.VAR_ANGLE,
            self.traci.constants.VAR_LENGTH,
            self.traci.constants.VAR_ROAD_ID,
            self.traci.constants.VAR_LANE_INDEX
        ]
        if rotateWheels:
            self.vehicleVariableList.append(self.traci.constants.VAR_SPEED)
        if enableHeight:
            self.vehicleVariableList.extend([
                self.traci.constants.VAR_ROAD_ID,
                self.traci.constants.VAR_LANEPOSITION,
                self.traci.constants.VAR_LANE_ID
            ])

        # create the SUMO display
        self.sumoDisplay = None
        if useDisplay:
            view = self.traci.gui.getIDList()[0]
            display = self.getDevice('sumo')
            if display is not None:
                from SumoDisplay import SumoDisplay
                self.sumoDisplay = SumoDisplay(display, displayZoom, view, directory, displayRefreshRate, displayFitSize,
                                               self.traci)

        # Main simulation loop
        while self.step(step) >= 0:
            if self.usePlugin:
                sumoSupervisorPlugin.run(step)

            if self.sumoDisplay is not None:
                self.sumoDisplay.step(step)

            # try to perform a SUMO step, if it fails it means SUMO has been closed by the user
            try:
                self.traci.simulationStep()
            except self.traci.exceptions.FatalTraCIError:
                print("Sumo closed")
                self.sumoClosed = True
                break

            result = self.traci.simulation.getSubscriptionResults()

            # SUMO simulation over (no more vehicle are expected)
            if result[self.traci.constants.VAR_MIN_EXPECTED_VEHICLES] == 0:
                break

            # subscribe to new vehicle
            for id in result[self.traci.constants.VAR_DEPARTED_VEHICLES_IDS]:
                if not id.startswith('webotsVehicle'):
                    self.traci.vehicle.subscribe(id, self.vehicleVariableList)
                elif self.sumoDisplay is not None and len(self.webotsVehicles) == 1:
                    # Only one vehicle controlled by Webots => center the view on it
                    self.traci.gui.trackVehicle(view, 'webotsVehicle0')

            # get result from the vehicle subscription and apply it
            idList = self.traci.vehicle.getIDList()
            for id in idList:
                self.get_vehicles_position(id, self.traci.vehicle.getSubscriptionResults(id),
                                           step, xOffset, yOffset, maximumLateralSpeed, maximumAngularSpeed,
                                           laneChangeDelay)
            self.disable_unused_vehicles(idList)

            # hide unused vehicles
            self.hide_unused_vehicles()

            if not disableTrafficLight:
                for id in self.trafficLights:
                    self.update_traffic_light_state(id, self.traci.trafficlight.getSubscriptionResults(id))

            self.update_vehicles_position_and_velocity(step, rotateWheels)
            self.update_webots_vehicles(xOffset, yOffset)

        if not self.sumoClosed:
            self.traci.close()
        else:
            self.stop_all_vehicles()
        sys.stdout.flush()
