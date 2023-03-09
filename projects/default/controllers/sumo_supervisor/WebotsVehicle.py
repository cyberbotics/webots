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

"""WebotsVehicle class (used to close the loop with SUMO)."""

import math


class WebotsVehicle:
    """Class that defines a vehicle controlled by Webots."""

    def __init__(self, node, id):
        self.previousPosition = [0, 0, 0]
        self.vehicleLength = 6
        self.vehicleHeight = 0.4
        self.node = node
        self.name = "webotsVehicle%d" % id

    def get_position(self, xOffset, yOffset):
        """Get the position of the vehicle in SUMO coordinate frame."""
        # get position
        position = self.node.getPosition()
        matrix = self.node.getOrientation()
        angle1 = math.asin(matrix[0])
        angle2 = math.atan(-matrix[3] / abs(matrix[0]))
        if angle2 < 0:
            angle1 = math.pi - angle1
        # compute position in sumo coordinate frame
        position[0] = position[0] - xOffset + 0.5 * self.vehicleLength * math.sin(angle1)
        position[1] = position[1] - yOffset - 0.5 * self.vehicleLength * math.cos(angle1)
        position[2] = position[2] - self.vehicleHeight
        return position

    def get_angle(self):
        """Get the angle of the vehicle in SUMO coordinate frame."""
        # get pitch angle
        matrix = self.node.getOrientation()
        angle1 = math.asin(matrix[0])
        angle2 = math.atan(-matrix[3] / abs(matrix[0]))
        angle = angle1
        if angle2 > 0:
            angle = math.pi - angle1
        return angle

    def is_on_road(self, xOffset, yOffset, maxDistance, net):
        """Check if the vehicle is on any of the SUMO roads."""
        # get position
        self.currentPosition = self.get_position(xOffset, yOffset)
        # get pitch angle
        self.angle = self.get_angle()

        # get all the edges in a radius of 5 meters from the vehicle position
        edges = net.getNeighboringEdges(self.currentPosition[0], self.currentPosition[1], maxDistance, False)
        # remove edges starting by ':' (internal edge of SUMO)
        for i in range(len(edges) - 1, -1, -1):
            if (edges[i][0]).getID().startswith(":"):
                edges.pop([i])

        # find the closest edge
        if edges:
            # correct distance using the third dimension
            for i in range(0, len(edges)):
                edge = (edges[i][0]).getID()
                # get height of this edge
                height = 0.0
                tags = edge.split('_')
                del tags[0]  # remove first which is the 'id' of the edge
                for tag in tags:
                    if tag.startswith('height'):
                        height = float(tag.split('height', 1)[1])
                newDist = math.sqrt(math.pow(edges[i][1], 2) + math.pow(self.currentPosition[2] - height, 2))
                edges[i] = (edges[i][0], newDist)

            # sort edges by distances to get the closest one
            self.currentDistancesToEdges = sorted([(dist, currentEdge) for currentEdge, dist in edges])
            if self.currentDistancesToEdges[0][0] < maxDistance:
                return True
        return False

    def update_position(self, time, net, traci, sumolib, xOffset, yOffset):
        """Update the vehicle position in SUMO."""
        # get position
        self.currentPosition = self.get_position(xOffset, yOffset)
        # get pitch angle
        self.angle = self.get_angle()
        # compute current speed and convert it to m/s
        speed = math.sqrt(math.pow(self.currentPosition[0] - self.previousPosition[0], 2) +
                          math.pow(self.currentPosition[1] - self.previousPosition[1], 2))
        self.previousPosition = self.currentPosition
        speed = speed / 0.2
        # if vehicle is not present in the network add it
        if self.name not in traci.vehicle.getIDList():
            try:
                traci.vehicle.add(vehID=self.name, routeID=traci.route.getIDList()[0])
                traci.vehicle.setColor(self.name, (0, 255, 0))
            except Exception:
                pass
        try:
            traci.vehicle.setSpeed(self.name, speed)
        except Exception:
            pass
        try:
            traci.vehicle.moveToXY(vehID=self.name, edgeID='', lane=0, x=self.currentPosition[0], y=self.currentPosition[1],
                                   angle=180 * self.angle / math.pi, keepRoute=0)
        except Exception:
            pass
        self.previousPosition = self.currentPosition
