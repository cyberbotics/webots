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

"""
This controller reads the lidar middle layer and use the z-axis of the point cloud to determine the number of objects in front
of the robot.
"""

from controller import Robot


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.sick = self.getDevice('sick')
        self.sick.enable(self.timeStep)
        self.sick.enablePointCloud()

    def run(self):
        resolution = self.sick.getHorizontalResolution()
        layers = self.sick.getNumberOfLayers()
        previous_object_counter = 0
        while self.step(self.timeStep) != -1:
            object_counter = 0
            previous_obstacle = False
            # For each point of the middle layer
            layer = self.sick.getLayerPointCloud(int(layers / 2))
            for p in range(resolution):
                point = layer[p]
                # Determine if an obstacle is present or not
                obstacle = point.x < 3.0
                # Each time a new obstacle is detected, then increment the object counter
                if obstacle and not previous_obstacle:
                    object_counter += 1
                previous_obstacle = obstacle
            # Display the result on the Webots console (only when the result has changed)
            if object_counter != previous_object_counter:
                print(f'I see {object_counter} can{"s" if object_counter > 1 else ""}')
                previous_object_counter = object_counter


controller = Controller()
controller.run()
