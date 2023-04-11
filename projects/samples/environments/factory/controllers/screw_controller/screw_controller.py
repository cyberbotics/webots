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


"""screw_controller controller."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

deviceNames = []
for i in range(robot.getNumberOfDevices()):
    deviceNames.append(robot.getDeviceByIndex(i).getName())

numberOfScrews = 0
motors = []
sensors = []
previousPosition = []
for i in range(robot.getNumberOfDevices()):
    linearMotorName = 'linear motor %d' % i
    positionSensorName = 'position sensor %d' % i
    if linearMotorName in deviceNames and positionSensorName in deviceNames:
        numberOfScrews += 1
        motors.append(robot.getDevice(linearMotorName))
        sensors.append(robot.getDevice(positionSensorName))
        previousPosition.append(0)
    else:
        break

for sensor in sensors:
    sensor.enable(timestep)

while robot.step(timestep) != -1:
    for i in range(numberOfScrews):
        targetPosition = sensors[i].getValue() * 0.001
        maxPosition = motors[i].getMaxPosition()
        minPosition = motors[i].getMinPosition()
        targetPosition = max(min(targetPosition, maxPosition), minPosition)
        if previousPosition[i] != targetPosition:
            previousPosition[i] = targetPosition
            motors[i].setPosition(targetPosition)
