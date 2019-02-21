# Copyright 1996-2018 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
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

motor = robot.getMotor('linear motor')

positionSensor = robot.getPositionSensor('position sensor')
positionSensor.enable(timestep)

while robot.step(timestep) != -1:
    targetPosition = positionSensor.getValue() * 0.001
    maxPosition = motor.getMaxPosition()
    minPosition = motor.getMinPosition()
    targetPosition = max(min(targetPosition, maxPosition), minPosition)
    motor.setPosition(targetPosition)
