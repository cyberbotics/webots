# Copyright 1996-2020 Cyberbotics Ltd.
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


"""led_controller controller."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

led = robot.getDevice('led')
led.set(True)

positionSensor = robot.getDevice('emergency button sensor')
positionSensor.enable(timestep)

released = True

while robot.step(timestep) != -1:
    value = positionSensor.getValue()
    if value > -0.002:
        released = True

    if released and value < -0.010:
        released = False
        led.set(not led.get())
