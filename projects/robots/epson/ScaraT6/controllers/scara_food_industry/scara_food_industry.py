# Copyright 1996-2022 Cyberbotics Ltd.
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

from controller import Supervisor
from random import randint

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# actuators
base_arm = supervisor.getDevice("base_arm_motor")
arm = supervisor.getDevice("arm_motor")
shaft_linear = supervisor.getDevice("shaft_linear_motor")
led = supervisor.getDevice("epson_led")

ledStatus = True
merged_tool = False
fruitType = 0


def merge_tool(fruit_id):
    global merged_tool

    if not merged_tool:
        fruit = supervisor.getFromDef("fruit" + str(fruit_id))
        vaccum = supervisor.getFromDef("VACCUM")
        if fruit:
            poses = vaccum.getPosition();
            fruitTranslation = fruit.getField('translation')
            if fruitTranslation and poses:
              fruitTranslation.setSFVec3f([poses[0], poses[1], poses[2] - 0.07])
              fruit.resetPhysics()

def ledAnimation():
    global ledStatus
    led.set(ledStatus)
    ledStatus = not(ledStatus)

i = 0
while supervisor.step(timestep) != -1:
    if i == 0:
        arm.setPosition(0.6)
        base_arm.setPosition(0.2)
        ledAnimation()
    elif i > 450:
        i = -1
    elif i > 400:
        fruitType = randint(0, 1)
    elif i > 180:
        merge_tool(fruitType)
        if i > 250:
            if fruitType:
                base_arm.setPosition(0)
                arm.setPosition(-0.83)
            else:
                base_arm.setPosition(-0.50)
                arm.setPosition(-0.83)
        else :
            shaft_linear.setPosition(0)
            ledAnimation()

    elif i > 150:
            merge_tool(fruitType)
    elif i > 110:
        shaft_linear.setPosition(-0.148)
        ledAnimation()

    i = i + 1
