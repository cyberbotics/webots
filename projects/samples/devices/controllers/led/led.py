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
An example of controller using a led device.
"""

from controller import Robot
import random


class Controller(Robot):
    SPEED = 6
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.ds0 = self.getDevice('ds0')
        self.ds1 = self.getDevice('ds1')
        self.led = []
        self.led.append(self.getDevice('led0'))
        self.led.append(self.getDevice('led1'))
        self.led.append(self.getDevice('led2'))
        self.ds0.enable(self.timeStep)
        self.ds1.enable(self.timeStep)
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        led_counter = [0, 0, 0]
        led1_increase = 10
        while self.step(self.timeStep) != -1:
            # For led 0 (right hand side), which is not a gradual LED, we simply
            # select randomly a color and we change its color using it. According
            # to the documentation, when using the color number 0 this turns the
            # led off.
            if led_counter[0] == 0:
                random_color = random.randrange(10)
                self.led[0].set(random_color)
                # We will rechange the color in a random amount of time.
                led_counter[0] = random.randrange(4, 8)
                if random_color == 0:
                    print('LED 0 is turned off')
                else:
                    print('LED 0 uses color ' + str(random_color))
            else:
                led_counter[0] -= 1

            # For led 1 (left hand side), which is a monochromatic gradual LED,
            # we increase and decrease its value, making it glow
            self.led[1].set(led_counter[1])
            led_counter[1] += led1_increase
            if led_counter[1] > 255:
                led_counter[1] = 255
                led1_increase = -10
            elif led_counter[1] < 0:
                led_counter[1] = 0
                led1_increase = 10

            # For led 2 (back side), which is a RGB gradual LED, we set a
            # random value each 1024 ms.
            if led_counter[2] == 0:
                self.led[2].set(random.randrange(0xffffff))
            led_counter[2] += 1
            if led_counter[2] == 16:
                led_counter[2] = 0

            ds0_value = self.ds0.getValue()
            ds1_value = self.ds1.getValue()
            if ds1_value > 500:
                # If both distance sensors are detecting something, this means that
                # we are facing a wall. In this case we need to move backwards.
                if ds0_value > 200:
                    left_speed = -self.SPEED
                    right_speed = -self.SPEED / 2
                else:
                    # We turn proportionnaly to the sensors value because the
                    # closer we are from the wall, the more we need to turn.
                    left_speed = -ds1_value / 100
                    right_speed = (ds0_value / 100) + 0.5
            elif ds0_value > 500:
                left_speed = (ds1_value / 100) + 0.5
                right_speed = -ds0_value / 100
            else:
                # If nothing was detected we can move forward at maximal speed.
                left_speed = self.SPEED
                right_speed = self.SPEED

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
