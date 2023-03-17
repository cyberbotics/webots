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

"""vehicle_driver_altino controller."""

from vehicle import Driver

sensorMax = 1000

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep
front_left_sensor = driver.getDevice('front_left_sensor')
front_center_sensor = driver.getDevice('front_center_sensor')
front_right_sensor = driver.getDevice('front_right_sensor')

headlights = driver.getDevice("headlights")
backlights = driver.getDevice("backlights")

keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

front_left_sensor.enable(sensorTimeStep)
front_center_sensor.enable(sensorTimeStep)
front_right_sensor.enable(sensorTimeStep)

side_left_sensor = driver.getDevice('side_left_sensor')
side_right_sensor = driver.getDevice('side_right_sensor')
back_sensor = driver.getDevice('back_sensor')

side_left_sensor.enable(sensorTimeStep)
side_right_sensor.enable(sensorTimeStep)
back_sensor.enable(sensorTimeStep)

# speed refers to the speed in km/h at which we want Altino to travel
speed = 0
# angle refers to the angle (from straight ahead) that the wheels
# currently have
angle = 0

# This the Altino's maximum speed
# all Altino controllers should use this maximum value
maxSpeed = 1.8
# ensure 0 starting speed and wheel angle
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)
# defaults for this controller
useManual = False
headlightsOn = False

printCounter = 0

while driver.step() != -1:

    # enable backlights for reverse
    speed = driver.getTargetCruisingSpeed()
    if speed < 0:
        backlights.set(1)
    else:
        backlights.set(0)

    while True:
        # handle keyboard input
        currentKey = keyboard.getKey()
        if currentKey == -1:
            break
        elif currentKey == ord('h') or currentKey == ord('H'):
            if not headlightsOn:
                headlights.set(1)
                headlightsOn = True
        elif currentKey == ord('g') or currentKey == ord('G'):
            if headlightsOn:
                headlights.set(0)
                headlightsOn = False
        elif currentKey == ord('m') or currentKey == ord('M'):
            if not useManual:
                useManual = True
        elif currentKey == ord('n') or currentKey == ord('N'):
            if useManual:
                useManual = False
        # handle manual control input
        if useManual:
            if currentKey == keyboard.UP:
                if speed < 0:
                    speed += 0.02
                else:
                    speed += 0.008
            elif currentKey == keyboard.DOWN:
                if speed > 0:
                    speed -= 0.02
                else:
                    speed -= 0.008
            elif currentKey == keyboard.LEFT:
                angle -= 0.01
            elif currentKey == keyboard.RIGHT:
                angle += 0.01
            # Emergency stop key
            elif currentKey == ord(' '):
                speed /= 4
    if not useManual:
        fLeftVal = front_left_sensor.getValue()
        fCenterVal = front_center_sensor.getValue()
        fRightVal = front_right_sensor.getValue()

        sLeftVal = side_left_sensor.getValue()
        sRightVal = side_right_sensor.getValue()
        backVal = back_sensor.getValue()

        # Distance Sensor Values:
        # 1000: 0cm
        # 800:  12cm
        # 600:  24cm
        # 400:  36cm
        # 200:  48cm
        # 0:    60cm

        if fCenterVal > 400 and fCenterVal < 600:
            speed -= (0.01 * speed)
        elif fCenterVal > 600 and fCenterVal < 800:
            speed /= 1.01
        if backVal > 400 and backVal < 600:
            speed /= 1.01
        elif backVal > 600 and backVal < 800:
            speed /= 1.1

        if fLeftVal > fRightVal:
            angle += (fLeftVal - fRightVal) / (300 * sensorMax)
        elif fRightVal > fLeftVal:
            angle -= (fRightVal - fLeftVal) / (300 * sensorMax)
        else:
            angle /= 1.5

        if sLeftVal > 300:
            angle += 0.003
        if sRightVal > 300:
            angle -= 0.003

        speed += 0.001

    # clamp speed and angle to max values
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
    if angle > 0.4:
        angle = 0.4
    elif angle < -0.4:
        angle = -0.4

    if (printCounter % 10) == 0:
        print("Welcome to the Altino Sample Controller")
        print("----------------------------------------------")
        print("This sample controller is based on a Braitenberg vehicle, \n")
        print("it uses the vehicle's infrared distance sensors to avoid obstacles.")
        print("\n-----------------Controls---------------------")
        print("'M' to enable manual control")
        print("'N' to disable manual control")
        print("'H' to turn on the headlights")
        print("'G' to turn off the headlights")
        print("Arrow Keys to accelerate, decelerate and turn")
        print("Space bar to brake (manual mode only)")
        print("----------------------------------------------")
        print("Current Wheel Angle and Throttle values:")
        print("Angle: %.2f" % angle)
        print("Throttle: %.1f " % (100 * speed / maxSpeed))
        if useManual:
            print("---------Manual Control ENABLED-------------")
        else:
            print("---------Manual Control DISABLED------------")
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)
    printCounter += 1
