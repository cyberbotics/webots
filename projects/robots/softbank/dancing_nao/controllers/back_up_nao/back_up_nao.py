"""back_up_nao controller."""

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Adapted from the Example of Python controller for Nao robot.

   This is a demo of dancing nao robots created by Milindi Kodikara
   Sound track - Bee Gees' Staying Alive"""

from controller import Robot, Keyboard, Motion
import time
from datetime import datetime, timedelta


class Nao(Robot):
    PHALANX_MAX = 8

    # load motion files
    def loadMotionFiles(self):
        self.stand = (Motion('../../motions/StandUpFromFront.motion'), 4.2)
        self.fall = (Motion('../../motions/Fall.motion'), 3)
        
        self.handWave = (Motion('../../motions/HandWave.motion'), 2)
        self.handWaveLeft = (Motion('../../motions/HandWaveLeft.motion'), 2)

        self.forwards50 = (Motion('../../motions/Forwards50.motion'), 2.8)
        self.forwards = (Motion('../../motions/Forwards.motion'), 2.8)
        self.backwards = (Motion('../../motions/Backwards.motion'), 2)

        self.sideStepLeft = (Motion('../../motions/SideStepLeft.motion'), 5)
        self.sideStepLeft4 = (Motion('../../motions/SideStepLeft.motion'), 3.5)
        self.sideStepRight = (Motion('../../motions/SideStepRight.motion'), 3.5)

        self.turnLeft60 = (Motion('../../motions/TurnLeft60.motion'), 4.6)
        self.turnRight60 = (Motion('../../motions/TurnRight60.motion'), 4.8)

        self.turnRight40 = (Motion('../../motions/TurnRight40.motion'), 3)
        self.turnLeft40 = (Motion('../../motions/TurnLeft40.motion'), 3)
        
        self.turnLeft10 = (Motion('../../motions/TurnLeft10.motion'), 3)

        self.turnLeft180 = (Motion('../../motions/TurnLeft180.motion'), 9)

        self.shoot = (Motion('../../motions/Shoot.motion'), 4)

        self.handHighRight = (Motion('../../motions/HandHighRight.motion'), 0.5)
        self.handHighLeft = (Motion('../../motions/HandHighLeft.motion'), 0.5)

        self.handDown = (Motion('../../motions/HandDown.motion'), 4)
        self.handsUp = (Motion('../../motions/HandsUp.motion'), 2)
        self.handsUpNoWiggle = (Motion('../../motions/HandsUpNoWiggle.motion'), 2.4)

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False

        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()

    def run(self):

        self.handWave[0].play()

        dance_steps = [self.forwards50, self.forwards50, self.forwards50,
                       self.backwards, self.backwards, self.handWave, self.backwards, self.backwards,
                       self.backwards, self.backwards, self.forwards50, self.forwards50, self.backwards,
                       self.turnLeft180,
                       self.backwards, self.backwards, self.turnLeft60, self.turnLeft40, 
                       self.turnLeft40, self.turnLeft10, self.backwards, self.backwards,
                       self.forwards50, self.handWave, self.forwards50, self.backwards, self.backwards,
                       self.backwards, self.handsUp, self.handsUp, self.handsUp, self.shoot, 
                       self.backwards, self.backwards, self.shoot, self.backwards, self.handsUpNoWiggle]

        dance_step_count = 0

        for dance_step in dance_steps:

            dance_steps[dance_step_count][0].play()
            finish = datetime.now() + timedelta(seconds=dance_steps[dance_step_count][1])
            while finish > datetime.now():
                self.step(self.timeStep)

            dance_step_count = dance_step_count + 1


        # create the Robot instance and run main loop


robot = Nao()
robot.run()

