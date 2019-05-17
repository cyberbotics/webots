# Copyright 1996-2019 Cyberbotics Ltd.
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

"""Referee supervisor controller for the Robot Wrestling Tournament."""

from controller import Supervisor


class Referee (Supervisor):

    def init(self):
        self.digit = [[0] * 10 for i in range(3)]  # create an array of size [3][10] filled in with zeros
        for j in range(3):
            for i in range(10):
                self.digit[j][i] = self.getMotor("digit " + str(j) + str(i))
        self.currentDigit = [0, 0, 0]  # 0:00

    def displayTime(self, minutes, seconds):
        for j in range(3):
            self.digit[j][self.currentDigit[j]].setPosition(1000)  # far away, not visible
        self.currentDigit[0] = minutes
        self.currentDigit[1] = seconds // 10
        self.currentDigit[2] = seconds % 10
        for j in range(3):
            self.digit[j][self.currentDigit[j]].setPosition(0)  # visible

    def run(self):
        matchDuration = 3 * 60 * 1000  # a match lasts 3 minutes
        timeStep = int(self.getBasicTimeStep())  # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        time = 0
        seconds = -1
        while True:
            s = int(time / 1000) % 60
            if seconds != s:
                seconds = s
                minutes = int(time / 60000)
                self.displayTime(minutes, seconds)
            if self.step(timeStep) == -1 or time > matchDuration:  # runs in a loop until Webots quits or the match is over
                break
            time += timeStep


# create the referee instance and run main loop
referee = Referee()
referee.init()
referee.run()
