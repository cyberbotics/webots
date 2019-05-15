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

    def run(self):
        display = self.getDisplay("display")
        matchDuration = 3 * 60 * 1000  # a match lasts 3 minutes
        timeStep = int(self.getBasicTimeStep())  # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        time = 0
        seconds = -1
        background = display.imageLoad("wrestling_board.png")
        display.imagePaste(background, 0, 0, False)
        display.setFont("Arial", 28, True)
        display.drawText("Robot Wrestling", 14, 14)
        display.drawText("Tournament", 54, 62)
        display.setFont("Arial", 53, True)
        while True:
            s = int(time / 1000) % 60
            if seconds != s:
                seconds = s
                minutes = int(time / 60000)
                display.setColor(0x636061)  # grey background
                display.fillRectangle(320, 32, 180, 17)
                display.setColor(0x000000)  # black background
                display.fillRectangle(320, 49, 180, 37)
                display.setColor(0xffffff)
                display.drawText("%02i:%02i" % (minutes, seconds), 320, 32)
            if self.step(timeStep) == -1 or time > matchDuration:  # runs in a loop until Webots quits or the match is over
                break
            time += timeStep


# create the referee instance and run main loop
referee = Referee()
referee.run()
