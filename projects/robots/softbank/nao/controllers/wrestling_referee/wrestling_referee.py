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
        matchDuration = 3 * 60 * 1000  # a match lasts 3 minutes
        timeStep = int(self.getBasicTimeStep())  # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        time = 0
        self.setLabel(0, "Wreslting match", 0.2, 0.4, 0.2, 0xffffff, 0, "Impact")
        while self.step(timeStep) != -1 and time < matchDuration:  # runs in a loop until Webots quits or the match is over
            pass


# create the referee instance and run main loop
referee = Referee()
referee.run()
