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

from controller.wb import wb
from controller.node import Node
from controller.robot import Robot


class Supervisor(Robot):
    def __init__(self):
        super().__init__()

    def getFromDef(self, d: str):
        return Node(d)

    def simulationQuit(self, status: int) -> None:
        wb.wb_supervisor_simulation_quit(status)
