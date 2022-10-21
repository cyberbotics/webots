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
import ctypes


class Supervisor(Robot):
    def __init__(self):
        super().__init__()

    def getRoot(self) -> Node:
        return Node()

    def getSelf(self) -> Node:
        return Node(tag=0)

    def getFromDef(self, d: str) -> Node:
        return Node(DEF=d)

    def getFromId(self, id) -> Node:
        return Node(id=id)

    def getFromDevice(self, tag) -> Node:
        return Node(tag=tag)

    def getSelected(self) -> Node:
        return Node(selected=True)

    def setLabel(self, id, label, x, y, size, color, transparency=0, font='Arial'):
        wb.wb_supervisor_set_label(id, str.encode(label), ctypes.c_double(x), ctypes.c_double(y), ctypes.c_double(size),
                                   color, ctypes.c_double(transparency), str.encode(font))

    def simulationQuit(self, status: int):
        wb.wb_supervisor_simulation_quit(status)
