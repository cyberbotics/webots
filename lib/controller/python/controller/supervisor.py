# Copyright 1996-2024 Cyberbotics Ltd.
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

from .wb import wb
from .node import Node
from .robot import Robot
import ctypes
import typing


class Supervisor(Robot):
    def __init__(self):
        super().__init__()

    def getRoot(self) -> Node:
        return Node()

    def getSelf(self) -> Node:
        return Node(tag=0)

    def getFromDef(self, d: str) -> Node:
        node = Node(DEF=d)
        return node if node._ref else None

    def getFromId(self, id) -> Node:
        node = Node(id=id)
        return node if node._ref else None

    def getFromDevice(self, tag) -> Node:
        node = Node(tag=tag)
        return node if node._ref else None

    def getSelected(self) -> Node:
        node = Node(selected=True)
        return node if node._ref else None

    def setLabel(self, id, label, x, y, size, color, transparency=0, font='Arial'):
        wb.wb_supervisor_set_label(id, str.encode(label), ctypes.c_double(x), ctypes.c_double(y), ctypes.c_double(size),
                                   color, ctypes.c_double(transparency), str.encode(font))

    def simulationQuit(self, status: int):
        wb.wb_supervisor_simulation_quit(status)

    def simulationSetMode(self, mode: int):
        self.simulation_mode = mode

    def simulationGetMode(self) -> int:
        return self.simulation_mode

    def simulationReset(self):
        wb.wb_supervisor_simulation_reset()

    def simulationResetPhysics(self):
        wb.wb_supervisor_simulation_reset_physics()

    def worldLoad(self, filename: str):
        wb.wb_supervisor_world_load(str.encode(filename))

    def worldSave(self, filename: str = None) -> int:
        if not filename:
            return wb.wb_supervisor_world_save(None)
        return wb.wb_supervisor_world_save(str.encode(filename))

    def worldReload(self):
        wb.wb_supervisor_world_reload()

    def exportImage(self, filename: str, quality: int):
        wb.wb_supervisor_export_image(str.encode(filename), quality)

    def movieStartRecording(self, filename: str, width: int, height: int, codec: int, quality: int, acceleration: int,
                            caption: bool):
        wb.wb_supervisor_movie_start_recording(str.encode(filename), width, height, codec, quality, acceleration,
                                               1 if caption else 0)

    def movieStopRecording(self):
        wb.wb_supervisor_movie_stop_recording()

    def movieIsReady(self):
        return wb.wb_supervisor_movie_is_ready() != 0

    def movieFailed(self):
        return wb.wb_supervisor_movie_failed() != 0

    def animationStartRecording(self, filename: str):
        return wb.wb_supervisor_animation_start_recording(str.encode(filename))

    def animationStopRecording(self):
        return wb.wb_supervisor_animation_stop_recording()

    def virtualRealityHeadsetIsUsed(self):
        return wb.wb_supervisor_virtual_reality_headset_is_used() != 0

    def virtualRealityHeadsetGetPosition(self) -> typing.List[float]:
        return wb.wb_supervisor_virtual_reality_headset_get_position()

    def virtualRealityHeadsetGetOrientation(self) -> typing.List[float]:
        return wb.wb_supervisor_virtual_reality_headset_get_orientation()

    @property
    def simulation_mode(self) -> int:
        return wb.wb_supervisor_simulation_get_mode()

    @simulation_mode.setter
    def simulation_mode(self, mode: int):
        return wb.wb_supervisor_simulation_set_mode(mode)


Supervisor.SIMULATION_MODE_PAUSE = 0
Supervisor.SIMULATION_MODE_REAL_TIME = 1
Supervisor.SIMULATION_MODE_FAST = 2
