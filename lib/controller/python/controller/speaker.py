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

from .wb import wb
from .device import Device
import ctypes
from typing import Union


class Speaker(Device):
    wb.wb_speaker_get_engine.restype = ctypes.c_char_p
    wb.wb_speaker_get_language.restype = ctypes.c_char_p

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def stop(self, sound: Union[str, None] = None):
        wb.wb_speaker_stop(self._tag, None if sound is None else str.encode(sound))

    def isSoundPlaying(self, sound: str) -> bool:
        return wb.wb_speaker_is_sound_playing(self._tag, str.encode(sound)) != 0

    def setEngine(self, engine: str) -> bool:
        return wb.wb_speaker_set_engine(self._tag, str.encode(engine)) != 0

    def setLanguage(self, language: str) -> bool:
        return wb.wb_speaker_set_engine(self._tag, str.encode(language)) != 0

    def getLanguage(self) -> str:
        return wb.wb_speaker_get_language(self._tag).decode()

    def getEngine(self) -> str:
        return wb.wb_speaker_get_engine(self._tag).decode()

    def isSpeaking(self) -> bool:
        return wb.wb_speaker_is_speaking(self._tag) != 0

    def speak(self, text: str, volume: float):
        wb.wb_speaker_speak(self._tag, str.encode(text), ctypes.c_double(volume))


def _playSound(left: Speaker, right: Speaker, sound: str, volume: float, pitch: float, balance: float, loop: bool):
    wb.wb_speaker_play_sound(left._tag, right._tag, str.encode(sound),
                             ctypes.c_double(volume), ctypes.c_double(pitch), ctypes.c_double(balance),
                             1 if loop else 0)


Speaker.playSound = staticmethod(_playSound)
