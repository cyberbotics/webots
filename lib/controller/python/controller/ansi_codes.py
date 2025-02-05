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

from .constants import constant


class AnsiCodes:
    RESET = constant('ANSI_RESET', type=str)
    BOLD = constant('ANSI_BOLD', type=str)
    UNDERLINE = constant('ANSI_UNDERLINE', type=str)
    BLACK_BACKGROUND = constant('ANSI_BLACK_BACKGROUND', type=str)
    RED_BACKGROUND = constant('ANSI_RED_BACKGROUND', type=str)
    GREEN_BACKGROUND = constant('ANSI_GREEN_BACKGROUND', type=str)
    YELLOW_BACKGROUND = constant('ANSI_YELLOW_BACKGROUND', type=str)
    BLUE_BACKGROUND = constant('ANSI_BLUE_BACKGROUND', type=str)
    MAGENTA_BACKGROUND = constant('ANSI_MAGENTA_BACKGROUND', type=str)
    CYAN_BACKGROUND = constant('ANSI_CYAN_BACKGROUND', type=str)
    WHITE_BACKGROUND = constant('ANSI_WHITE_BACKGROUND', type=str)
    BLACK_FOREGROUND = constant('ANSI_BLACK_FOREGROUND', type=str)
    RED_FOREGROUND = constant('ANSI_RED_FOREGROUND', type=str)
    GREEN_FOREGROUND = constant('ANSI_GREEN_FOREGROUND', type=str)
    YELLOW_FOREGROUND = constant('ANSI_YELLOW_FOREGROUND', type=str)
    BLUE_FOREGROUND = constant('ANSI_BLUE_FOREGROUND', type=str)
    MAGENTA_FOREGROUND = constant('ANSI_MAGENTA_FOREGROUND', type=str)
    CYAN_FOREGROUND = constant('ANSI_CYAN_FOREGROUND', type=str)
    WHITE_FOREGROUND = constant('ANSI_WHITE_FOREGROUND', type=str)
    CLEAR_SCREEN = constant('ANSI_CLEAR_SCREEN', type=str)
