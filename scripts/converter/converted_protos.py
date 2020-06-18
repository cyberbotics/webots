#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
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

'''List of protos converted to the FLU orientation.'''

_r1 = [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196]

converted_protos = {'Book': [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196],
                    'BunchOfSunFlowers': [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196],
                    'CardboardBox': [0.577350269189626, 0.577350269189626, 0.577350269189626, 2.094395102393196],
                    'CircleArena': [0, 0.7071067811865476, 0.7071067811865476, 3.1415927],
                    'E-puck': _r1,
                    'Floor': [0, 1, 0, 1.5708],
                    'Nao': [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196],
                    'Pedestrian': [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196],
                    'Pioneer3dx': [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196],
                    'RectangleArena': _r1,
                    'WoodenBox': [0.577350269189626, -0.577350269189626, -0.577350269189626, -2.094395102393196]}
