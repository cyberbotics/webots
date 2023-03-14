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

"""Temporary script to generate ground and forest. Will be removed before merge."""

vehicleCounter = 0
initialVehicleNumber = 300
vehicleInsertionInterval = 0.8
simulationTime = 3600

busRatio = 16
motorcycleRatio = 10
truckRatio = 40
trailerRatio = 15

print('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"'
      ' xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">')
print('  <route id="0" edges="-1"/>')
print('  <route id="1" edges="-0"/>')
print('  <vType id="car" minGap="5"/>')
print('  <vType id="bus" accel="0.8" decel="4.5" sigma="0.8" length="10" minGap="8" maxSpeed="20"'
      ' guiShape="bus" vClass="bus"/>')
print('  <vType id="motorcycle" accel="1.2" decel="4.5" sigma="0.5" length="3" minGap="1.5" maxSpeed="25"'
      ' guiShape="motorcycle" vClass="motorcycle"/>')
print('  <vType id="trailer" accel="0.8" decel="4.5" sigma="0.8" length="14" minGap="8" maxSpeed="20"'
      ' guiShape="truck" vClass="trailer"/>')
print('  <vType id="truck" accel="0.8" decel="4.5" sigma="0.5" length="8" minGap="8" maxSpeed="25"'
      ' guiShape="truck" vClass="truck"/>')
for i in range(initialVehicleNumber // 2):
    departPos = 5000.0 - 2 * i * 5000.0 / initialVehicleNumber
    if vehicleCounter % truckRatio == 0:
        print('  <vehicle id="%d" type="truck" route="1" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    elif vehicleCounter % busRatio == 0:
        print('  <vehicle id="%d" type="bus" route="1" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    elif vehicleCounter % trailerRatio == 0:
        print('  <vehicle id="%d" type="trailer" route="1" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    elif vehicleCounter % motorcycleRatio == 0:
        print('  <vehicle id="%d" type="motorcycle" route="1" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>'
              % (vehicleCounter, departPos))
    else:
        print('  <vehicle id="%d" type="car" route="1" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    vehicleCounter += 1
for i in range(initialVehicleNumber // 2):
    departPos = 5000.0 - 2 * i * 5000.0 / initialVehicleNumber
    if vehicleCounter % truckRatio == 0:
        print('  <vehicle id="%d" type="truck" route="0" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    elif vehicleCounter % busRatio == 0:
        print('  <vehicle id="%d" type="bus" route="0" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    elif vehicleCounter % trailerRatio == 0:
        print('  <vehicle id="%d" type="trailer" route="0" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    elif vehicleCounter % motorcycleRatio == 0:
        print('  <vehicle id="%d" type="motorcycle" route="0" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>'
              % (vehicleCounter, departPos))
    else:
        print('  <vehicle id="%d" type="car" route="0" depart="0" departLane="free" departPos="%lf" departSpeed="max"/>' %
              (vehicleCounter, departPos))
    vehicleCounter += 1
for i in range(int(simulationTime // vehicleInsertionInterval)):
    if vehicleCounter % truckRatio == 0:
        print('  <vehicle id="%d" type="truck" route="%d" depart="%lf" departLane="random"/>' %
              (vehicleCounter, vehicleCounter % 2, i * vehicleInsertionInterval))
    elif vehicleCounter % busRatio == 0:
        print('  <vehicle id="%d" type="bus" route="%d" depart="%lf" departLane="random"/>' %
              (vehicleCounter, vehicleCounter % 2, i * vehicleInsertionInterval))
    elif vehicleCounter % trailerRatio == 0:
        print('  <vehicle id="%d" type="trailer" route="%d" depart="%lf" departLane="random"/>' %
              (vehicleCounter, vehicleCounter % 2, i * vehicleInsertionInterval))
    elif vehicleCounter % motorcycleRatio == 0:
        print('  <vehicle id="%d" type="motorcycle" route="%d" depart="%lf" departLane="random"/>' %
              (vehicleCounter, vehicleCounter % 2, i * vehicleInsertionInterval))
    else:
        print('  <vehicle id="%d" type="car" route="%d" depart="%lf" departLane="random"/>' %
              (vehicleCounter, vehicleCounter % 2, i * vehicleInsertionInterval))
    vehicleCounter += 1

print('</routes>')
