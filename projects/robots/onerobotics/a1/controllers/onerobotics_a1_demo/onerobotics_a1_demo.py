# Copyright 1996-2025 Cyberbotics Ltd.
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

"""Move the OneRobotics A1 right arm between two conservative poses."""

from controller import Robot


robot = Robot()
timestep = int(robot.getBasicTimeStep())

motors = [robot.getDevice(f"joint{index}-a1_r") for index in range(1, 8)]
for motor in motors:
    motor.setVelocity(0.5)

poses = (
    (0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.6, -1.0, 0.5, 0.6, -0.5, 0.4, 0.7),
)
pose_duration = 4.0
active_pose = -1

while robot.step(timestep) != -1:
    next_pose = int(robot.getTime() / pose_duration) % len(poses)
    if next_pose != active_pose:
        for motor, target in zip(motors, poses[next_pose]):
            motor.setPosition(target)
        active_pose = next_pose
