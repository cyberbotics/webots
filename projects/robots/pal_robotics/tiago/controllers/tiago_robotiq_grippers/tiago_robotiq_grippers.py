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

"""tiago_robotiq_grippers controller."""
from controller import Robot


def is_motor_velocity_control_mode(name):
    return name in ("front::left finger joint", "wheel_right_joint", "wheel_left_joint")


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_names = ["torso_lift_joint",
               "arm_1_joint",
               "arm_2_joint",
               "arm_3_joint",
               "arm_4_joint",
               "arm_6_joint",
               "front::left finger joint",  # gripper motors are coupled, only left one is directly controlled
               "wheel_right_joint",
               "wheel_left_joint"]

GRIPPER_VELOCITY = [0.4, 0.4] if robot.getName() == "TIAGo 2F-85" else [0.3, 0]
MOVE_FORWARDS_STEPS = 120 if robot.getName() == "TIAGo 2F-85" else 100
CLOSE_GRIPPER_STEPS = 30 if robot.getName() == "TIAGo 2F-85" else 50

target_positions = [[0.105, 0.07, 0, 0, 0, 0, 0, 0, 0],
                    [0.105, 0.331, 0.0337, -1.575, 1.403, 0, 0, 0, 0],
                    [0.105, 0.331, 0.0337, -1.575, 1.403, 0, 0, 1.0, 1.0],
                    [0.105, 0.331, 0.0337, -1.575, 1.403, 0, GRIPPER_VELOCITY[0], 0, 0],
                    [0.105, 0.331, 0.0337, -2.237, 1.403, -0.5, GRIPPER_VELOCITY[1], 0, 0],
                    [0.105, 0.331, 0.0337, -1.65, 1.403, 0, GRIPPER_VELOCITY[1], 0, 0],
                    [0.105, 0.331, 0.0337, -1.65, 1.403, 0, -GRIPPER_VELOCITY[0], 0, 0],
                    [0.105, 0.331, 0.0337, -2.237, 1.403, 0, 0, 0, 0, 0]]
target_steps = [0, 60, 50, MOVE_FORWARDS_STEPS, CLOSE_GRIPPER_STEPS, 70, 50, 50, 50]


for motor_name in motor_names:
    if is_motor_velocity_control_mode(motor_name):
        robot.getDevice(motor_name).setPosition(float('inf'))

# Main loop:
target_idx = 0
step = 0
while robot.step(timestep) != -1 and target_idx < len(target_positions):
    if step < target_steps[target_idx]:
        step = step + 1
        continue

    for idx, motor_name in enumerate(motor_names):
        motor = robot.getDevice(motor_name)
        if is_motor_velocity_control_mode(motor_name):
            motor.setVelocity(target_positions[target_idx][idx])
        else:
            motor.setPosition(target_positions[target_idx][idx])

    target_idx = target_idx + 1
    step = 0
