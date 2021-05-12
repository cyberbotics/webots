# Copyright 1996-2021 Cyberbotics Ltd.
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


"""jetbot_collect_data controller."""

# Create training dataset of images in "free" and "blocked" categories.
# After the dataset of "free" and "blocked" images is created, please run the `jetbot_train` on a terminal.
# A `best_model.pth` file will be created that can be copied in the `jetbot_collision_avoidance` controller folder.
#
# The code is taken from the Jupyter notebooks at:
# https://github.com/NVIDIA-AI-IOT/jetbot/tree/master/notebooks/collision_avoidance

import os
import os.path

from controller import Supervisor
import jetbot_train

# create the Robot instance.
robot = Supervisor()

# print instructions
print('Collect data to build the training dataset for AI collision avoidance classifier\n'
      'Manually move the robot to the desired position and press:\n'
      '- "F": if the robot can safely move forward\n'
      '- "B": if the robot should turn\n'
      'Datasets images will be automatically stored in two different folders named "free" and "blocked".'
      'At least 20 images per category are needed.\n\n'
      'When dataset is ready, press "C" to compute the model.'
      'Copy the resulting "best_model.pth" file in in the "jetbot_collision_avoidance" controller directory to use the model.')

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera')
camera.enable(timestep)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Main loop
index = 1
while robot.step(4 * timestep) != -1:
    key = keyboard.getKey()
    dir_name = 'dataset/'
    if not os.path.isdir(dir_name):
        os.mkdir(dir_name)
    if key == ord('F'):
        dir_name += 'free'
    elif key == ord('B'):
        dir_name += 'blocked'
    elif key == ord('C'):
        print('Start training model.\nThis could take a while...')
        robot.step(timestep)
        jetbot_train.train()
        print('Trained model ready.')
        continue
    else:
        continue

    if not os.path.isdir(dir_name):
        os.mkdir(dir_name)

    path = os.path.join(dir_name, 'image' + str(index) + '.jpg')
    camera.saveImage(path, 100)
    print(path)
    index += 1
