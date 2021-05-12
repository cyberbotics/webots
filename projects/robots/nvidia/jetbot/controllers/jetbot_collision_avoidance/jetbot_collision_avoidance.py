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

"""jetbot_collision_avoidance controller."""

# The code is taken from the Jupyter notebook at
# https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/notebooks/collision_avoidance/live_demo.ipynb

import torch
import torch.nn.functional as F
import torchvision
import cv2
import numpy as np
import os.path

from jetbot_python_control import JetBot

mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])
normalize = torchvision.transforms.Normalize(mean, stdev)


def preprocessCameraImage(camera):
    global device, normalize
    data = camera.getImage()
    x = np.frombuffer(data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    x = cv2.cvtColor(x, cv2.COLOR_BGRA2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]
    return x


if not os.path.isfile('best_model.pth'):
    print('Trained model "best_model.pth" not found, please use the "jetbot_collect_data" controller to generate it.')
    exit()

# Create the Robot instance.
robot = JetBot()

# Set the controller time step
timestep = 5 * int(robot.getBasicTimeStep())

robot.camera.enable(timestep)

robot.step(10 * timestep)

print('Load the trained model..')
model = torchvision.models.alexnet(pretrained=False)
model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 2)
model.load_state_dict(torch.load('best_model.pth'))
device = torch.device('cpu')
model = model.to(device)


# Main loop
print('Start collision avoidance control..')
direction = ''
while robot.step(timestep) != -1:
    camera_value = preprocessCameraImage(robot.camera)
    y = model(camera_value)
    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)
    prob_blocked = float(y.flatten()[0])
    if prob_blocked < 0.5:
        # Free path detected: move forward.
        if direction != 'forward':
            robot.forward(0.5)
            direction = 'forward'
    elif direction != 'left':
        # Edge detected: robot should turn!
        direction = 'left'
        robot.left(0.4)
    pass
