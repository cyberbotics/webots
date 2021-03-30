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

from controller import Supervisor, AnsiCodes
import json
import math
import os
import random
from shutil import copyfile
import socket
import subprocess
import sys
import time
from types import SimpleNamespace


global supervisor, game, red_team, blue_team, game_controller


def spawn_team(team, color, red_on_right, children):
    for number in team['players']:
        model = team['players'][number]['proto']
        n = int(number) - 1
        port = game.red.ports[n] if color == 'red' else game.blue.ports[n]
        translation = team['players'][number]['halfTimeStartingPose']['translation']
        rotation = team['players'][number]['halfTimeStartingPose']['rotation']
        if red_on_right:  # symmetry with respect to the central line of the field
            translation[0] = -translation[0]
            rotation[3] = math.pi - rotation[3]
        defname = color.upper() + '_PLAYER_' + number
        string = f'DEF {defname} {model}{{name "{color} player {number}" ' + \
            f'translation {translation[0]} {translation[1]} {translation[2]} ' + \
            f'rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]} ' + \
            f'controllerArgs ["{port}"'
        hosts = game.red.hosts if color == 'red' else game.blue.hosts
        for host in hosts:
            string += f', "{host}"'
        string += '] }}'
        children.importMFNodeFromString(-1, string)


def display_score():
    red = 0xd62929
    blue = 0x2943d6
    black = 0x000000
    white = 0xffffff
    n = len(red_team['name'])
    red_team_name = ' ' * 27 + red_team['name'] if game.red_on_right else (20 - n) * ' ' + red_team['name']
    n = len(blue_team['name'])
    blue_team_name = (20 - n) * ' ' + blue_team['name'] if game.red_on_right else ' ' * 27 + blue_team['name']
    red_score = str(red_team['score'])
    blue_score = str(blue_team['score'])
    if game.red_on_right:
        red_score = ' ' * 24 + red_score
        offset = 21 if len(blue_score) == 2 else 22
        blue_score = ' ' * offset + blue_score
    else:
        blue_score = ' ' * 24 + blue_score
        offset = 21 if len(red_score) == 2 else 22
        red_score = ' ' * offset + red_score
    transparency = 0.2
    # default background
    x = game.overlay_x
    y = game.overlay_y
    size = game.font_size
    font = game.font
    supervisor.setLabel(0, '█' * 7 + ' ' * 14 + '█' * 5 + 14 * ' ' + '█' * 14, x, y, size, white, transparency, font)
    # team name background
    supervisor.setLabel(1, ' ' * 7 + '█' * 14 + ' ' * 5 + 14 * '█', x, y, size, white, transparency * 2, font)
    supervisor.setLabel(2, ' 00:00', x, y, size, black, transparency, font)
    supervisor.setLabel(3, red_team_name, x, y, size, red, transparency, font)
    supervisor.setLabel(4, blue_team_name, x, y, size, blue, transparency, font)
    supervisor.setLabel(5, red_score, x, y, size, black, transparency, font)
    supervisor.setLabel(6, blue_score, x, y, size, black, transparency, font)
    supervisor.setLabel(7, ' ' * 23 + '-', x, y, size, black, transparency, font)
    supervisor.setLabel(8, ' ' * 41 + game.status, x, y, size, black, transparency, font)


def game_controller_send(message):
    if game_controller:
        game_controller_send.id += 1
        message = f'{game_controller_send.id}:{message}\n'
        game_controller.sendall(message.encode('ascii'))
        answer = game_controller.recv(1024).decode('ascii')
        if answer == f'{game_controller_send.id}:OK':
            return True
        else:
            print('Warning: received unexepected answer from GameController: ' + answer)
            return False


game_controller_send.id = 0

# read configuration files
with open('game.json') as json_file:
    game = json.loads(json_file.read(), object_hook=lambda d: SimpleNamespace(**d))
with open(game.red.config) as json_file:
    red_team = json.load(json_file)
with open(game.blue.config) as json_file:
    blue_team = json.load(json_file)

# check team name length (should be at most 12 characters long, trim them if too long)
if len(red_team['name']) > 12:
    red_team['name'] = red_team['name'][:12]
if len(blue_team['name']) > 12:
    blue_team['name'] = blue_team['name'][:12]

# launch the GameController
JAVA_HOME = os.environ['JAVA_HOME']
game_controller_process = None
if JAVA_HOME is None:
    print('Warning: JAVA_HOME environment variable not set, unable to launch GameController.', file=sys.stderr)
    GAME_CONTROLLER_HOME = None
else:
    GAME_CONTROLLER_HOME = os.environ['GAME_CONTROLLER_HOME']
    if GAME_CONTROLLER_HOME:

        # FIXME: instead of that we should pass the game.json file to GameControllerSimulator when it supports it
        copyfile('game.json', os.path.join(GAME_CONTROLLER_HOME, 'resources', 'config', 'sim', 'game.json'))

        game_controller_process = subprocess.Popen(
          [os.path.join(JAVA_HOME, 'bin', 'java'), '-jar', 'GameControllerSimulator.jar'],
          cwd=os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar'))
    else:
        print('Warning: GAME_CONTROLLER_HOME environment variable not set, unable to launch GameController.', file=sys.stderr)

# start the webots supervisor
supervisor = Supervisor()
root = supervisor.getRoot()
children = root.getField('children')
game.red_on_right = bool(random.getrandbits(1))  # toss a coin to determine who is playing on which side
game.font_size = 0.1
game.font = 'Lucida Console'
game.overlay_x = 0.02
game.overlay_y = 0.01
spawn_team(red_team, 'red', game.red_on_right, children)
spawn_team(blue_team, 'blue', not game.red_on_right, children)
red_team['score'] = 0
blue_team['score'] = 0
game.status = 'KICK-OFF'
display_score()

host = socket.gethostbyname(socket.gethostname())
if host != '127.0.0.1' and host != game.host:
    print('Warning: host is not correctly defined in game.json file, it should be ' + game.host + ' instead of ' + host,
          file=sys.stderr)

time_step = int(supervisor.getBasicTimeStep())

game_controller = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
retry = 0
while True:
    try:
        game_controller.connect(('localhost', 8750))
        break
    except socket.error as msg:
        retry += 1
        if retry <= 10:
            print(f'{AnsiCodes.YELLOW_FOREGROUND}{AnsiCodes.BOLD}' +
                  f'Warning: could not connect to GameController at localhost:8750: {msg}. ' +
                  f'Retrying ({retry}/10)...{AnsiCodes.RESET}')
            time.sleep(1)  # give some time to allow the GameControllerSimulator to start-up
            supervisor.step(time_step)
        else:
            print('Warning: could not connect to GameController at localhost:8750: Giving up.', file=sys.stderr)
            game_controller = None
            break
print('Connected to GameControllerSimulator at localhost:8750')

time_count = 0
previous_seconds = -1
while supervisor.step(time_step) != -1:
    game_controller_send(f'CLOCK:{time_count}')
    seconds = (int)(time_count / 1000) % 60
    if seconds != previous_seconds:
        previous_seconds = seconds
        minutes = (int)(time_count / 60000)
        supervisor.setLabel(2, f' {minutes:02d}:{seconds:02d}', game.overlay_x, game.overlay_y, game.font_size, 0x000000, 0.2,
                            game.font)
    time_count += time_step

if game_controller:
    game_controller.close()
if game_controller_process:
    game_controller_process.terminate()
