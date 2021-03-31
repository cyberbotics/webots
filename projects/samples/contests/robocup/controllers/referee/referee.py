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


def update_time_display():
    if game.state == 'INIT':
        value = '00:00'
    else:
        value = '00.00'
    supervisor.setLabel(2, ' ' + value, game.overlay_x, game.overlay_y, game.font_size, 0x000000, 0.2, game.font)


def update_display():
    red = 0xd62929
    blue = 0x2943d6
    black = 0x000000
    white = 0xffffff
    n = len(red_team['name'])
    red_team_name = ' ' * 27 + red_team['name'] if game.side_left == game.blue.id else (20 - n) * ' ' + red_team['name']
    n = len(blue_team['name'])
    blue_team_name = (20 - n) * ' ' + blue_team['name'] if game.side_left == game.blue.id else ' ' * 27 + blue_team['name']
    red_score = str(red_team['score'])
    blue_score = str(blue_team['score'])
    if game.side_left == game.blue.id:
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
    update_time_display()
    supervisor.setLabel(3, red_team_name, x, y, size, red, transparency, font)
    supervisor.setLabel(4, blue_team_name, x, y, size, blue, transparency, font)
    supervisor.setLabel(5, red_score, x, y, size, black, transparency, font)
    supervisor.setLabel(6, blue_score, x, y, size, black, transparency, font)
    supervisor.setLabel(7, ' ' * 23 + '-', x, y, size, black, transparency, font)
    supervisor.setLabel(8, ' ' * 41 + game.state, x, y, size, black, transparency, font)


def game_controller_send(message):
    if game_controller:
        game_controller_send.id += 1
        message = f'{game_controller_send.id}:{message}\n'
        game_controller.sendall(message.encode('ascii'))
        # print(f'Sent {message}')
        game_controller_send.unanswered[game_controller_send.id] = message.strip()
        while True:
            try:
                answers = game_controller.recv(1024).decode('ascii').split('\n')
                for answer in answers:
                    try:
                        id, result = answer.split(':')
                    except ValueError:
                        print(f'Cannot split {answer}', file=sys.stderr)
                    # print(f'Received {id}:{result}')
                    try:
                        message = game_controller_send.unanswered[int(id)]
                        del game_controller_send.unanswered[int(id)]
                    except KeyError:
                        print(f'Warning: received acknowledgment message for unknown message: {id}', file=sys.stderr)
                        return
                    if result == 'OK':
                        return
                    if result == 'ILLEGAL':
                        print(f'Warning: received illegal answer from GameController for message {id}:{message}.',
                              file=sys.stderr)
                    elif result == 'INVALID':
                        print(f'Warning: received invalid answer from GameController for message {id}:{message}.',
                              file=sys.stderr)
                    else:
                        print(f'Warning: received unknown answer from GameController: {answer}.', file=sys.stderr)
            except BlockingIOError:
                return


game_controller_send.id = 0
game_controller_send.unanswered = {}

# read configuration files
with open('game.json') as json_file:
    game = json.loads(json_file.read(), object_hook=lambda d: SimpleNamespace(**d))
with open(game.red.config) as json_file:
    red_team = json.load(json_file)
with open(game.blue.config) as json_file:
    blue_team = json.load(json_file)
field_size = getattr(game, 'class').lower()

# check team name length (should be at most 12 characters long, trim them if too long)
if len(red_team['name']) > 12:
    red_team['name'] = red_team['name'][:12]
if len(blue_team['name']) > 12:
    blue_team['name'] = blue_team['name'][:12]

# check if the host parameter of the game.json file correspond to the actual host
host = socket.gethostbyname(socket.gethostname())
if host != '127.0.0.1' and host != game.host:
    print('Warning: host is not correctly defined in game.json file, it should be ' + host + ' instead of ' + game.host,
          file=sys.stderr)

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
        path = os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar', 'config', f'hl_sim_{field_size}', 'teams.cfg')
        red_line = f'{game.red.id}={red_team["name"]}\n'
        blue_line = f'{game.blue.id}={blue_team["name"]}\n'
        with open(path, 'w') as file:
            file.write((red_line + blue_line) if game.red.id < game.blue.id else (blue_line + red_line))
        game_controller_process = subprocess.Popen(
          [os.path.join(JAVA_HOME, 'bin', 'java'), '-jar', 'GameControllerSimulator.jar'],
          cwd=os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar'))
    else:
        print('Warning: GAME_CONTROLLER_HOME environment variable not set, unable to launch GameController.', file=sys.stderr)

# start the webots supervisor
supervisor = Supervisor()
root = supervisor.getRoot()
children = root.getField('children')
children.importMFNodeFromString(-2, f'RobocupSoccerField {{ size "{field_size}" }}')
game.field_size_y = 3 if field_size == 'kid' else 4.5
game.field_size_x = 4.5 if field_size == 'kid' else 7
game.goal_half_width = 1.3
game.ball_radius = 0.07 if field_size == 'kid' else 0.1125
game.turf_depth = 0.01
game.ball_kickoff_translation = [0, 0, game.ball_radius + game.turf_depth]
game.side_left = game.red.id if bool(random.getrandbits(1)) else game.blue.id  # toss a coin to determine field side
game.kickoff = game.red.id if bool(random.getrandbits(1)) else game.blue.id  # toss a coin to determine which team has kickoff
game.state = 'INIT'
game.font_size = 0.1
game.font = 'Lucida Console'
game.overlay_x = 0.02
game.overlay_y = 0.01
spawn_team(red_team, 'red', game.side_left == game.blue.id, children)
spawn_team(blue_team, 'blue', game.side_left == game.red.id, children)
red_team['score'] = 0
blue_team['score'] = 0
update_display()

time_step = int(supervisor.getBasicTimeStep())

game_controller = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
retry = 0
while True:
    try:
        game_controller.connect(('localhost', 8750))
        game_controller.setblocking(False)
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

game.state = 'READY'
update_display()
game_controller_send(f'SIDE_LEFT:{game.side_left}')
game_controller_send(f'KICKOFF:{game.kickoff}')
game_controller_send(f'STATE:{game.state}')

game.ball = supervisor.getFromDef('BALL')
game.ball_translation = supervisor.getFromDef('BALL').getField('translation')
game.ball_exited = 0
game.extra_time = 0
time_count = 0
time_offset = 0
previous_seconds = -1
scoring_team = None
while supervisor.step(time_step) != -1:
    game_controller_send(f'CLOCK:{time_count}')
    if game.state == 'READY':
        seconds = 45 - (int)((time_count - time_offset) / 1000)
        if seconds != previous_seconds:
            previous_seconds = seconds
            supervisor.setLabel(2, f' 00:{seconds:02d}', game.overlay_x, game.overlay_y, game.font_size,
                                0x000000, 0.2, game.font)
        if time_count >= 45000 + time_offset:
            game.state = 'SET'
            previous_seconds = -1
            time_offset = time_count
            game_controller_send(f'STATE:{game.state}')
            update_display()
    elif game.state == 'SET':
        seconds = 5 - (int)((time_count - time_offset) / 1000)
        if seconds != previous_seconds:
            previous_seconds = seconds
            supervisor.setLabel(2, f' 00:{seconds:02d}', game.overlay_x, game.overlay_y, game.font_size,
                                0x000000, 0.2, game.font)
        if time_count >= 5000 + time_offset:
            game.state = 'PLAY'
            previous_seconds = -1
            time_offset = time_count
            game_controller_send(f'STATE:{game.state}')
            update_display()
    elif game.state == 'PLAY':
        countdown = 600000 - time_count + time_offset
        seconds = (int)(countdown / 1000) % 60
        if seconds != previous_seconds:
            previous_seconds = seconds
            minutes = (int)(countdown / 60000)
            supervisor.setLabel(2, f' {minutes:02d}:{seconds:02d}', game.overlay_x, game.overlay_y, game.font_size,
                                0x000000, 0.2, game.font)
        ball_translation = game.ball_translation.getSFVec3f()
        if game.ball_exited == 0 and \
            (ball_translation[1] > game.field_size_y or
             ball_translation[1] < -game.field_size_y or
             ball_translation[0] > game.field_size_x or
             ball_translation[0] < -game.field_size_x):
            game.ball_exited = 2000  # wait 2 seconds after ball exited to replace it
            game.extra_time += game.ball_exited
            game.ball_exit_translation = ball_translation
            scoring_team = None
            if game.ball_exit_translation[1] > game.field_size_y:
                game.ball_exit_translation[1] = game.field_size_y
            elif game.ball_exit_translation[1] < -game.field_size_y:
                game.ball_exit_translation[1] = -game.field_size_y
            if game.ball_exit_translation[0] > game.field_size_x:
                if game.ball_exit_translation[1] < game.goal_half_width and \
                   game.ball_exit_translation[1] > -game.goal_half_width:
                    scoring_team = game.side_left
                else:
                    game.ball_exit_translation[0] = game.field_size_x
            elif game.ball_exit_translation[0] < -game.field_size_x:
                if game.ball_exit_translation[1] < game.goal_half_width and \
                   game.ball_exit_translation[1] > -game.goal_half_width:
                    scoring_team = game.red.id if game.blue.id == game.side_left else game.blue.id
                else:
                    game.ball_exit_translation[0] = -game.field_size_x
            if scoring_team:
                game.ball_exit_translation = game.ball_kickoff_translation
                game_controller_send(f'SCORE:{scoring_team}')
                if scoring_team == game.red.id:
                    red_team['score'] += 1
                else:
                    blue_team['score'] += 1
                game.state = 'READY'
                time_offset = time_count
                update_display()

        if game.ball_exited > 0:
            game.ball_exited -= time_step
            if game.ball_exited <= 0:
                game.ball_translation.setSFVec3f(game.ball_exit_translation)
                game.ball.resetPhysics()
    time_count += time_step

if game_controller:
    game_controller.close()
if game_controller_process:
    game_controller_process.terminate()
