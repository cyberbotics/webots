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

from gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION
from construct import Container

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

global supervisor, game, red_team, blue_team, log_file, time_count
global REAL_TIME_FACTOR


def log(message, type):
    console_message = f'{AnsiCodes.YELLOW_FOREGROUND}{AnsiCodes.BOLD}{message}{AnsiCodes.RESET}' if type == 'Warning' \
                      else message
    print(console_message, file=sys.stderr if type == 'Error' else sys.stdout)
    if log_file:
        real_time = int(1000 * time.time()) / 1000
        log_file.write(f'[{real_time:.3f}|{time_count / 1000:08.3f}] {type}: {message}\n')  # log real and virtual times


def info(message):
    log(message, 'Info')


def warning(message):
    log(message, 'Warning')


def error(message):
    log(message, 'Error')


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
        info(f'Spawned {defname} {model} on port {port} at translation {translation[0]} {translation[1]} {translation[2]}, ' +
             f'rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]}')


def format_time(s):
    seconds = str(s % 60)
    minutes = str(int(s / 60))
    if len(minutes) == 1:
        minutes = '0' + minutes
    if len(seconds) == 1:
        seconds = '0' + seconds
    return minutes + ':' + seconds


def update_time_display():
    if game.state:
        if game.state.seconds_remaining == 600 and game.state.game_state == 'STATE_PLAYING':
            error('GameController sent "600 seconds remaining"!')
            return
        s = game.state.seconds_remaining
        if s < 0:
            s = -s
            sign = '-'
        else:
            sign = ' '
        value = format_time(s)
    else:
        sign = ' '
        value = '--:--'
    supervisor.setLabel(5, sign + value, game.overlay_x, game.overlay_y, game.font_size, 0x000000, 0.2, game.font)


def update_state_display():
    if game.state:
        state = game.state.game_state[6:]
        sr = game.state.secondary_seconds_remaining
        if sr > 65000:  # bug in GameController sometimes sending value like 65534
            sr = 0
        if sr > 0:
            if state == 'PLAYING':
                state = 'PLAY'
            state += ': ' + format_time(sr)
    else:
        state = ''
    supervisor.setLabel(6, ' ' * 41 + state, game.overlay_x, game.overlay_y, game.font_size, 0x000000, 0.2, game.font)


def update_score_display():
    if game.state:
        red = 0 if game.state.teams[0].team_color == 'RED' else 1
        blue = 1 if red == 0 else 0
        red_score = str(game.state.teams[red].score)
        blue_score = str(game.state.teams[blue].score)
    else:
        red_score = '0'
        blue_score = '0'
    if game.side_left == game.blue.id:
        red_score = ' ' * 24 + red_score
        offset = 21 if len(blue_score) == 2 else 22
        blue_score = ' ' * offset + blue_score
    else:
        blue_score = ' ' * 24 + blue_score
        offset = 21 if len(red_score) == 2 else 22
        red_score = ' ' * offset + red_score
    supervisor.setLabel(7, red_score, game.overlay_x, game.overlay_y, game.font_size, 0x000000, 0.2, game.font)
    supervisor.setLabel(8, blue_score, game.overlay_x, game.overlay_y, game.font_size, 0x000000, 0.2, game.font)


def setup_display():
    red = 0xd62929
    blue = 0x2943d6
    black = 0x000000
    white = 0xffffff
    n = len(red_team['name'])
    red_team_name = ' ' * 27 + red_team['name'] if game.side_left == game.blue.id else (20 - n) * ' ' + red_team['name']
    n = len(blue_team['name'])
    blue_team_name = (20 - n) * ' ' + blue_team['name'] if game.side_left == game.blue.id else ' ' * 27 + blue_team['name']
    transparency = 0.2
    # default background
    x = game.overlay_x
    y = game.overlay_y
    size = game.font_size
    font = game.font
    # default background
    supervisor.setLabel(0, '█' * 7 + ' ' * 14 + '█' * 5 + 14 * ' ' + '█' * 14, x, y, size, white, transparency, font)
    # team name background
    supervisor.setLabel(1, ' ' * 7 + '█' * 14 + ' ' * 5 + 14 * '█', x, y, size, white, transparency * 2, font)
    supervisor.setLabel(2, red_team_name, x, y, size, red, transparency, font)
    supervisor.setLabel(3, blue_team_name, x, y, size, blue, transparency, font)
    supervisor.setLabel(4, ' ' * 23 + '-', x, y, size, black, transparency, font)
    update_time_display()
    update_state_display()
    update_score_display()


def game_controller_heartbeat():
    heartbeat = Container(
        header=b"RGrt",
        version=GAME_CONTROLLER_RESPONSE_VERSION,
        team=0,
        player=1,
        message=2)
    try:
        game.udp_out.sendto(ReturnData.build(heartbeat), ('127.0.0.1', 3939))
    except Exception as e:
        error(f'UDP out failure: {e}')
        return
    data = None
    try:
        data, peer = game.udp_in.recvfrom(GameState.sizeof())
    except BlockingIOError:
        return
    except Exception as e:
        error(f'UDP input failure: {e}')
        pass
    if not data:
        error(f'No UDP data received')
        return
    previous_seconds_remaining = game.state.seconds_remaining if game.state else None
    previous_secondary_seconds_remaining = game.state.secondary_seconds_remaining if game.state else None
    previous_state = game.state.game_state if game.state else None
    if game.state:
        red = 0 if game.state.teams[0].team_color == 'RED' else 1
        blue = 1 if red == 0 else 0
        previous_red_score = game.state.teams[red].score
        previous_blue_score = game.state.teams[blue].score
    else:
        previous_red_score = 0
        previous_blue_score = 0

    game.state = GameState.parse(data)
    if previous_state != game.state.game_state or \
       previous_secondary_seconds_remaining != game.state.secondary_seconds_remaining or \
       game.state.seconds_remaining == 0:
        update_state_display()
    if previous_seconds_remaining != game.state.seconds_remaining:
        update_time_display()
    red = 0 if game.state.teams[0].team_color == 'RED' else 1
    blue = 1 if red == 0 else 0
    if previous_red_score != game.state.teams[red].score or \
       previous_blue_score != game.state.teams[blue].score:
        update_score_display()
    # print(game.state)
    info = str(game.state.secondary_state_info)
    secondary_state = game.state.secondary_state
    if secondary_state != 'STATE_NORMAL':
        print(f'GameController {secondary_state}: {info}')


def game_controller_send(message):
    if game.controller:
        game_controller_send.id += 1
        message = f'{game_controller_send.id}:{message}\n'
        game.controller.sendall(message.encode('ascii'))
        game_controller_send.unanswered[game_controller_send.id] = message.strip()
        while True:
            try:
                answers = game.controller.recv(1024).decode('ascii').split('\n')
                for answer in answers:
                    try:
                        id, result = answer.split(':')
                    except ValueError:
                        error(f'Cannot split {answer}')
                    try:
                        message = game_controller_send.unanswered[int(id)]
                        del game_controller_send.unanswered[int(id)]
                    except KeyError:
                        error(f'Received acknowledgment message for unknown message: {id}')
                        return
                    if result == 'OK':
                        return
                    if result == 'ILLEGAL':
                        error(f'Received illegal answer from GameController for message {id}:{message}.')
                    elif result == 'INVALID':
                        error(f'Received invalid answer from GameController for message {id}:{message}.')
                    else:
                        error(f'Warning: received unknown answer from GameController: {answer}.')
            except BlockingIOError:
                return


game_controller_send.id = 0
game_controller_send.unanswered = {}

time_count = 0
# REAL_TIME_FACTOR = 3  # simulated time is running 3 times slower than real time
REAL_TIME_FACTOR = 0.1  # used for testing (runs faster)

log_file = open('log.txt', 'w')

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
    error(f'Host is not correctly defined in game.json file, it should be {host} instead of {game.host}.')

# launch the GameController
try:
    JAVA_HOME = os.environ['JAVA_HOME']
    try:
        GAME_CONTROLLER_HOME = os.environ['GAME_CONTROLLER_HOME']
        if not os.path.exists(GAME_CONTROLLER_HOME):
            error(f'{GAME_CONTROLLER_HOME} (GAME_CONTROLLER_HOME) folder not found.')
            game.controller_process = None
        else:
            copyfile('game.json', os.path.join(GAME_CONTROLLER_HOME, 'resources', 'config', 'sim', 'game.json'))
            path = os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar', 'config', f'hl_sim_{field_size}', 'teams.cfg')
            red_line = f'{game.red.id}={red_team["name"]}\n'
            blue_line = f'{game.blue.id}={blue_team["name"]}\n'
            with open(path, 'w') as file:
                file.write((red_line + blue_line) if game.red.id < game.blue.id else (blue_line + red_line))
            game.controller_process = subprocess.Popen(
              [os.path.join(JAVA_HOME, 'bin', 'java'), '-jar', 'GameControllerSimulator.jar'],
              cwd=os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar'))
    except KeyError:
        GAME_CONTROLLER_HOME = None
        game.controller_process = None
        error('GAME_CONTROLLER_HOME environment variable not set, unable to launch GameController.')
except KeyError:
    JAVA_HOME = None
    GAME_CONTROLLER_HOME = None
    game.controller_process = None
    error('Warning: JAVA_HOME environment variable not set, unable to launch GameController.')

# start the webots supervisor
supervisor = Supervisor()
root = supervisor.getRoot()
children = root.getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "{field_size}" }}')
game.field_size_y = 3 if field_size == 'kid' else 4.5
game.field_size_x = 4.5 if field_size == 'kid' else 7
game.goal_half_width = 1.3
game.ball_radius = 0.07 if field_size == 'kid' else 0.1125
game.turf_depth = 0.01
game.ball_kickoff_translation = [0, 0, game.ball_radius + game.turf_depth]
ball_size = 1 if field_size == 'kid' else 5
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 {game.ball_kickoff_translation[2]} ' +
                                f'size {ball_size} }}')
game.side_left = game.red.id if bool(random.getrandbits(1)) else game.blue.id  # toss a coin to determine field side
game.kickoff = random.randint(1, 2)
game.state = None
game.font_size = 0.1
game.font = 'Lucida Console'
game.overlay_x = 0.02
game.overlay_y = 0.01
spawn_team(red_team, 'red', game.side_left == game.blue.id, children)
spawn_team(blue_team, 'blue', game.side_left == game.red.id, children)
red_team['score'] = 0
blue_team['score'] = 0
setup_display()

time_step = int(supervisor.getBasicTimeStep())

if game.controller_process:
    game.controller = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    retry = 0
    while True:
        try:
            game.controller.connect(('localhost', 8750))
            game.controller.setblocking(False)
            break
        except socket.error as msg:
            retry += 1
            if retry <= 10:
                warning(f'Could not connect to GameController at localhost:8750: {msg}. Retrying ({retry}/10)...')
                time.sleep(1)  # give some time to allow the GameControllerSimulator to start-up
                supervisor.step(time_step)
            else:
                error('Could not connect to GameController at localhost:8750.')
                game.controller = None
                break
    info('Connected to GameControllerSimulator at localhost:8750')
    game.udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    game.udp_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    game.udp_in.bind(('0.0.0.0', 3838))
    game.udp_in.setblocking(False)
    game.udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    game.udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
else:
    game.controller = None

update_state_display()
game_controller_send(f'SIDE_LEFT:{game.side_left}')
info(f'Left side is {"RED" if game.side_left == game.red.id else "BLUE"}')
game_controller_send(f'KICKOFF:{game.kickoff}')  # toss a coin to determine which team has kickoff
info(f'Kickoff is {"RED" if game.kickoff == game.red.id else "BLUE"}')

game.ball = supervisor.getFromDef('BALL')
game.ball_translation = supervisor.getFromDef('BALL').getField('translation')
game.ball_exited_countdown = 0
game.ready_countdown = (int)(120000 * REAL_TIME_FACTOR / time_step)  # 2 real minutes before we enter the ready state
game.play_countdown = 0
game.finish_countdown = 0
previous_seconds_remaining = 0
real_time_start = time.time()
while supervisor.step(time_step) != -1:
    game_controller_send(f'CLOCK:{time_count}')
    game_controller_heartbeat()
    if game.state is None:
        pass
    elif game.state.game_state == 'STATE_PLAYING':
        if previous_seconds_remaining != game.state.seconds_remaining:
            update_state_display()
            previous_seconds_remaining = game.state.seconds_remaining
            if game.state.seconds_remaining <= 0:
                game_controller_send(f'STATE:FINISH')
                if game.state.first_half:
                    info('End of first half')
                    game.finish_countdown = int(15000 * REAL_TIME_FACTOR / time_step)  # 15 real seconds for half time break
                else:
                    info('End of match')
        ball_translation = game.ball_translation.getSFVec3f()
        if game.ball_exited_countdown == 0 and \
            (ball_translation[1] > game.field_size_y or
             ball_translation[1] < -game.field_size_y or
             ball_translation[0] > game.field_size_x or
             ball_translation[0] < -game.field_size_x):
            game.ball_exited_countdown = int(2000 / time_step)  # wait 2 seconds after ball exited to replace it
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
                info(f'Score by {"RED" if scoring_team == game.red.id else "BLUE"}')

    elif game.state.game_state == 'STATE_READY':
        # the GameController will automatically change to the SET state once the state READY is over
        # the referee should wait about 5 seconds since the state SET started before sending the PLAY state
        game.play_countdown = int(5000 / time_step)
    elif game.state.game_state == 'STATE_SET' and game.play_countdown > 0:
        game.play_countdown -= 1
        if game.play_countdown == 0:
            game_controller_send(f'STATE:PLAY')
            info('State: PLAYING')
    elif game.state.game_state == 'STATE_FINISHED':
        game.finish_countdown -= 1
        if game.state.first_half:
            if game.finish_countdown == 0:
                game_controller_send('STATE:READY')  # begining of second half
        else:
            if game.finish_countdown == 0:
                info('End of the game: the winner is...')

    elif game.state.game_state == 'STATE_INITIAL':
        if game.ready_countdown > 0:
            game.ready_countdown -= 1
            if game.ready_countdown == 0:
                game_controller_send(f'STATE:READY')
                info('State: READY')

    if game.ball_exited_countdown > 0:
        game.ball_exited_countdown -= 1
        if game.ball_exited_countdown == 0:
            game.ball_translation.setSFVec3f(game.ball_exit_translation)
            game.ball.resetPhysics()

    time_count += time_step
    delta_time = real_time_start - time.time() + REAL_TIME_FACTOR * time_count / 1000
    if delta_time > 0:
        time.sleep(delta_time)

if log_file:
    log_file.close()
if game.controller:
    game.controller.close()
if game.controller_process:
    game.controller_process.terminate()
