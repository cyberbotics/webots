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

from gamestate import GameState

from controller import Supervisor, AnsiCodes

import json
import math
import os
import random
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

SIMULATED_TIME_BEFORE_BALL_RESET = 2      # once the ball exited the field, let it run for 2 simulated seconds and replacing it
SIMULATED_TIME_BEFORE_PLAY_STATE = 5      # wait 5 simulated seconds in SET state before sending the PLAY state
HALF_TIME_BREAK_SIMULATED_DURATION = 15   # the half-time break lasts 15 simulated seconds
REAL_TIME_BEFORE_FIRST_READY_STATE = 120  # wait 2 real minutes before sending the first READY state
LINE_WIDTH = 0.05                         # width of the white lines on the soccer field
GOAL_WIDTH = 2.6                          # width of the goal
GOAL_HEIGHT_KID = 1.2                     # height of the goal in kid size league
GOAL_HEIGHT_ADULT = 1.8                   # height of the goal in adult size league

LINE_HALF_WIDTH = LINE_WIDTH / 2
GOAL_HALF_WIDTH = GOAL_WIDTH / 2

global supervisor, game, red_team, blue_team, log_file, time_count


def log(message, type):
    console_message = f'{AnsiCodes.YELLOW_FOREGROUND}{AnsiCodes.BOLD}{message}{AnsiCodes.RESET}' if type == 'Warning' \
                      else message
    print(console_message, file=sys.stderr if type == 'Error' else sys.stdout)
    if log_file:
        real_time = int(1000 * (time.time() - log.start_time)) / 1000
        log_file.write(f'[{real_time:08.3f}|{time_count / 1000:08.3f}] {type}: {message}\n')  # log real and virtual times


log.start_time = time.time()


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
        halfTimeStartingTranslation = team['players'][number]['halfTimeStartingPose']['translation']
        halfTimeStartingRotation = team['players'][number]['halfTimeStartingPose']['rotation']
        reentryStartingTranslation = team['players'][number]['reentryStartingPose']['translation']
        reentryStartingRotation = team['players'][number]['reentryStartingPose']['rotation']
        shootoutStartingTranslation = team['players'][number]['shootoutStartingPose']['translation']
        shootoutStartingRotation = team['players'][number]['shootoutStartingPose']['rotation']
        if red_on_right:  # symmetry with respect to the central line of the field
            halfTimeStartingTranslation[0] = -halfTimeStartingTranslation[0]
            reentryStartingTranslation[0] = -reentryStartingTranslation[0]
            shootoutStartingTranslation[0] = -shootoutStartingTranslation[0]
            halfTimeStartingRotation[3] = math.pi - halfTimeStartingRotation[3]
            reentryStartingRotation[3] = math.pi - reentryStartingRotation[3]
            shootoutStartingRotation[3] = math.pi - shootoutStartingRotation[3]
        defname = color.upper() + '_PLAYER_' + number
        string = f'DEF {defname} {model}{{name "{color} player {number}" translation ' + \
            f'{halfTimeStartingTranslation[0]} {halfTimeStartingTranslation[1]} {halfTimeStartingTranslation[2]} rotation ' + \
            f'{halfTimeStartingRotation[0]} {halfTimeStartingRotation[1]} {halfTimeStartingRotation[2]} ' + \
            f'{halfTimeStartingRotation[3]} controllerArgs ["{port}"'
        hosts = game.red.hosts if color == 'red' else game.blue.hosts
        for host in hosts:
            string += f', "{host}"'
        string += '] }}'
        children.importMFNodeFromString(-1, string)
        team['players'][number]['robot'] = supervisor.getFromDef(defname)
        info(f'Spawned {defname} {model} on port {port} at halfTimeStartingPose: translation ' +
             f'{halfTimeStartingTranslation[0]} {halfTimeStartingTranslation[1]} {halfTimeStartingTranslation[2]}, rotation ' +
             f'{halfTimeStartingRotation[0]} {halfTimeStartingRotation[1]} {halfTimeStartingRotation[2]} ' +
             f'{halfTimeStartingRotation[3]}')


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


def update_team_display():
    red = 0xd62929
    blue = 0x2943d6
    n = len(red_team['name'])
    red_team_name = ' ' * 27 + red_team['name'] if game.side_left == game.blue.id else (20 - n) * ' ' + red_team['name']
    n = len(blue_team['name'])
    blue_team_name = (20 - n) * ' ' + blue_team['name'] if game.side_left == game.blue.id else ' ' * 27 + blue_team['name']
    supervisor.setLabel(3, red_team_name, game.overlay_x, game.overlay_y, game.font_size, red, 0.2, game.font)
    supervisor.setLabel(4, blue_team_name, game.overlay_x, game.overlay_y, game.font_size, blue, 0.2, game.font)
    update_score_display()


def setup_display():
    black = 0x000000
    white = 0xffffff
    transparency = 0.2
    x = game.overlay_x
    y = game.overlay_y
    size = game.font_size
    font = game.font
    # default background
    supervisor.setLabel(0, '█' * 7 + ' ' * 14 + '█' * 5 + 14 * ' ' + '█' * 14, x, y, size, white, transparency, font)
    # team name background
    supervisor.setLabel(1, ' ' * 7 + '█' * 14 + ' ' * 5 + 14 * '█', x, y, size, white, transparency * 2, font)
    supervisor.setLabel(2, ' ' * 23 + '-', x, y, size, black, transparency, font)
    update_team_display()
    update_time_display()
    update_state_display()


def game_controller_receive():
    try:
        data, peer = game.udp.recvfrom(GameState.sizeof())
    except BlockingIOError:
        return
    except Exception as e:
        error(f'UDP input failure: {e}')
        pass
    if not data:
        error('No UDP data received')
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
    if previous_state != game.state.game_state:
        info(f'New state received from GameController: {game.state.game_state}')
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
    secondary_state_info = str(game.state.secondary_state_info)
    secondary_state = game.state.secondary_state
    if secondary_state != 'STATE_NORMAL':
        print(f'GameController {secondary_state}: {secondary_state_info}')


def game_controller_send(message):
    if game.controller:
        game_controller_send.id += 1
        if message[:6] != 'CLOCK:':
            info(f'Sending {message} to GameController')
        message = f'{game_controller_send.id}:{message}\n'
        game.controller.sendall(message.encode('ascii'))
        # info(f'sending {message.strip()} to GameController')
        game_controller_send.unanswered[game_controller_send.id] = message.strip()
        while True:
            try:
                answers = game.controller.recv(1024).decode('ascii').split('\n')
                # info(f'received {answers} from GameController')
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


def point_inside_field(point):
    if point[2] > 0.01:  # in the air
        return False
    if point[0] > game.field_size_x or point[0] < -game.field_size_x or \
       point[1] > game.field_size_y or point[1] < -game.field_size_y:
        return False
    return True


def check_team_position(team, color):
    for number in team['players']:
        robot = team['players'][number]['robot']
        n = robot.getNumberOfContactPoints(True)
        for i in range(0, n):
            point = robot.getContactPoint(i)
            if point_inside_field(point):
                team['players'][number]['penalty'] = 'INCAPABLE'
                team['players'][number]['penalty_reason'] = 'halfTimeStartingPose inside field'
            else:  # check if player are fully on their side of the field
                if game.side_left == (game.red.id if color == 'red' else game.blue.id):
                    if point[0] > -LINE_HALF_WIDTH:
                        team['players'][number]['penalty'] = 'INCAPABLE'
                        team['players'][number]['penalty_reason'] = 'halfTimeStartingPose outside team side'
                else:
                    if point[0] < LINE_HALF_WIDTH:
                        team['players'][number]['penalty'] = 'INCAPABLE'
                        team['players'][number]['penalty_reason'] = 'halfTimeStartingPose outside team side'


def send_penalties(team, color):
    for number in team['players']:
        if 'penalty' in team['players'][number]:
            penalty = team['players'][number]['penalty']
            reason = team['players'][number]['penalty_reason']
            del team['players'][number]['penalty']
            del team['players'][number]['penalty_reason']
            team_id = game.red.id if color == 'red' else game.blue.id
            game_controller_send(f'PENALTY:{team_id}:{number}:{penalty}')
            robot = team['players'][number]['robot']
            robot.resetPhysics()
            translation = robot.getField('translation')
            rotation = robot.getField('rotation')
            t = team['players'][number]['reentryStartingPose']['translation']
            r = team['players'][number]['reentryStartingPose']['rotation']
            translation.setSFVec3f(t)
            rotation.setSFRotation(r)
            info(f'{penalty} penalty for {color} player {number}: {reason}. Sent to reentryStartingPose: ' +
                 f'translation {t[0]} {t[1]} {t[2]}, rotation {r[0]} {r[1]} {r[2]} {r[3]}')


def flip_pose(pose):
    pose['translation'][0] = -pose['translation'][0]
    pose['rotation'][3] = math.pi - pose['rotation'][3]


def flip_sides():
    game.side_left = 2 if game.side_left == 1 else 1  # flip sides (no need to notify GameController, it does it automatically)
    for team in [red_team, blue_team]:
        for number in team['players']:
            flip_pose(team['players'][number]['halfTimeStartingPose'])
            flip_pose(team['players'][number]['reentryStartingPose'])
            flip_pose(team['players'][number]['shootoutStartingPose'])


def reset_player(color, number, pose):
    team = red_team if color == 'red' else blue_team
    player = team['players'][str(number)]
    player['robot'].resetPhysics()
    translation = player['robot'].getField('translation')
    rotation = player['robot'].getField('rotation')
    t = player[pose]['translation']
    r = player[pose]['rotation']
    translation.setSFVec3f(t)
    rotation.setSFRotation(r)
    info(f'{color} player {number} reset to {pose}: ' +
         f'translation {t[0]} {t[1]} {t[2]}, rotation {r[0]} {r[1]} {r[2]} {r[3]}')


def reset_teams(pose):
    for number in red_team['players']:
        reset_player('red', number, pose)
    for number in blue_team['players']:
        reset_player('blue', number, pose)


def check_touch(point, team, color):  # check which player in a team has touch the ball at specified point
    for number in team['players']:
        n = team['players'][number]['robot'].getNumberOfContactPoints(True)
        for i in range(0, n):
            r_point = team['players'][number]['robot'].getContactPoint(i)
            if r_point[2] <= 0.01:  # contact with the ground
                continue
            if point[0] == r_point[0] and point[1] == r_point[1] and point[2] == r_point[2]:
                if game.ball_last_touch_team != color or game.ball_last_touch_player != int(number):
                    game.ball_last_touch_team = color
                    game.ball_last_touch_player = int(number)
                    info(f'Ball touched by {color} player {number}.')
                else:
                    info(f'Ball touched again by same player.')


def check_fallen(team, color):
    for number in team['players']:
        n = team['players'][number]['robot'].getNumberOfContactPoints(True)
        already_down = 'fallen' in team['players'][number]
        fallen = False
        for i in range(0, n):
            r_point = team['players'][number]['robot'].getContactPoint(i)
            if r_point[2] > 0.01:  # not a contact with the ground
                continue
            node = team['players'][number]['robot'].getContactPointNode(i)
            if not node:  # FIXME: bug here?
                continue
            name = node.getField('name').getSFString()
            if name[:-6] == '[feet]':
                continue
            fallen = True
            if already_down:
                break
            team['players'][number]['fallen'] = time_count
            break
        if already_down:
            if fallen:
                if time_count - team['players'][number]['fallen'] > 20000:  # more than 20 seconds down
                    info(f'{color} player {number} has fallen and didn\'t recover in the last 20 seconds.')
            else:  # recovered on time
                del team['players'][number]['fallen']


game_controller_send.id = 0
game_controller_send.unanswered = {}

time_count = 0

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
            path = os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar', 'config', f'hl_sim_{field_size}', 'teams.cfg')
            red_line = f'{game.red.id}={red_team["name"]}\n'
            blue_line = f'{game.blue.id}={blue_team["name"]}\n'
            json_file = os.path.join(os.getcwd(), 'game.json')
            with open(path, 'w') as file:
                file.write((red_line + blue_line) if game.red.id < game.blue.id else (blue_line + red_line))
            game.controller_process = subprocess.Popen(
              [os.path.join(JAVA_HOME, 'bin', 'java'), '-jar', 'GameControllerSimulator.jar', '--config', json_file],
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

# finalize the game object
if not hasattr(game, 'real_time_factor'):
    game.real_time_factor = 3  # simulation speed defaults to 1/3 of real time, e.g., 0.33x real time in the Webots speedometer
info(f'Real time factor is set to {game.real_time_factor}.')
game.field_size_y = 3 if field_size == 'kid' else 4.5
game.field_size_x = 4.5 if field_size == 'kid' else 7
game.center_circle_radius = 0.75 if field_size == 'kid' else 1.5
game.goal_height = GOAL_HEIGHT_KID if field_size == 'kid' else GOAL_HEIGHT_ADULT
game.ball_radius = 0.07 if field_size == 'kid' else 0.1125
game.turf_depth = 0.01
game.ball_kickoff_translation = [0, 0, game.ball_radius + game.turf_depth]

# start the webots supervisor
supervisor = Supervisor()
root = supervisor.getRoot()
children = root.getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "{field_size}" }}')
ball_size = 1 if field_size == 'kid' else 5
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 {game.ball_kickoff_translation[2]} ' +
                                f'size {ball_size} }}')
game.side_left = game.red.id if bool(random.getrandbits(1)) else game.blue.id  # toss a coin to determine field side
game.kickoff = random.randint(1, 2)  # toss a coin to determine which team has kickoff
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
                time.sleep(retry)  # give some time to allow the GameControllerSimulator to start-up
                supervisor.step(time_step)
            else:
                error('Could not connect to GameController at localhost:8750.')
                game.controller = None
                break
    info('Connected to GameControllerSimulator at localhost:8750')
    game.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    game.udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    game.udp.bind(('0.0.0.0', 3838))
    game.udp.setblocking(False)
else:
    game.controller = None

update_state_display()

info(f'Red team is {red_team["name"]}')
info(f'Blue team is {blue_team["name"]}')
info(f'Left side is {"red" if game.side_left == game.red.id else "blue"}')
info(f'Kickoff is {"red" if game.kickoff == game.red.id else "blue"}')
game_controller_send(f'SIDE_LEFT:{game.side_left}')
game_controller_send(f'KICKOFF:{game.kickoff}')

game.ball = supervisor.getFromDef('BALL')
game.ball_translation = supervisor.getFromDef('BALL').getField('translation')
game.ball_exited_countdown = 0
game.ball_last_touch_team = 0
game.ball_last_touch_player = 0
game.ready_countdown = (int)(REAL_TIME_BEFORE_FIRST_READY_STATE * 1000 * game.real_time_factor / time_step)
game.play_countdown = 0
game.sent_finish = False
previous_seconds_remaining = 0
real_time_start = time.time()
while supervisor.step(time_step) != -1:
    game_controller_send(f'CLOCK:{time_count}')
    game_controller_receive()
    if game.state is None:
        pass
    elif game.state.game_state == 'STATE_PLAYING':
        if previous_seconds_remaining != game.state.seconds_remaining:
            update_state_display()
            previous_seconds_remaining = game.state.seconds_remaining
            if not game.sent_finish and game.state.seconds_remaining <= 0:
                game_controller_send('STATE:FINISH')
                game.sent_finish = True
                if game.state.first_half:
                    info('End of first half')
                    flip_sides()
                    reset_teams('halfTimeStartingPose')
                    update_team_display()
                else:
                    info('End of match')
        ball_translation = game.ball_translation.getSFVec3f()
        if game.ball_exited_countdown == 0 and \
            (ball_translation[1] - game.ball_radius > game.field_size_y or
             ball_translation[1] + game.ball_radius < -game.field_size_y or
             ball_translation[0] - game.ball_radius > game.field_size_x or
             ball_translation[0] + game.ball_radius < -game.field_size_x):
            info(f'Ball left the field at {ball_translation[0]} {ball_translation[1]} {ball_translation[2]} after being '
                 f'touched by {game.ball_last_touch_team} player {game.ball_last_touch_player}.')
            game.ball_exited_countdown = int(SIMULATED_TIME_BEFORE_BALL_RESET * 1000 / time_step)
            game.ball_exit_translation = ball_translation
            scoring_side = None
            if game.ball_exit_translation[1] - game.ball_radius > game.field_size_y:
                game.ball_exit_translation[1] = game.field_size_y - LINE_HALF_WIDTH
            elif game.ball_exit_translation[1] + game.ball_radius < -game.field_size_y:
                game.ball_exit_translation[1] = -game.field_size_y + LINE_HALF_WIDTH
            if game.ball_exit_translation[0] - game.ball_radius > game.field_size_x:
                if game.ball_exit_translation[1] < GOAL_HALF_WIDTH and \
                   game.ball_exit_translation[1] > -GOAL_HALF_WIDTH and game.ball_exit_translation[2] < game.goal_height:
                    scoring_side = game.side_left
                else:
                    if game.ball_last_touch_team == 'red' and game.side_left == game.red.id or \
                       game.ball_last_touch_team == 'blue' and game.side_left == game.blue.id:
                        game.ball_exit_translation[0] = 0  # reset the ball of the centerline
                    else:  # corner kick
                        game.ball_exit_translation[0] = game.field_size_x - LINE_HALF_WIDTH
                        game.ball_exit_translation[1] = game.field_size_y - LINE_HALF_WIDTH \
                            if game.ball_exit_translation[1] > 0 else -game.field_size_y + LINE_HALF_WIDTH
            elif game.ball_exit_translation[0] + game.ball_radius < -game.field_size_x:
                if game.ball_exit_translation[1] < GOAL_HALF_WIDTH and \
                   game.ball_exit_translation[1] > -GOAL_HALF_WIDTH and game.ball_exit_translation[2] < game.goal_height:
                    scoring_side = game.red.id if game.blue.id == game.side_left else game.blue.id
                else:
                    if game.ball_last_touch_team == 'red' and game.side_left == game.blue.id or \
                       game.ball_last_touch_team == 'blue' and game.side_left == game.red.id:
                        game.ball_exit_translation[0] = 0  # reset the ball of the centerline
                    else:  # corner kick
                        game.ball_exit_translation[0] = -game.field_size_x + LINE_HALF_WIDTH
                        game.ball_exit_translation[1] = game.field_size_y - LINE_HALF_WIDTH \
                            if game.ball_exit_translation[1] > 0 else -game.field_size_y + LINE_HALF_WIDTH
            if scoring_side:
                game.ball_exit_translation = game.ball_kickoff_translation
                game_controller_send(f'SCORE:{scoring_side}')
                goal = 'red' if scoring_side == game.blue.id else 'blue'
                game.kickoff = scoring_side
                info(f'Score in {goal} goal by {game.ball_last_touch_team} player {game.ball_last_touch_player}')

    elif game.state.game_state == 'STATE_READY':
        # the GameController will automatically change to the SET state once the state READY is over
        # the referee should wait a little time since the state SET started before sending the PLAY state
        game.play_countdown = int(SIMULATED_TIME_BEFORE_PLAY_STATE * 1000 / time_step)
        game.ball.resetPhysics()
        game.ball_translation.setSFVec3f(game.ball_kickoff_translation)
    elif game.state.game_state == 'STATE_SET' and game.play_countdown > 0:
        game.play_countdown -= 1
        if game.play_countdown == 0:
            game_controller_send('STATE:PLAY')
    elif game.state.game_state == 'STATE_FINISHED':
        game.sent_finish = False
        if game.state.first_half:
            if game.ready_countdown == 0:
                print('state FINISHED!')
                info('Beginning of second half.')
                game.ready_countdown = int(HALF_TIME_BREAK_SIMULATED_DURATION * 1000 * game.real_time_factor / time_step)
        else:
            info('End of the game.')
            info(f'The score is {game.state.teams[0].score}-{game.state.teams[1].score}.')
            if game.state.teams[0].score == game.state.teams[1].score:
                info('This is a draw.')
            else:
                winner = 0 if game.state.teams[0].score > game.state.teams[1].score else 1
                info(f'The winner is the {game.state.teams[winner].team_color.lower()} team.')
            break

    elif game.state.game_state == 'STATE_INITIAL':
        check_team_position(red_team, 'red')
        check_team_position(blue_team, 'blue')
        if game.ready_countdown > 0:
            game.ready_countdown -= 1
            if game.ready_countdown == 0:
                game_controller_send('STATE:READY')
                send_penalties(red_team, 'red')
                send_penalties(blue_team, 'blue')

    if game.ball_exited_countdown > 0:
        game.ball_exited_countdown -= 1
        if game.ball_exited_countdown == 0:
            game.ball.resetPhysics()
            game.ball_translation.setSFVec3f(game.ball_exit_translation)
            info('Ball respawned at '
                 f'{game.ball_exit_translation[0]} {game.ball_exit_translation[1]} {game.ball_exit_translation[2]}')

    # determine which robot touched the ball if any
    n = game.ball.getNumberOfContactPoints()
    for i in range(0, n):
        point = game.ball.getContactPoint(i)
        if point[2] <= 0.01:  # contact with the ground
            continue
        check_touch(point, red_team, 'red')
        check_touch(point, blue_team, 'blue')

    # detect fallen robots
    check_fallen(red_team, 'red')
    check_fallen(blue_team, 'blue')

    time_count += time_step

    # slow down the simulation if needed to respect the real time factor constraint
    delta_time = real_time_start - time.time() + game.real_time_factor * time_count / 1000
    if delta_time > 0:
        time.sleep(delta_time)

if log_file:
    log_file.close()
if game.controller:
    game.controller.close()
if game.controller_process:
    game.controller_process.terminate()

supervisor.simulationQuit(0)
while supervisor.step(time_step) != -1:
    pass
