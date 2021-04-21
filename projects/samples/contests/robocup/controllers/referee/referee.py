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

from controller import Supervisor, AnsiCodes, Node

import copy
import json
import math
import os
import random
import socket
import subprocess
import sys
import time
import transforms3d

from types import SimpleNamespace

SIMULATED_TIME_BEFORE_INTERRUPTION = 2    # run the game for 2 simulated seconds before firing interruption
SIMULATED_TIME_BEFORE_PLAY_STATE = 5      # wait 5 simulated seconds in SET state before sending the PLAY state
HALF_TIME_BREAK_SIMULATED_DURATION = 15   # the half-time break lasts 15 simulated seconds
REAL_TIME_BEFORE_FIRST_READY_STATE = 120  # wait 2 real minutes before sending the first READY state
IN_PLAY_TIMEOUT = 10                      # time after which the ball is considered in play even if it was not kicked
FALLEN_TIMEOUT = 20                       # if a robot is down (fallen) for more than this amount of time, it gets penalized
GOAL_KEEPER_BALL_HOLDING_TIMEOUT = 6      # a goal keeper may hold the ball up to 6 seconds on the ground
PLAYER_BALL_HOLDING_TIMEOUT = 1           # a field player may hold the ball up to 1 second
HAND_BALL_HOLDING_TIMEOUT = 10            # a player throwing in or a goal keeper may hold the ball up to 10 seconds in hands
FOUL_PUSHING_TIME = 1                     # 1 second
FOUL_PUSHING_PERIOD = 2                   # 2 seconds
FOUL_VINCITY_DISTANCE = 2                 # 2 meters
FOUL_DISTANCE_THRESHOLD = 0.1             # 0.1 meter
FOUL_SPEED_THRESHOLD = 0.2                # 0.2 m/s
FOUL_DIRECTION_THRESHOLD = math.pi / 6    # 30 degrees
LINE_WIDTH = 0.05                         # width of the white lines on the soccer field
GOAL_WIDTH = 2.6                          # width of the goal
GOAL_HEIGHT_KID = 1.2                     # height of the goal in kid size league
GOAL_HEIGHT_ADULT = 1.8                   # height of the goal in adult size league
RED_COLOR = 0xd62929                      # red team color used for the display
BLUE_COLOR = 0x2943d6                     # blue team color used for the display

# game interruptions requiring a free kick procedure
GAME_INTERRUPTIONS = {
    'DIRECT_FREEKICK': 'direct free kick',
    'INDIRECT_FREEKICK': 'indirect free kick',
    'PENALTYKICK': 'penalty kick',
    'CORNERKICK': 'corner kick',
    'GOALKICK': 'goal kick',
    'THROWIN': 'throw in'}

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


def toss_a_coin_if_needed(attribute):  # attribute should be either "side_left" or "kickoff"
    # If game.json contains such an attribute, use it to determine field side and kick-off
    # Supported values are "red", "blue" and "random". Default value is "random".
    if hasattr(game, attribute):
        if getattr(game, attribute) == 'red':
            setattr(game, attribute, game.red.id)
        elif getattr(game, attribute) == 'blue':
            setattr(game, attribute, game.blue.id)
        elif getattr(game, attribute) != 'random':
            error(f'Unsupported value for "{attribute}" in game.json file: {getattr(game, attribute)}, using "random".')
            setattr(game, attribute, 'random')
    else:
        setattr(game, attribute, 'random')
    if getattr(game, attribute) == 'random':  # toss a coin to determine a random team
        setattr(game, attribute, game.red.id if bool(random.getrandbits(1)) else game.blue.id)


def spawn_team(team, color, red_on_right, children):
    for number in team['players']:
        model = team['players'][number]['proto']
        n = int(number) - 1
        port = game.red.ports[n] if color == 'red' else game.blue.ports[n]
        if red_on_right:  # symmetry with respect to the central line of the field
            player = team['players'][number]
            flip_pose(player['halfTimeStartingPose'])
            flip_pose(player['reentryStartingPose'])
            flip_pose(player['shootoutStartingPose'])
        defname = color.upper() + '_PLAYER_' + number
        halfTimeStartingTranslation = team['players'][number]['halfTimeStartingPose']['translation']
        halfTimeStartingRotation = team['players'][number]['halfTimeStartingPose']['rotation']
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
        info(f'Spawned {defname} {model} on port {port} at halfTimeStartingPose: translation (' +
             f'{halfTimeStartingTranslation[0]} {halfTimeStartingTranslation[1]} {halfTimeStartingTranslation[2]}), ' +
             f'rotation ({halfTimeStartingRotation[0]} {halfTimeStartingRotation[1]} {halfTimeStartingRotation[2]} ' +
             f'{halfTimeStartingRotation[3]}).')


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
        if state == 'READY' or state == 'SET':  # kickoff
            color = RED_COLOR if game.kickoff == game.red.id else BLUE_COLOR
        elif game.interruption_team is not None:  # interruption
            color = RED_COLOR if game.interruption_team == game.red.id else BLUE_COLOR
        else:
            color = 0x000000
        sr = IN_PLAY_TIMEOUT - game.interruption_seconds + game.state.seconds_remaining \
            if game.interruption_seconds is not None \
            else game.state.secondary_seconds_remaining
        if sr > 0:
            if game.interruption is None:  # kickoff
                color = RED_COLOR if game.kickoff == game.red.id else BLUE_COLOR
                state = 'PLAY' if state == 'PLAYING' else 'READY'
            else:  # interruption
                state = 'PLAY' if state == 'PLAYING' and game.interruption_seconds is not None else 'READY'
            state += ' ' + format_time(sr)
        elif game.interruption is not None:
            state = game.interruption
    else:
        state = ''
        color = 0x000000
    supervisor.setLabel(6, ' ' * 41 + state, game.overlay_x, game.overlay_y, game.font_size, color, 0.2, game.font)


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
    n = len(red_team['name'])
    red_team_name = ' ' * 27 + red_team['name'] if game.side_left == game.blue.id else (20 - n) * ' ' + red_team['name']
    n = len(blue_team['name'])
    blue_team_name = (20 - n) * ' ' + blue_team['name'] if game.side_left == game.blue.id else ' ' * 27 + blue_team['name']
    supervisor.setLabel(3, red_team_name, game.overlay_x, game.overlay_y, game.font_size, RED_COLOR, 0.2, game.font)
    supervisor.setLabel(4, blue_team_name, game.overlay_x, game.overlay_y, game.font_size, BLUE_COLOR, 0.2, game.font)
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


def team_index(color):
    id = game.red.id if color == 'red' else game.blue.id
    index = 0 if game.state.teams[0].team_number == id else 1
    return index


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
        info(f'New state received from GameController: {game.state.game_state}.')
    if game.state.game_state == 'STATE_PLAYING' and \
       game.state.secondary_seconds_remaining == 0 and previous_secondary_seconds_remaining > 0:
        info('Ball in play.')
        game.in_play = True
    if previous_state != game.state.game_state or \
       previous_secondary_seconds_remaining != game.state.secondary_seconds_remaining or \
       game.state.seconds_remaining <= 0:
        update_state_display()
    if previous_seconds_remaining != game.state.seconds_remaining:
        if game.interruption_seconds is not None:
            if game.interruption_seconds - game.state.seconds_remaining > IN_PLAY_TIMEOUT:
                info('Ball in play.')
                game.in_play = True
                game.interruption = None
                game.interruption_step = None
                game.interruption_team = None
                game.interruption_seconds = None
            update_state_display()
        update_time_display()
    red = 0 if game.state.teams[0].team_color == 'RED' else 1
    blue = 1 if red == 0 else 0
    if previous_red_score != game.state.teams[red].score or \
       previous_blue_score != game.state.teams[blue].score:
        update_score_display()
    # print(game.state.game_state)
    secondary_state = game.state.secondary_state
    secondary_state_info = game.state.secondary_state_info
    if secondary_state[0:6] == 'STATE_' and secondary_state[6:] in GAME_INTERRUPTIONS:
        kick = secondary_state[6:]
        step = secondary_state_info[1]
        if step == 0 and game.interruption_step != step:
            game.interruption_step = step
            info(f'Awarding a {GAME_INTERRUPTIONS[kick]}.')
        elif step == 1 and game.state.secondary_seconds_remaining <= 0 and game.interruption_step != step:
            game.interruption_step = step
            game_controller_send(f'{kick}:{secondary_state_info[0]}:PREPARE')
            info(f'Prepare for {GAME_INTERRUPTIONS[kick]}.')
        elif step == 2 and game.state.secondary_seconds_remaining <= 0 and game.interruption_step != step:
            game.interruption_step = step
            game_controller_send(f'{kick}:{secondary_state_info[0]}:EXECUTE')
            info(f'Execute {GAME_INTERRUPTIONS[kick]}.')
            game.interruption_seconds = game.state.seconds_remaining
    elif secondary_state not in ['STATE_NORMAL', 'STATE_OVERTIME']:
        print(f'GameController {secondary_state}: {secondary_state_info}')


def game_controller_send(message):
    if message[:6] == 'STATE:':
        # we don't want to send twice the same STATE message
        if game_controller_send.sent_once == message:
            return False
        game_controller_send.sent_once = message
    game_controller_send.id += 1
    if message[:6] != 'CLOCK:':
        info(f'Sending {game_controller_send.id}:{message} to GameController.')
    message = f'{game_controller_send.id}:{message}\n'
    game.controller.sendall(message.encode('ascii'))
    # info(f'sending {message.strip()} to GameController')
    game_controller_send.unanswered[game_controller_send.id] = message.strip()
    answered = False
    sent_id = game_controller_send.id
    while True:
        try:
            answers = game.controller.recv(1024).decode('ascii').split('\n')
            for answer in answers:
                if answer == '':
                    continue
                try:
                    id, result = answer.split(':')
                    if int(id) == sent_id:
                        answered = True
                except ValueError:
                    error(f'Cannot split {answer}')
                try:
                    answered_message = game_controller_send.unanswered[int(id)]
                    del game_controller_send.unanswered[int(id)]
                except KeyError:
                    error(f'Received acknowledgment message for unknown message: {id}')
                    continue
                if result == 'OK':
                    continue
                if result == 'INVALID':
                    error(f'Received invalid answer from GameController for message {answered_message}.')
                elif result == 'ILLEGAL':
                    error(f'Received illegal answer from GameController for message {answered_message}.')
                else:
                    error(f'Received unknown answer from GameController: {answer}.')
        except BlockingIOError:
            if not game.game_controller_synchronization:
                break
            elif answered or ':CLOCK:' in message:
                break
            else:  # keep sending CLOCK messages to keep the GameController happy
                info(f'Waiting for GameController to answer to {message.strip()}.')
                time.sleep(0.2)
                game_controller_send.id += 1
                clock_message = f'{game_controller_send.id}:CLOCK:{time_count}\n'
                game.controller.sendall(clock_message.encode('ascii'))
                game_controller_send.unanswered[game_controller_send.id] = clock_message.strip()

    return True


game_controller_send.id = 0
game_controller_send.unanswered = {}
game_controller_send.sent_once = None


def distance2(v1, v2):
    return math.sqrt((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2)


def distance3(v1, v2):
    return math.sqrt((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2 + (v1[2] - v2[2]) ** 2)


def point_inside_field(point):
    if point[2] > game.turf_depth:  # in the air
        return False
    if point[0] > game.field_size_x or point[0] < -game.field_size_x or \
       point[1] > game.field_size_y or point[1] < -game.field_size_y:
        return False
    return True


def rotate_along_z(axis_and_angle):
    q = transforms3d.quaternions.axangle2quat([axis_and_angle[0], axis_and_angle[1], axis_and_angle[2]], axis_and_angle[3])
    rz = [0, 0, 0, 1]
    # rz = [0, 0, 0.7071068, 0.7071068]
    r = transforms3d.quaternions.qmult(q, rz)
    v, a = transforms3d.quaternions.quat2axangle(r)
    return [v[0], v[1], v[2], a]


def append_solid(solid, solids):  # we list only the hands and feet
    model_field = solid.getField('model')
    if model_field:
        model = model_field.getSFString()
        suffix = model[-4:]
        if suffix == 'hand' or suffix == 'foot':
            solids.append(solid)
    children = solid.getProtoField('children') if solid.isProto() else solid.getField('children')
    for i in range(0, children.getCount()):
        child = children.getMFNode(i)
        if child.getType() in [Node.ROBOT, Node.SOLID, Node.GROUP, Node.TRANSFORM, Node.ACCELEROMETER, Node.CAMERA, Node.GYRO,
                               Node.TOUCH_SENSOR]:
            append_solid(child, solids)
            continue
        if child.getType() in [Node.HINGE_JOINT, Node.HINGE_2_JOINT, Node.SLIDER_JOINT, Node.BALL_JOINT]:
            endPoint = child.getProtoField('endPoint') if child.isProto() else child.getField('endPoint')
            solid = endPoint.getSFNode()
            if solid.getType() == Node.NO_NODE:
                continue
            append_solid(solid, solids)


def list_team_solids(team):
    for number in team['players']:
        player = team['players'][number]
        robot = player['robot']
        player['solids'] = []
        solids = player['solids']
        append_solid(robot, solids)
        if len(solids) != 4:
            color = 'red' if team == red_team else 'blue'
            error(f'{color} player {number} is missing a hand or a foot.')


def list_solids():
    list_team_solids(red_team)
    list_team_solids(blue_team)


def area(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)


def point_in_triangle(m, a, b, c):
    abc = area(a[0], a[1], b[0], b[1], c[0], c[1])
    mbc = area(m[0], m[1], b[0], b[1], c[0], c[1])
    amc = area(a[0], a[1], m[0], m[1], c[0], c[1])
    amb = area(a[0], a[1], b[0], b[1], m[0], m[1])
    if abc == mbc + amc + amb:
        return True
    else:
        return False


def aabb_circle_collision(aabb, x, y, radius):
    if x + radius < aabb[0]:
        return False
    if x - radius > aabb[2]:
        return False
    if y + radius < aabb[1]:
        return False
    if y - radius > aabb[3]:
        return False
    return True


def segment_circle_collision(p1, p2, center, radius):
    len = distance2(p1, p2)
    dx = (p2[0] - p1[0]) / len
    dy = (p2[1] - p1[1]) / len
    t = dx * (center[0] - p1[0]) + dy * (center[1] - p1[1])
    e = [t * dx + p1[0], t * dy + p1[1]]  # projection of circle center onto the segment
    d = distance2(e, center)
    return d < radius


def triangle_circle_collision(p1, p2, p3, center, radius):
    if distance2(p1, center) < radius or distance2(p2, center) < radius or distance2(p3, center) < radius:
        return True
    if point_in_triangle(center, p1, p2, p3):
        return True
    return segment_circle_collision(p1, p2, center, radius) \
        or segment_circle_collision(p1, p3, center, radius) \
        or segment_circle_collision(p2, p3, center, radius)


def update_team_convex_hulls(team):
    for number in team['players']:
        player = team['players'][number]
        player['shadow'] = []
        if 'aabb' in player:
            del player['aabb']
        shadow = player['shadow']
        for solid in player['solids']:
            position = solid.getPosition()
            shadow.append([position[0], position[1]])
            if 'aabb' not in player:
                player['aabb'] = [position[0], position[1], position[0], position[1]]
            else:
                if position[0] < player['aabb'][0]:
                    player['aabb'][0] = position[0]
                elif position[0] > player['aabb'][2]:
                    player['aabb'][2] = position[0]
                if position[1] < player['aabb'][1]:
                    player['aabb'][1] = position[1]
                elif position[1] > player['aabb'][3]:
                    player['aabb'][3] = position[1]
        # check AABB for ball collision
        if not aabb_circle_collision(player['aabb'], game.ball_position[0], game.ball_position[1], game.ball_radius):
            hold_ball = False
        else:  # compute convex hull of quad
            if point_in_triangle(shadow[0], shadow[1], shadow[2], shadow[3]):
                del shadow[0]
            if point_in_triangle(shadow[1], shadow[0], shadow[2], shadow[3]):
                del shadow[1]
            if point_in_triangle(shadow[2], shadow[0], shadow[1], shadow[3]):
                del shadow[2]
            if point_in_triangle(shadow[3], shadow[0], shadow[1], shadow[2]):
                del shadow[3]
            if len(shadow) == 3:
                hold_ball = triangle_circle_collision(shadow[0], shadow[1], shadow[2], game.ball_position, game.ball_radius)
            else:
                hold_ball = triangle_circle_collision(shadow[0], shadow[1], shadow[2], game.ball_position, game.ball_radius) \
                         or triangle_circle_collision(shadow[3], shadow[1], shadow[2], game.ball_position, game.ball_radius)
        color = 'Red' if team == red_team else 'Blue'
        if 'hold_ball' in player:
            if hold_ball:
                continue
            delay = int((time_count - player['hold_ball']) / 100) / 10
            info(f'{color} player {number} released the ball after {delay} seconds.')
            del player['hold_ball']
        elif hold_ball:
            player['hold_ball'] = time_count
            info(f'{color} player {number} is holding the ball.')


def update_convex_hulls():
    update_team_convex_hulls(red_team)
    update_team_convex_hulls(blue_team)


def update_team_contacts(team, color):
    for number in team['players']:
        player = team['players'][number]
        robot = player['robot']
        n = robot.getNumberOfContactPoints(True)
        player['contact_points'] = []
        if n == 0:
            continue
        player['outside_circle'] = True    # true if fully outside the center cicle
        player['outside_field'] = True     # true if fully outside the field
        player['inside_field'] = True      # true if fully inside the field
        player['inside_own_side'] = True   # true if fully inside its own side (half field side)
        fallen = False
        for i in range(0, n):
            point = robot.getContactPoint(i)
            if point[2] > game.turf_depth:  # not a contact with the ground
                if point in game.ball.contact_points:  # ball contact
                    if game.ball_last_touch_team != color or game.ball_last_touch_player != int(number):
                        game.ball_last_touch_team = color
                        game.ball_last_touch_player = int(number)
                        game.ball_last_touch_time = time_count
                        info(f'Ball touched by {color} player {number}.')
                    elif time_count - game.ball_last_touch_time >= 1000:  # dont produce too many touched messages
                        game.ball_last_touch_time = time_count
                        info('Ball touched again by same player.')
                    continue
                # the robot touched something else than the ball or the ground
                player['contact_points'].append(point)  # this list will be checked later for robot-robot collisions
                continue
            if distance3(point, [0, 0, game.turf_depth]) < game.center_circle_radius:
                player['outside_circle'] = False
            if point_inside_field(point):
                player['outside_field'] = False
            else:
                player['inside_field'] = False
            if game.side_left == (game.red.id if color == 'red' else game.blue.id):
                if point[0] > -LINE_HALF_WIDTH:
                    player['inside_own_side'] = False
            else:
                if point[0] < LINE_HALF_WIDTH:
                    player['inside_own_side'] = False
            # check if the robot has fallen
            node = robot.getContactPointNode(i)
            if not node:
                continue
            model_field = node.getField('model')
            if model_field is None:
                continue
            model = model_field.getSFString()
            if model[-4:] == 'foot':
                continue
            if 'fallen' in player:  # was already down
                fallen = True
                continue
            info(f'{color} player {number} has fallen down.')
            player['fallen'] = time_count
        if not fallen and 'fallen' in player:  # the robot has recovered
            delay = (int((time_count - player['fallen']) / 100)) / 10
            info(f'{color} player {number} just recovered after {delay} seconds.')
            del player['fallen']


def update_ball_contacts():
    game.ball.contact_points = []
    for i in range(0, game.ball.getNumberOfContactPoints()):
        point = game.ball.getContactPoint(i)
        if point[2] <= 0.01:  # contact with the ground
            continue
        game.ball.contact_points.append(point)
        break


def update_contacts():
    update_ball_contacts()
    update_team_contacts(red_team, 'red')
    update_team_contacts(blue_team, 'blue')


def check_team_ball_holding(team):
    for number in team['players']:
        player = team['players'][number]
        if 'hold_ball' in player:
            color = 'Red' if team == red_team else 'Blue'
            dt = (time_count - player['hold_ball']) / 1000
            if number == '1':
                if dt > GOAL_KEEPER_BALL_HOLDING_TIMEOUT:
                    del player['hold_ball']
                    info(f'{color} player {number} (goal keeper) has held the ball for too long.')
                    return True
            elif dt > PLAYER_BALL_HOLDING_TIMEOUT:
                del player['hold_ball']
                info(f'{color} player {number} has held the ball for too long.')
                return True
    return False


def check_ball_holding():  # return the team id which gets a free kick in case of ball holding from the other team
    if check_team_ball_holding(red_team):
        return game.blue.id
    if check_team_ball_holding(blue_team):
        return game.red.id
    return None


def check_team_fallen(team, color):
    penalty = False
    for number in team['players']:
        if game.state.teams[team_index(color)].players[int(number) - 1].secs_till_unpenalized > 0:
            continue  # skip penalized players
        player = team['players'][number]
        if 'fallen' in player and time_count - player['fallen'] > 1000 * FALLEN_TIMEOUT:
            del player['fallen']
            info(f'{color} player {number} has fallen down and didn\'t recover in the last 20 seconds.')
            player['penalty'] = 'INCAPABLE'
            player['penalty_reason'] = 'fallen down'
            penalty = True
    return penalty


def check_fallen():
    red = check_team_fallen(red_team, 'red')
    blue = check_team_fallen(blue_team, 'blue')
    return red or blue


def check_team_start_position(team, color):
    penalty = False
    for number in team['players']:
        player = team['players'][number]
        if not player['outside_field']:
            player['penalty'] = 'INCAPABLE'
            player['penalty_reason'] = 'halfTimeStartingPose inside field'
            penalty = True
        elif not player['inside_own_side']:
            player['penalty'] = 'INCAPABLE'
            player['penalty_reason'] = 'halfTimeStartingPose outside team side'
            penalty = True
    return penalty


def check_start_position():
    red = check_team_start_position(red_team, 'red')
    blue = check_team_start_position(blue_team, 'blue')
    return red or blue


def check_team_kickoff_position(team, color):
    team_id = game.red.id if color == 'red' else game.blue.id
    penalty = False
    for number in team['players']:
        if game.state.teams[team_index(color)].players[int(number) - 1].secs_till_unpenalized > 0:
            continue  # skip penalized players
        player = team['players'][number]
        if not player['inside_field']:
            player['penalty'] = 'INCAPABLE'
            player['penalty_reason'] = 'outside of field at kickoff'
            penalty = True
        elif not player['inside_own_side']:
            player['penalty'] = 'INCAPABLE'
            player['penalty_reason'] = 'outside team side at kickoff'
            penalty = True
        elif game.kickoff != team_id and not player['outside_circle']:
            player['penalty'] = 'INCAPABLE'
            player['penalty_reason'] = 'inside center circle during oppenent\'s kickoff'
            penalty = True
    return penalty


def check_kickoff_position():
    red = check_team_kickoff_position(red_team, 'red')
    blue = check_team_kickoff_position(blue_team, 'blue')
    return red or blue


def check_team_penalized_in_field(team, color):
    penalty = False
    for number in team['players']:
        if game.state.teams[team_index(color)].players[int(number) - 1].secs_till_unpenalized == 0:
            continue  # skip non penalized players
        player = team['players'][number]
        if player['outside_field']:
            continue
        info(f'Penalized {color} player {number} re-entered the field: shown yellow card.')
        game_controller_send(f'CARD:{game.red.id if color == "red" else game.blue.id}:{number}:YELLOW')
        player['penalty'] = 'INCAPABLE'
        player['penalty_reason'] = 'penalized player re-entered field'
        penalty = True
    return penalty


def check_penalized_in_field():
    red = check_team_penalized_in_field(red_team, 'red')
    blue = check_team_penalized_in_field(blue_team, 'blue')
    return red or blue


def send_team_penalties(team, color):
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
            t = copy.deepcopy(team['players'][number]['reentryStartingPose']['translation'])
            r = copy.deepcopy(team['players'][number]['reentryStartingPose']['rotation'])
            t[0] = game.field_penalty_mark_x if t[0] > 0 else -game.field_penalty_mark_x
            if (game.ball_position[1] > 0 and t[1] > 0) or (game.ball_position[1] < 0 and t[1] < 0):
                t[1] = -t[1]
                r = rotate_along_z(r)
            # check if position is already occupied by a penalized robot
            while True:
                moved = False
                for n in team['players']:
                    other_robot = team['players'][n]['robot']
                    other_t = other_robot.getField('translation').getSFVec3f()
                    if distance3(other_t, t) < game.robot_radius:
                        t[0] += game.penalty_offset if game.ball_position[0] < t[0] else -game.penalty_offset
                        moved = True
                if not moved:
                    break
            # test if position is behind the goal line (note: it should never end up beyond the center line)
            if t[0] > game.field_size_x:
                t[0] -= 4 * game.penalty_offset
            elif t[0] < -game.field_size_x:
                t[0] += 4 * game.penalty_offset
            translation.setSFVec3f(t)
            rotation.setSFRotation(r)
            info(f'{penalty} penalty for {color} player {number}: {reason}. Sent to ' +
                 f'translation ({t[0]} {t[1]} {t[2]}), rotation ({r[0]} {r[1]} {r[2]} {r[3]}).')


def send_penalties():
    send_team_penalties(red_team, 'red')
    send_team_penalties(blue_team, 'blue')


def flip_pose(pose):
    pose['translation'][0] = -pose['translation'][0]
    pose['rotation'][3] = math.pi - pose['rotation'][3]


def flip_sides():
    game.side_left = 2 if game.side_left == 1 else 1  # flip sides (no need to notify GameController, it does it automatically)
    for team in [red_team, blue_team]:
        for number in team['players']:
            player = team['players'][number]
            flip_pose(player['halfTimeStartingPose'])
            flip_pose(player['reentryStartingPose'])
            flip_pose(player['shootoutStartingPose'])


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
         f'translation ({t[0]} {t[1]} {t[2]}), rotation ({r[0]} {r[1]} {r[2]} {r[3]}).')


def reset_teams(pose):
    for number in red_team['players']:
        reset_player('red', number, pose)
    for number in blue_team['players']:
        reset_player('blue', number, pose)


def interruption(type, team=None):
    game.in_play = False
    game.interruption = type
    game.interruption_countdown = int(SIMULATED_TIME_BEFORE_INTERRUPTION * 1000 / time_step)
    if not team:
        game.interruption_team = game.red.id if game.ball_last_touch_team == 'blue' else game.blue.id
    else:
        game.interruption_team = team
    color = 'red' if game.interruption_team == game.red.id else 'blue'
    info(f'{GAME_INTERRUPTIONS[type].capitalize()} awarded to {color} team.')
    game_controller_send(f'{game.interruption}:{game.interruption_team}')


def throw_in(left_side):
    # set the ball on the touch line for throw in
    sign = -1 if left_side else 1
    game.ball_exit_translation[1] = sign * (game.field_size_y - LINE_HALF_WIDTH)
    interruption('THROWIN')


def corner_kick(left_side):
    # set the ball in the right corner for corner kick
    sign = -1 if left_side else 1
    game.ball_exit_translation[0] = sign * (game.field_size_x - LINE_HALF_WIDTH)
    game.ball_exit_translation[1] = game.field_size_y - LINE_HALF_WIDTH if game.ball_exit_translation[1] > 0 \
        else -game.field_size_y + LINE_HALF_WIDTH
    interruption('CORNERKICK')


def goal_kick():
    # set the ball at intersection between the centerline and touchline
    game.ball_exit_translation[0] = 0
    game.ball_exit_translation[1] = game.field_size_y - LINE_HALF_WIDTH if game.ball_exit_translation[1] > 0 \
        else -game.field_size_y + LINE_HALF_WIDTH
    interruption('GOALKICK')


time_count = 0

log_file = open('log.txt', 'w')

# determine configuration file name
game_config_file = os.environ['WEBOTS_ROBOCUP_GAME'] if 'WEBOTS_ROBOCUP_GAME' in os.environ \
    else os.path.join(os.getcwd(), 'game.json')
if not os.path.isfile(game_config_file):
    error(f'Cannot read {game_config_file} game config file.')

# read configuration files
with open(game_config_file) as json_file:
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
            with open(path, 'w') as file:
                file.write((red_line + blue_line) if game.red.id < game.blue.id else (blue_line + red_line))
            game.controller_process = subprocess.Popen(
              [os.path.join(JAVA_HOME, 'bin', 'java'), '-jar', 'GameControllerSimulator.jar', '--config', game_config_file],
              cwd=os.path.join(GAME_CONTROLLER_HOME, 'build', 'jar'))
    except KeyError:
        GAME_CONTROLLER_HOME = None
        game.controller_process = None
        error('GAME_CONTROLLER_HOME environment variable not set, unable to launch GameController.')
except KeyError:
    JAVA_HOME = None
    GAME_CONTROLLER_HOME = None
    game.controller_process = None
    error('JAVA_HOME environment variable not set, unable to launch GameController.')

# finalize the game object
if game.type not in ['NORMAL', 'KNOCKOUT', 'PENALTY']:
    error(f'Unsupported game type: {game.type}.')
if not hasattr(game, 'real_time_factor'):
    game.real_time_factor = 3  # simulation speed defaults to 1/3 of real time, e.g., 0.33x real time in the Webots speedometer
if not hasattr(game, 'press_a_key_to_terminate'):
    game.press_a_key_to_terminate = False
if not hasattr(game, 'game_controller_synchronization'):
    game.game_controller_synchronization = False
message = f'Real time factor is set to {game.real_time_factor}.'
if game.real_time_factor == 0:
    message += ' Simulation will run as fast as possible, real time waiting times will be minimised.'
else:
    message += f' Simulation will run at {1/game.real_time_factor:.2f}x, real time waiting times will be respected.'
info(message)
game.field_size_y = 3 if field_size == 'kid' else 4.5
game.field_size_x = 4.5 if field_size == 'kid' else 7
game.field_penalty_mark_x = 3 if field_size == 'kid' else 4.9
game.penalty_offset = 0.6 if field_size == 'kid' else 1
game.center_circle_radius = 0.75 if field_size == 'kid' else 1.5
game.robot_radius = 0.3 if field_size == 'kid' else 0.5
game.goal_height = GOAL_HEIGHT_KID if field_size == 'kid' else GOAL_HEIGHT_ADULT
game.ball_radius = 0.07 if field_size == 'kid' else 0.1125
game.turf_depth = 0.01
game.ball_kickoff_translation = [0, 0, game.ball_radius + game.turf_depth]

toss_a_coin_if_needed('side_left')
toss_a_coin_if_needed('kickoff')

# start the webots supervisor
supervisor = Supervisor()
root = supervisor.getRoot()
children = root.getField('children')

if hasattr(game, 'supervisor'):  # optional supervisor used for CI tests
    children.importMFNodeFromString(-1, f'DEF TEST_SUPERVISOR Robot {{ supervisor TRUE controller "{game.supervisor}" }}')

children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "{field_size}" }}')
ball_size = 1 if field_size == 'kid' else 5
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 {game.ball_kickoff_translation[2]} ' +
                                f'size {ball_size} }}')

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
    info('Connected to GameControllerSimulator at localhost:8750.')
    game.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    game.udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    game.udp.bind(('0.0.0.0', 3838))
    game.udp.setblocking(False)
else:
    game.controller = None

update_state_display()

list_solids()  # prepare lists of solids to monitor in each robot to compute the convex hulls

info(f'Game type is {game.type}.')
info(f'Red team is {red_team["name"]}.')
info(f'Blue team is {blue_team["name"]}.')
info(f'Left side is {"red" if game.side_left == game.red.id else "blue"}.')
info(f'Kickoff is {"red" if game.kickoff == game.red.id else "blue"}.')
info('Beginning of first half.')
game_controller_send(f'SIDE_LEFT:{game.side_left}')
game_controller_send(f'KICKOFF:{game.kickoff}')

game.ball = supervisor.getFromDef('BALL')
game.ball_translation = supervisor.getFromDef('BALL').getField('translation')
game.ball_exit_translation = None
game.ball_last_touch_team = None
game.ball_last_touch_player = None
game.real_time_multiplier = 1000 / (game.real_time_factor * time_step) if game.real_time_factor > 0 else 10
game.in_play = False
game.interruption = None
game.interruption_countdown = 0
game.interruption_step = None
game.interruption_team = None
game.interruption_seconds = None
game.overtime = False
game.ready_countdown = (int)(REAL_TIME_BEFORE_FIRST_READY_STATE * game.real_time_multiplier)
game.play_countdown = 0
game.sent_finish = False
game.over = False
previous_seconds_remaining = 0
real_time_start = time.time()
while supervisor.step(time_step) != -1:
    game_controller_send(f'CLOCK:{time_count}')
    game_controller_receive()
    if game.state is None:
        time_count += time_step
        continue
    game.ball_position = game.ball_translation.getSFVec3f()
    update_contacts()  # check for collisions with the ground and ball
    update_convex_hulls()  # badly affects the performance (drop from 5x to 0.5x)
    if game.state.game_state == 'STATE_PLAYING':
        if previous_seconds_remaining != game.state.seconds_remaining:
            update_state_display()
            previous_seconds_remaining = game.state.seconds_remaining
            if not game.sent_finish and game.state.seconds_remaining <= 0:
                game_controller_send('STATE:FINISH')
                game.sent_finish = True
                if game.state.first_half:
                    type = 'knockout ' if game.type == 'KNOCKOUT' and game.overtime else ''
                    info(f'End of {type}first half.')
                    flip_sides()
                    reset_teams('halfTimeStartingPose')
                    update_team_display()
                elif game.type == 'NORMAL':
                    info('End of second half.')
                elif game.type == 'KNOCKOUT':
                    if not game.overtime:
                        info('End of second half.')
                        flip_sides()
                        reset_teams('halfTimeStartingPose')
                        game.overtime = True
                    else:
                        info('End of knockout second half.')
                elif game.type == 'PENALTY':
                    warning('PENALTY game not yet supported!')  # FIXME
                else:
                    error(f'Unsupported game type: {game.type}.')
        if game.interruption_countdown == 0 and \
            (game.ball_position[1] - game.ball_radius > game.field_size_y or
             game.ball_position[1] + game.ball_radius < -game.field_size_y or
             game.ball_position[0] - game.ball_radius > game.field_size_x or
             game.ball_position[0] + game.ball_radius < -game.field_size_x):
            info(f'Ball left the field at ({game.ball_position[0]} {game.ball_position[1]} {game.ball_position[2]}) after '
                 f'being touched by {game.ball_last_touch_team} player {game.ball_last_touch_player}.')
            game.ball_exit_translation = game.ball_position
            scoring_team = None
            right_way = None
            if game.ball_exit_translation[1] - game.ball_radius > game.field_size_y:
                game.ball_exit_translation[1] = game.field_size_y - LINE_HALF_WIDTH
                throw_in(left_side=False)
            elif game.ball_exit_translation[1] + game.ball_radius < -game.field_size_y:
                throw_in(left_side=True)
            if game.ball_exit_translation[0] - game.ball_radius > game.field_size_x:
                right_way = game.ball_last_touch_team == 'red' and game.side_left == game.red.id or \
                    game.ball_last_touch_team == 'blue' and game.side_left == game.blue.id
                if game.ball_exit_translation[1] < GOAL_HALF_WIDTH and \
                   game.ball_exit_translation[1] > -GOAL_HALF_WIDTH and game.ball_exit_translation[2] < game.goal_height:
                    scoring_team = game.side_left  # goal
                else:
                    if right_way:
                        goal_kick()
                    else:
                        corner_kick(left_side=False)
            elif game.ball_exit_translation[0] + game.ball_radius < -game.field_size_x:
                right_way = game.ball_last_touch_team == 'red' and game.side_left == game.blue.id or \
                    game.ball_last_touch_team == 'blue' and game.side_left == game.red.id
                if game.ball_exit_translation[1] < GOAL_HALF_WIDTH and \
                   game.ball_exit_translation[1] > -GOAL_HALF_WIDTH and game.ball_exit_translation[2] < game.goal_height:
                    # goal
                    scoring_team = game.red.id if game.blue.id == game.side_left else game.blue.id
                else:
                    if right_way:
                        goal_kick()
                    else:
                        corner_kick(left_side=True)
            if scoring_team:
                game.ball_exit_translation = game.ball_kickoff_translation
                goal = 'red' if scoring_team == game.blue.id else 'blue'
                game.kickoff = game.blue.id if scoring_team == game.red.id else game.red.id
                i = team_index(game.ball_last_touch_team)
                if game.state.teams[i].players[game.ball_last_touch_player - 1].secs_till_unpenalized == 0:
                    game_controller_send(f'SCORE:{scoring_team}')
                    info(f'Score in {goal} goal by {game.ball_last_touch_team} player {game.ball_last_touch_player}')
                    info(f'Kickoff is {"red" if game.kickoff == game.red.id else "blue"}')
                elif not right_way:  # own goal
                    game_controller_send(f'SCORE:{scoring_team}')
                    info(f'Score in {goal} goal by {game.ball_last_touch_team} player {game.ball_last_touch_player} (own goal)')
                    info(f'Kickoff is {"red" if game.kickoff == game.red.id else "blue"}')
                else:
                    info(f'Invalidated score in {goal} goal by penalized ' +
                         f'{game.ball_last_touch_team} player {game.ball_last_touch_player}')
                    goal_kick()

    elif game.state.game_state == 'STATE_READY':
        # the GameController will automatically change to the SET state once the state READY is over
        # the referee should wait a little time since the state SET started before sending the PLAY state
        game.play_countdown = int(SIMULATED_TIME_BEFORE_PLAY_STATE * 1000 / time_step)
        game.ball.resetPhysics()
        game.ball_translation.setSFVec3f(game.ball_kickoff_translation)
        game.checked_kickoff_position = False
    elif game.state.game_state == 'STATE_SET' and game.play_countdown > 0:
        if not game.checked_kickoff_position:
            check_kickoff_position()
            game.checked_kickoff_position = True
        game.play_countdown -= 1
        if game.play_countdown == 0:
            game_controller_send('STATE:PLAY')
    elif game.state.game_state == 'STATE_FINISHED':
        game.sent_finish = False
        if game.state.first_half:
            if game.ready_countdown == 0:
                if game.overtime:
                    type = 'knockout '
                    game_controller_send('STATE:OVERTIME-SECOND-HALF')
                else:
                    type = ''
                    game_controller_send('STATE:SECOND-HALF')
                info(f'Beginning of {type}second half.')
                game.ready_countdown = int(HALF_TIME_BREAK_SIMULATED_DURATION * game.real_time_multiplier)
        elif game.type == 'KNOCKOUT' and game.overtime and game.state.teams[0].score == game.state.teams[1].score:
            if game.ready_countdown == 0:
                info('Beginning of the knockout first half.')
                game_controller_send('STATE:OVERTIME-FIRST-HALF')
                game.ready_countdown = int(HALF_TIME_BREAK_SIMULATED_DURATION * game.real_time_multiplier)
        else:
            info('End of the game.')
            if game.state.teams[0].score > game.state.teams[1].score:
                winner = 0
                loser = 1
            else:
                winner = 1
                loser = 1
            info(f'The score is {game.state.teams[winner].score}-{game.state.teams[loser].score}.')
            if game.state.teams[0].score != game.state.teams[1].score:
                info(f'The winner is the {game.state.teams[winner].team_color.lower()} team.')
            else:
                info('This is a draw.')
                game.over = True
            break

    elif game.state.game_state == 'STATE_INITIAL':
        if game.ready_countdown > 0:
            game.ready_countdown -= 1
            if game.ready_countdown == 0:
                check_start_position()
                game_controller_send('STATE:READY')

    if game.interruption_countdown > 0:
        game.interruption_countdown -= 1
        if game.interruption_countdown == 0:
            if game.ball_exit_translation:
                game.ball.resetPhysics()
                game.ball_translation.setSFVec3f(game.ball_exit_translation)
                info('Ball respawned at '
                     f'{game.ball_exit_translation[0]} {game.ball_exit_translation[1]} {game.ball_exit_translation[2]}.')
            if game.interruption:
                game_controller_send(f'{game.interruption}:{game.interruption_team}:READY')

    check_fallen()                                # detect fallen robots

    if not game.interruption:
        ball_holding = check_ball_holding()       # check for ball holding fouls
        if ball_holding:
            interruption('DIRECT_FREEKICK', ball_holding)

    check_penalized_in_field()                    # check for penalized robots inside the field
    if game.state.game_state != 'STATE_INITIAL':  # send penalties if needed
        send_penalties()

    time_count += time_step

    if game.real_time_factor != 0:
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

if game.over and game.press_a_key_to_terminate:
    print('Press a key to terminate')
    keyboard = supervisor.getKeyboard()
    keyboard.enable(time_step)
    while supervisor.step(time_step) != -1:
        if keyboard.getKey() != -1:
            break

supervisor.simulationQuit(0)
while supervisor.step(time_step) != -1:
    pass
