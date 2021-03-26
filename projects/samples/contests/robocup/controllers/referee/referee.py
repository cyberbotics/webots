import json
import math

from controller import Supervisor


def spawn_team(color, children):
    with open(color + '_config.json') as json_file:
        config = json.load(json_file)
        for number in config:
            model = config[number]['proto']
            translation = config[number]['halfTimeStartingPose']['translation']
            rotation = config[number]['halfTimeStartingPose']['rotation']
            if color == 'blue':  # symmetry with respect to the central line of the field
                translation[0] = -translation[0]
                rotation[3] = math.pi - rotation[3]
            string = f'{model}{{name "{color} player {number}" ' + \
                f'translation {translation[0]} {translation[1]} {translation[2]} ' + \
                f'rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]}}}'
            children.importMFNodeFromString(-1, string)


# spawing the teams
robot = Supervisor()
root = robot.getRoot()
children = root.getField('children')
spawn_team('red', children)
spawn_team('blue', children)

time_step = int(robot.getBasicTimeStep())
while robot.step(time_step) != -1:
    print('Hello World!')
