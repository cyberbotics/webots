import json
from controller import Supervisor

robot = Supervisor()

with open('red_config.json') as json_file:
    red_config = json.load(json_file)

with open('blue_config.json') as json_file:
    blue_config = json.load(json_file)

time_step = int(robot.getBasicTimeStep())

# spawing the robots
root = robot.getRoot()
children = root.getField('children')
for number in red_config:
    model = red_config[number]['proto']
    translation = red_config[number]['halfTimeStartingPose']['translation']
    rotation = red_config[number]['halfTimeStartingPose']['rotation']
    string = f'{model}{{name "red player {number}" ' + \
        f'translation {translation[0]} {translation[1]} {translation[2]} ' + \
        f'rotation {rotation[0]} {rotation[1]} {rotation[2]} {rotation[3]}}}'
    print(string)
    children.importMFNodeFromString(-1, string)
while robot.step(time_step) != -1:
    print('Hello World!')
