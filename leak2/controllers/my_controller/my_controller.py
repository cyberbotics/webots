"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

rootNode = robot.getRoot()  # get root of the scene tree
rootChildrenField = rootNode.getField('children')

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    node_ref = robot.getFromDef('HUMAN')
    if (node_ref):
        node_ref.remove();
    else:
        rootChildrenField.importMFNodeFromString(-1, 'DEF HUMAN human_01_standing {}')
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    robot.step(timestep)

# Enter here exit cleanup code.
