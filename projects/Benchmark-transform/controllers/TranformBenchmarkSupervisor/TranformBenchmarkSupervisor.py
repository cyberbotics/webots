from math import sqrt
from controller import Supervisor
import time
import sys

TIME_STEP = 4
num_sim = 1000
sim_time = 0
supervisor = Supervisor()

# get handle to robot's translation field
robot_node = supervisor.getFromDef("vehicle")
ball_node = supervisor.getFromDef("ball1")

if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")

trans_field_ball = ball_node.getField("translation")

# get handle to worldInfo's coordinate system field
worldInfo_node = supervisor.getFromDef("world_info")
CS_field = worldInfo_node.getField("coordinateSystem")
coordinateSystem = CS_field.getSFString()

for _ in range(0, num_sim):
    height = 3
    # reset robot position and physics
    if coordinateSystem == "ENU":
        init = [0, 0, 3]
        INITIAL_ball = [0,0,0.6]
    else:
        init = [0, 3, 0]
        INITIAL_ball = [0,0.6,0]

    trans_field.setSFVec3f(init)
    rotation_field.setSFRotation([0,0,1,0])
    trans_field_ball.setSFVec3f(INITIAL_ball)
    robot_node.resetPhysics()
    ball_node.resetPhysics()

    t = supervisor.getTime()
    while height > 0.6:
        if coordinateSystem == "ENU":
            height = trans_field.getSFVec3f()[2]
        else:
            height = trans_field.getSFVec3f()[1]

        # controller termination
        if supervisor.step(TIME_STEP) == -1:
            quit()
    sim_time += (supervisor.getTime() - t)
    
sim_time /= num_sim
print("Simulation time after {} simulations: {}".format(num_sim, sim_time))