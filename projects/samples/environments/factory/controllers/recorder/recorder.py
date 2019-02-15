"""recorder controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor

# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

orientation = supervisor.getFromDef('VIEWPOINT').getField('orientation')

while supervisor.step(timestep) != -1:
    orientation.setSFRotation([0, 1, 0, 0.2 * supervisor.getTime()])
    pass

# Enter here exit cleanup code.
