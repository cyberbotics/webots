"""Sample Webots controller for the humanoid marathon benchmark."""

# This controller uses built-in motion manager modules to get the OP2 to walk.

import os
import sys
from controller import Robot

try:
    pythonVersion = 'python%d%d' % (sys.version_info[0], sys.version_info[1])
    libraryPath = os.path.join(os.environ.get("WEBOTS_HOME"), 'projects', 'robots', 'robotis', 'darwin-op', 'libraries',
                               pythonVersion)
    libraryPath = libraryPath.replace('/', os.sep)
    sys.path.append(libraryPath)
    from managers import RobotisOp2GaitManager, RobotisOp2MotionManager
except ImportError:
    sys.stderr.write("Warning: 'managers' module not found.\n")
    sys.exit(0)

# Names of position sensors needed to get the corresponding device and read the measurements.
positionSensorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                       'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                       'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                       'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                       'FootR', 'FootL', 'Neck', 'Head')
# List of position sensor devices.
positionSensors = []

# Create the Robot instance.
robot = Robot()
basicTimeStep = int(robot.getBasicTimeStep())
# Initialize motion manager.
motionManager = RobotisOp2MotionManager(robot)
# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Retrieve devices.
headLed = robot.getLED('HeadLed')
eyeLed = robot.getLED('EyeLed')
gyro = robot.getGyro('Gyro')

# Enable all the position sensors and populate the 'positionSensor' list.
for i in range(0, len(positionSensorNames)):
    positionSensors.append(robot.getPositionSensor(positionSensorNames[i] + 'S'))
    positionSensors[i].enable(basicTimeStep)

# Initialize the LED devices.
headLed.set(0xff0000)
eyeLed.set(0xa0a0ff)
# Enable gyro device.
gyro.enable(basicTimeStep)

# Perform one simulation step to get sensors working properly.
robot.step(timestep)

# Page 1: stand up.
# Page 9: assume walking position.
motionManager.playPage(1, True)
motionManager.playPage(9, True)

# Initialize OP2 gait manager.
gaitManager = None
gaitManager = RobotisOp2GaitManager(robot, "")
gaitManager.start()
gaitManager.setXAmplitude(0.0)
gaitManager.setYAmplitude(0.0)
gaitManager.setBalanceEnable(True)
gaitAmplitude = 0.5
looptimes = 0

# Main loop: perform a simulation step until the simulation is over.
# At the beginning, start walking on the spot.
# After 45 timesteps, begin taking steps forward.
while robot.step(timestep) != -1:
    if looptimes == 45:
        gaitManager.setXAmplitude(gaitAmplitude)

    gaitManager.step(basicTimeStep)
    looptimes += 1
