"""square_path_supervisor controller."""

from controller import Supervisor
from square_path_metric import SquarePathMetric

import os
import sys
try:
    includePath = os.path.join(os.path.normpath(os.environ.get("WEBOTS_HOME")), 'projects', 'samples', 'robotbenchmark',
                               'include')
    sys.path.append(includePath)
    from robotbenchmark import robotbenchmarkRecord
except ImportError:
    sys.stderr.write("Warning: 'robotbenchmark' module not found.\n")
    sys.exit(0)


# Set to true to enable information displayed in labels.
ALLOW_LABELS = False

# The color used for the labels.
TEXT_COLOR = 0x0000ff


metric = SquarePathMetric(True)


# Create the Supervisor instance.
supervisor = Supervisor()

# Gets the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# Gets the reference to the robot.
pioneer = supervisor.getFromDef('PIONEER')


# Main loop starts here.
while (supervisor.step(timestep) != -1 and
       not metric.isBenchmarkOver()):

    # Recovers current time and position/orientation of the robot.
    pos = pioneer.getPosition()
    pos2d = [pos[0], -pos[1]]

    orientation = pioneer.getOrientation()
    time = supervisor.getTime()

    metric.update(pos2d, orientation, time)

    if ALLOW_LABELS:
        metric.updateLabels(0x0000ff, supervisor, time)

    supervisor.wwiSendText('update:' + metric.getWebMetricUpdate())

    for pointMessage in metric.getWebNewPoints():
        supervisor.wwiSendText(pointMessage)

supervisor.wwiSendText('stop')


# Wait for credentials sent by the robot window.
while supervisor.step(timestep) != -1:
    message = supervisor.wwiReceiveText()
    while message:
        if message.startswith("record:"):
            performance = metric.getPerformance()
            record = robotbenchmarkRecord(message, "square_path", performance)
            supervisor.wwiSendText(record)
        elif message == "exit":
            break
        message = supervisor.wwiReceiveText()

supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
