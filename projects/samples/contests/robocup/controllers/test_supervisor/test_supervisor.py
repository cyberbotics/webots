"""test_supervisor controller."""
from controller import Supervisor

import json
import os
import rc_testing
import time

scenario_config_file = os.environ['WEBOTS_ROBOCUP_TEST_SCENARIO']

supervisor = Supervisor()
system_start = time.time()

with open(scenario_config_file) as json_file:
    test_scenario = rc_testing.Scenario.buildFromList(json.load(json_file))

time_step = int(supervisor.getBasicTimeStep())

status = rc_testing.StatusInformation(0.0, 0.0, None)

finished = False

while supervisor.step(time_step) != -1 and not finished:
    status.system_time = time.time() - system_start
    status.simulated_time += time_step/1000
    test_scenario.step(status, supervisor)

    if test_scenario.isFinished():
        print(f"{status.getFormattedTime()} END OF TESTING")
        test_scenario.printResults()
        finished = True

# TODO make exit status different if one of the tests did not pass

supervisor.simulationQuit(0)

# Enter here exit cleanup code.
