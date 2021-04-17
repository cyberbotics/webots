"""test_supervisor controller."""
from controller import Supervisor
from gamestate import GameState

import json
import os
import rc_testing
import socket
import time

class GCListener:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('0.0.0.0', 3838))
        self._socket.setblocking(False)
        self._state = None

    def receive(self):
        try:
            data, peer = self._socket.recvfrom(GameState.sizeof())
        except BlockingIOError:
            return
        except Exception as e:
            print(f'UDP input failure: {e}')
            pass
        if not data:
            print('No UDP data received')
            return
        self._state = GameState.parse(data)

    def getState(self):
        return self._state

scenario_config_file = os.environ['WEBOTS_ROBOCUP_TEST_SCENARIO']

supervisor = Supervisor()
system_start = time.time()

with open(scenario_config_file) as json_file:
    test_scenario = rc_testing.Scenario.buildFromList(json.load(json_file))

time_step = int(supervisor.getBasicTimeStep())

status = rc_testing.StatusInformation(0.0, 0.0, None)

gc_listener = GCListener()

finished = False

while supervisor.step(time_step) != -1 and not finished:
    gc_listener.receive()
    status.system_time = time.time() - system_start
    status.simulated_time += time_step/1000
    status.gc_status = gc_listener.getState()
    test_scenario.step(status, supervisor)

    if test_scenario.isFinished():
        print(f"{status.getFormattedTime()} END OF TESTING")
        test_scenario.printResults()
        finished = True

# TODO make exit status different if one of the tests did not pass

supervisor.simulationQuit(0)

# Enter here exit cleanup code.
