from abc import ABC, abstractmethod
from controller import Supervisor

import numpy as np

POS_ABS_TOL=0.03 # [m]

def target_to_def_name(target):
    return target.upper().replace(" ","_")

class StatusInformation:
    def __init__(self, system_time, simulated_time, gc_status):
        self.system_time = system_time
        self.simulated_time = simulated_time
        self.gc_status = gc_status

    def getFormattedTime(self):
        return "[{:08.3f}|{:08.3f}]".format(self.system_time, self.simulated_time)

class TimeSpecification(ABC):
    """
    Params
    ------
    properties: dictionary
        The properties of the given time specification
    """
    def __init__(self, properties):
       self._time = properties["time"]
       # TODO currently time specification based on gc_status are not available

    @abstractmethod
    def isActive(self, status):
        pass

    @abstractmethod
    def isFinished(self, status):
        pass

    def getCurrentTime(self, status):
        if self._clock_type == "Simulated":
            return status.simulated_time
        elif self._clock_type == "System":
            return status.system_time
        raise RuntimeError(f"Unexpected value for _clock_type: {self._clock_type}")

    def buildFromDictionary(properties):
       t = properties["time"]
       clock_type = properties.get("clock_type", "Simulated")
       if clock_type not in ["Simulated", "System"]:
           raise RuntimeError(f"clock_type: '{clock_type}' unknown")
       if isinstance(t,float) or isinstance(t,int):
           return TimePoint(t, clock_type)
       elif isinstance(t, list):
           if len(t) == 2:
               return TimeInterval(t[0], t[1], clock_type)
           raise RuntimeError(f"Invalid size for time: {len(t)}")
       raise RuntimeError("Invalid type for time")

class TimeInterval(TimeSpecification):
    def __init__(self, start, end, clock_type):
       self._start = start
       self._end = end
       self._clock_type = clock_type

    def isActive(self, status):
        t = self.getCurrentTime(status)
        return t > self._start and t < self._end

    def isFinished(self, status):
        t = self.getCurrentTime(status)
        return t > self._end

class TimePoint(TimeSpecification):
    def __init__(self, t, clock_type):
       self._t = t
       self._clock_type = clock_type

    def isActive(self, status):
        t = self.getCurrentTime(status)
        return t > self._t

    def isFinished(self, status):
        t = self.getCurrentTime(status)
        return t > self._t

class Test:
    def __init__(self, name, target = None, position = None, rotation = None,
                 state = None, penalty = None):
        self._name = name
        self._target = target
        self._position = position
        self._rotation = rotation
        self._state = state
        self._penalty = penalty
        self._msg = []
        self._success = True

    def perform(self, status, supervisor):
        if self._position is not None:
            self._testTargetPosition(status, supervisor)
        if self._rotation is not None:
            self._testTargetRotation(status, supervisor)
        if self._penalty is not None:
            self._testTargetPenalty(status, supervisor)
        if self._state is not None:
            self._testState(status, supervisor)

    def hasPassed(self):
        return self._success

    def printResult(self):
        if self._success:
            print(f"[PASS] Test {self._name}")
        else:
            print(f"[FAIL] Test {self._name}")
            for m in self._msg:
                print(f"\tCaused by {m}")

    def buildFromDictionary(dic):
        c = Test(dic["name"])
        c._target = dic.get("target")
        c._position = dic.get("position")
        c._rotation = dic.get("rotation")
        c._state = dic.get("state")
        c._penalty = dic.get("penalty")
        return c

    def _testTargetPosition(self, status, supervisor):
        if self._target is None:
            raise RuntimeError("{self._name} tests position and has no target")
        def_name = target_to_def_name(self._target)
        target = supervisor.getFromDef(def_name)
        received_pos = target.getField('translation').getSFVec3f()
        if not np.allclose(self._position, received_pos, atol = POS_ABS_TOL):
            failure_msg = f"Position Invalid at {status.getFormattedTime()}: "\
                f"expecting {self._position}, received {received_pos}"
            print(f"Failure in test {self._name}\n\t{failure_msg}")
            self._msg.append(failure_msg)
            self._success = False
            # Each test can only fail once to avoid spamming
            self._position = None

    def _testTargetRotation(self, status, supervisor):
        if self._target is None:
            raise RuntimeError("{self._name} tests position and has no target")
        # TODO Implement of rotation check is currently disabled, to be
        # properly introduced, it will require to be able to compare properly
        # the axis-angle obtained. Moreover, after reset, it does not seem that
        # the robot respects has the exact orientation used in pose

    def _testTargetPenalty(self, status, supervisor):
        if self._target is None:
            raise RuntimeError("{self._name} tests position and has no target")
        self._msg.append("Test of penalties is not implemented yet")
        self._penalty = None
        self._success = False

    def _testState(self, status, supervisor):
        self._msg.append("Test of state is not implemented yet")
        self._state = None
        self._success = False

class Action:

    def __init__(self, target, position = None, force = None, velocity = None):
        self._def_name = target_to_def_name(target)
        self._position = position
        self._force = force
        self._velocity = velocity

    def buildFromDictionary(dic):
        a = Action(dic["target"])
        a._position = dic.get("position")
        a._force = dic.get("force")
        a._velocity = dic.get("velocity")
        return a


    def perform(self, supervisor):
        obj = supervisor.getFromDef(self._def_name)
        obj.resetPhysics()
        if self._position is not None:
            self._setPosition(obj)
        if self._force is not None:
            self._setForce(obj)
        if self._velocity is not None:
            self._setVelocity(obj)

    def _setPosition(self, obj):
        print(f"Setting {self._def_name} to {self._position}")
        obj.getField("translation").setSFVec3f(self._position)

    def _setForce(self, obj):
        obj.addForce(self._force)

    def _setVelocity(self, obj):
        obj.setVelocity(self._velocity)



class Event:
    def __init__(self, time_spec, tests=[], actions=[], done=False):
        self._time_spec = time_spec
        self._tests = tests
        self._actions = actions
        self._done = done

    def isActive(self, status):
        return self._time_spec.isActive(status)

    def isFinished(self, status):
        #TODO an event might also been 'finished' if all tests have failed and
        # no action is there
        return self._time_spec.isFinished(status)

    """
    Applies tests and *afterwards* apply actions
    """
    def perform(self, status, supervisor):
        for c in self._tests:
            c.perform(status, supervisor)
        for a in self._actions:
            a.perform(supervisor)

    def getNbTests(self):
        return len(self._tests)

    """
    Return the number of tests that have not failed. It is the caller
    responsibility to make sure event is finished before calling this function.
    """
    def getNbTestsPassed(self):
        return sum([1 for c in self._tests if c.hasPassed()])

    def printResults(self):
        for c in self._tests:
            c.printResult()

    def buildFromDictionary(dic):
        tests_str = dic.get("tests")
        tests = []
        if tests_str is not None:
            for test_dic in tests_str:
                tests.append(Test.buildFromDictionary(test_dic))
        actions_str = dic.get("actions")
        actions = []
        if actions_str is not None:
            for action_dic in actions_str:
                actions.append(Action.buildFromDictionary(action_dic))
        event = Event(TimeSpecification.buildFromDictionary(dic["timing"]), tests, actions)
        # TODO build actions to add them
        return event


class Scenario:
    def __init__(self, events = []):
        self._waiting_events = events
        self._finished_events = []

    def step(self, status, supervisor):
        finished_events = []
        for i in range(len(self._waiting_events)):
            e = self._waiting_events[i]
            if e.isActive(status):
                e.perform(status, supervisor)
            if e.isFinished(status):
                finished_events.insert(0,i)
        for i in finished_events:
            self._finished_events.append(self._waiting_events.pop(i))

    def isFinished(self):
        return len(self._waiting_events) == 0

    def getNbTests(self):
        waiting = sum([e.getNbTests() for e in self._waiting_events])
        finished = sum([e.getNbTests() for e in self._finished_events])
        return waiting + finished

    def getNbTestsPassed(self):
        return sum([e.getNbTestsPassed() for e in self._finished_events])

    def printResults(self):
        for e in self._waiting_events:
            e.printResults()
        for e in self._finished_events:
            e.printResults()
        nb_tests = self.getNbTests()
        nb_tests_passed = self.getNbTestsPassed()
        print(f"TEST RESULTS: {nb_tests_passed}/{nb_tests}")

    def buildFromList(event_list):
        s = Scenario()
        for e in event_list:
            s._waiting_events.append(Event.buildFromDictionary(e))
        return s

