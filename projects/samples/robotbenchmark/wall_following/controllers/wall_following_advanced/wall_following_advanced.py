"""wall_following_advanced controller."""

from controller import Robot

from math import isinf


# Target distance from the wall, in meters.
MIN_WALL_THRESHOLD = 0.25
MAX_WALL_THRESHOLD = 0.5

# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAX_SPEED = 5.24

# Maximum range of the sensors.
# Don't change this value.
MAX_SENSOR_RANGE = 5

# Minimum distance for an obstacle to be considered as touching the robot.
HIT_WALL_DISTANCE = 0.01

# States used by our state machine.
ST_INITIAL = 0
ST_FORWARD = 1
ST_RIGHT = 2
ST_BACKWARD = 3

# Recovers robot instance.
robot = Robot()

# Recovers basic timestep.
timestep = int(robot.getBasicTimeStep())

# Initializes the wheels.
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))


# Initializes the sensors.
sonarSensor = [None] * 16
for i in range(0, 16):
    sonarSensor[i] = robot.getDevice("so%d" % i)
    sonarSensor[i].enable(timestep)


def setMotors(left, right):
    """Set the velocity value of both wheels."""
    leftWheel.setVelocity(left)
    rightWheel.setVelocity(right)


def getDistance(sensor):
    """Return the distance of an obstacle for a sensor."""
    # Special case, when the sensor doesn't detect anything, we use an
    # infinite distance.
    if sensor.getValue() == 0:
        return float("inf")
    return 5.0 * (1.0 - sensor.getValue() / 1024.0)


state = ST_INITIAL
distance = [None] * 16


def updateDistances():
    """
    Update the distances value of each sensor.

    The function will update the distance array with a value for each sensor that
    corresponds to how far is the closest obstacle detected by that sensor, or float("inf")
    if the sensor doesn't detect anything.
    """
    # We don't iterate over all the sensors because some are never used.
    # If you need to use those sensors you will have to add them here.
    for j in range(0, 7):
        distance[j] = getDistance(sonarSensor[j])
    distance[15] = getDistance(sonarSensor[15])


def handleInitial():
    """
    Handle the INITIAL state.

    This is the initial situation where we are far from a wall that is directly
    in front of us.
    We need to move forward until we are close enough to the wall.
    """
    setMotors(MAX_SPEED, MAX_SPEED)
    distanceToWall = (getDistance(sonarSensor[3]) +
                      getDistance(sonarSensor[4])) / 2

    if distanceToWall < (MAX_WALL_THRESHOLD + MIN_WALL_THRESHOLD) / 2:
        return ST_RIGHT


def handleForward():
    """
    Handle the FORWARD state.

    In this state we know there is a wall to our left, so we move forward
    and correct our path to follow the wall.
    """
    updateDistances()

    # We ran into a wall, we need to go backward.
    if (distance[2] <= HIT_WALL_DISTANCE or
       distance[3] <= HIT_WALL_DISTANCE or
       distance[4] <= HIT_WALL_DISTANCE or
       distance[5] <= HIT_WALL_DISTANCE):
        setMotors(0, 0)
        return ST_BACKWARD

    # Side sensor can't see the wall, we reached an edge.
    elif isinf(distance[0]) or distance[0] > (2 * distance[15]):

        # Are the front sensors seeing anything? If yes we need to turn right.
        if (distance[1] < MAX_WALL_THRESHOLD or
           distance[2] < MAX_WALL_THRESHOLD or
           distance[3] < MAX_WALL_THRESHOLD):
            setMotors(MAX_SPEED, MAX_SPEED / 2)
        # Otherwise we need to turn left.
        else:
            setMotors(MAX_SPEED * 0.5, MAX_SPEED)

    # There is a wall in front of us, we need to turn right.
    elif (distance[1] < MIN_WALL_THRESHOLD or
          distance[2] < MIN_WALL_THRESHOLD or
          distance[3] < MIN_WALL_THRESHOLD or
          distance[4] < MIN_WALL_THRESHOLD or
          distance[5] < MIN_WALL_THRESHOLD or
          distance[6] < MIN_WALL_THRESHOLD):
        setMotors(MAX_SPEED * 0.5, MAX_SPEED * 0.5)
        return ST_RIGHT

    # Too close to the wall, we need to turn right.
    elif distance[0] < MIN_WALL_THRESHOLD or distance[15] < MIN_WALL_THRESHOLD:
        # Special case to avoid division by 0 when we are very close.
        if distance[15] == 0:
            if distance[0] == 0:
                setMotors(MAX_SPEED, MAX_SPEED * 0.5)
            else:
                setMotors(MAX_SPEED, MAX_SPEED)
        # We are already heading away from the wall, no need to turn any further.
        elif distance[0] > distance[15]:
            setMotors(MAX_SPEED, MAX_SPEED)
        else:
            ratio = distance[0] / distance[15]
            setMotors(MAX_SPEED, MAX_SPEED * ratio)

    # Too far from the wall, we need to turn left.
    elif distance[0] > MIN_WALL_THRESHOLD:
        # We are already heading toward the wall, no need to turn any further.
        if distance[0] < distance[15]:
            setMotors(MAX_SPEED, MAX_SPEED)
        else:
            ratio = distance[15] / distance[0]
            setMotors(MAX_SPEED * ratio, MAX_SPEED)

    # We are in the right direction, or almost in the right direction.
    else:
        ratio = distance[15] / distance[0]
        absRatio = max(ratio, 1 / ratio)
        # A correction is needed.
        if absRatio > 1.01:
            # We need to correct to the right.
            if distance[15] > distance[0]:
                setMotors(MAX_SPEED, MAX_SPEED / absRatio)
            # We need to correct to the left.
            else:
                setMotors(MAX_SPEED / absRatio, MAX_SPEED)
        else:
            setMotors(MAX_SPEED, MAX_SPEED)


def handleRight():
    """
    Handle the RIGHT state.

    In this state we know there is a wall in front of us,
    and we need to make a clockwise rotation until the wall
    is to our left.
    """
    setMotors(MAX_SPEED / 2, -MAX_SPEED / 2)
    updateDistances()

    # Rotate until there is a wall to our left, and nothing in front of us.
    if (((distance[0] >= distance[15] and distance[15] < MAX_WALL_THRESHOLD) or
         (distance[15] > 2 * distance[0] and distance[0] < MAX_WALL_THRESHOLD)) and
       distance[1] > MAX_WALL_THRESHOLD and
       distance[2] > MAX_WALL_THRESHOLD and
       distance[3] > MAX_WALL_THRESHOLD and
       distance[4] > MAX_WALL_THRESHOLD and
       distance[5] > MIN_WALL_THRESHOLD and
       distance[6] > MIN_WALL_THRESHOLD):
        return ST_FORWARD


def handleBackward():
    """
    Handle the BACKWARD state.

    In this state we have run into a wall in front of us, we need to go backward
    until the wall is far enough.
    """
    setMotors(-MAX_SPEED, -MAX_SPEED)
    updateDistances()

    if (distance[2] > MIN_WALL_THRESHOLD and
       distance[3] > MIN_WALL_THRESHOLD and
       distance[4] > MIN_WALL_THRESHOLD and
       distance[5] > MIN_WALL_THRESHOLD):
        return ST_RIGHT


# Associates each state with a handler function.
# Handler functions should return the next state, or None if the state doesn't
# change.
switcher = {
    ST_INITIAL: handleInitial,
    ST_FORWARD: handleForward,
    ST_RIGHT: handleRight,
    ST_BACKWARD: handleBackward,
}


while robot.step(timestep) != -1:

    # Fetches the handler function for the current state.
    func = switcher.get(state, lambda: None)

    # Executes the handler function, and updates the state if necessary.
    result = func()
    if result is not None:
        state = result

# Stops the robot when we are done.
setMotors(0, 0)
