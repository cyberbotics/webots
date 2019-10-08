"""Contains a class to handle a segment of the robot path."""

from math import sqrt
from math import exp


# Map between pairs of quarters and the vertex in between.
# This is used to detect when the robot has reached the
# next segment.
QUARTERS2VERTEX = [
    [None, 0, 0, 3],
    [0, None, 1, 1],
    [2, 1, None, 2],
    [3, 3, 2, None]
]


def getQuarter(side, point):
    """
    Given a 2d point, returns the corresponding quarter ID.

    The path of the robot is split in 4 segments, each segment corresponds
    to one edge of the square. Since the robot is not exactly on the edge, we
    need a way to map any point to one of the 4 edges. To do this we split the
    world into 4 quarters using the extended diagonals of the square. Each
    quarter corresponds to one edge of the square. The ID of each quarter
    corresponds to the order in which the robot will go through them.

    side: size of one side of the square, in meters.
    point: tuple containing x and y coordinates.

    returns 0, 1, 2 or 3 depending on the quarter in which the point is located
    """
    # Checks on which side of the bottom-left to top-right diagonal the point
    # is.
    posDiag = (point[1] - point[0] > 0)

    # Checks on which side of the top-left to bottom-right diagonal the point
    # is.
    negDiag = (point[0] + point[1] - side < 0)

    if posDiag:
        if negDiag:
            return 0
        else:
            return 3
    else:
        if negDiag:
            return 1
        else:
            return 2


class PathSegment(object):
    """
    Class used to represent one side of the square.

    It handles the performance of the robot for this segment.
    """

    def __init__(self, length, goal, goalID, orientation):
        """
        Default constructor.

        length: length of the segment, in meters.
        goal: tuple containing the 2d coordinates of the goal for this
        segment.
        goalID: ID of the vertex corresponding to the end of the segment
        orientation: 0 or 1 if the path is parallel to the X axis or
        the Y axis, respectively.
        """
        self.length = length
        self.goal = goal
        self.goalID = goalID
        self.orientation = orientation

        self.minPath = goal[orientation]
        self.maxPath = goal[orientation]
        self.currentPoint = None
        self.currentAngle = None
        self.currentTime = None
        self.startTime = None

        self.totalTime = None

        self.performance = 0
        self.goalReached = False

        self.performanceInit = False

    def setPerformanceParam(self, linearPart, maxTime,
                            distanceWeight, pathWeight, timeWeight):
        """
        Set the parameters used to compute the performance for this segment.

        This method must be called at least once or the performance will not
        be computed.

        linearPart: weight of the linear component of the distance part of
        the performance.
        maxTime: maximum time before the time part of the performance is 0, in
        seconds.
        distanceWeight: weight of the distance part in the performance.
        pathWeight: weight of the path part in the performance.
        timeWeight: weight of the time part in the performance.
        """
        self.linearPart = linearPart
        self.expPart = 1 - linearPart

        self.maxTime = maxTime

        self.distanceWeight = distanceWeight
        self.pathWeight = pathWeight
        self.timeWeight = timeWeight
        self.totalWeight = distanceWeight + pathWeight + timeWeight

        self.performanceInit = True

    def getPerformance(self):
        """Return the current performance for this segment."""
        return self.performance

    def isGoalReached(self):
        """
        Check if robot has reached the goal.

        Return True if the robot has already reached the goal,
        or False otherwise.
        """
        return self.goalReached

    def timeStopped(self, currentTime):
        """
        Return how much time since the robot has stopped.

        Returns the difference between the time supplied in argument, and the
        last time the position of the robot was updated, or 0 if the robot
        position was never updated.

        currentTime: current time in the simulation.
        """
        if self.currentTime is None:
            return 0
        return currentTime - self.currentTime

    def update(self, point, angle, time):
        """
        Update the position of the robot.

        Updates the position of the robot on this segment and computes a new
        performance accordingly.

        point: tuple with the coordinates of the robot.
        angle: orientation of the robot, in radian.
        time: current time of the simulation, in seconds.
        """
        # If the point is outside of the current corridor, enlarge the corridor
        # to include the point.
        if point[self.orientation] < self.minPath:
            self.minPath = point[self.orientation]
        elif point[self.orientation] > self.maxPath:
            self.maxPath = point[self.orientation]

        if self.startTime is None:
            self.startTime = time
            self.currentTime = time

        # If the robot position or orientation has changed, we need to update
        # the performance.
        if self.currentPoint != point or self.currentAngle != angle:

            # Checks if the robot has crossed a diagonal (=reached the goal).
            if self.currentPoint is not None:
                quarterBefore = getQuarter(self.length, self.currentPoint)
                quarterAfter = getQuarter(self.length, point)

                if QUARTERS2VERTEX[quarterBefore][quarterAfter] == self.goalID:
                    self.goalReached = True

            # Updates the position of the robot.
            self.currentPoint = point
            self.currentTime = time
            self.currentAngle = angle

            if self.performanceInit:

                # Computes distance performance.
                distX = self.goal[0] - self.currentPoint[0]
                distY = self.goal[1] - self.currentPoint[1]
                distance = (sqrt(distX**2 + distY**2) / self.length)
                distanceExp = exp(-6 * distance)
                distancePerformance = ((self.linearPart * -distance) +
                                       (self.expPart * distanceExp))

                # Computes path performance.
                pathPerformance = 1 - min((self.maxPath - self.minPath) / self.length, 1)

                # Computes time performance.
                self.totalTime = self.currentTime - self.startTime
                timePerformance = max(1 - (self.totalTime / self.maxTime), 0)

                # Computes average for this segment.
                self.performance = (distancePerformance * self.distanceWeight +
                                    pathPerformance * self.pathWeight +
                                    timePerformance * self.timeWeight) / self.totalWeight
