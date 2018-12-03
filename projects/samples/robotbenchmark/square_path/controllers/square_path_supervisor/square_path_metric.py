"""Class to handle the computation of the metric of the square path benchmark."""

from path_segment import PathSegment

# Size of one edge of the square, in meters.
SQUARE_EDGE_SIZE = 2

# Amount of time, in seconds, the robot is allowed to stop before the
# supervisor considers the robot has finished.
MAX_STOP_TIME = 2

# The maximum time, in seconds, for the robot to reach a goal.
# This is used to compute the time performance of a segment (if a robot takes more
# than that, the performance will be 0) and the total time of the simulation (the
# supervisor will end after four times this value has passed).
MAX_TIME = 20

# The maximum time, in seconds, for the robot to complete all the segments.
# By default this is based on the value of MAX_TIME.
MAX_TOTAL_TIME = 4 * MAX_TIME

# Weight of the different parts of the performance.
# The overall performance is a weighted average of the different parts.
TIME_WEIGHT = 0.5
DISTANCE_WEIGHT = 1
PATH_WEIGHT = 1

# The distance metric uses 2 components, one is linear and one is exponential.
# This is the weight of the linear component.
# The weight of the exponential component will be 1 - LINEAR_PART.
LINEAR_PART = 0.1


# Ratio between the coordinate of the world and the web interface
# canvas
WEB_INTERFACE_SCALE = 50


class SquarePathMetric(object):
    """Class used to handle the metric of the square path benchmark."""

    def __init__(self, storePoints):
        """
        Default constructor.

        storePoints: when True, points added with update will be stored in
        a buffer to be sent to the web interface canvas with the
        getWebNewPoints() method.
        """
        self.currentSegment = 0
        self.robotHasFinished = False
        self.storePoints = storePoints
        self.newPoints = []

        # Creates 4 instances of PathSegment for the 4 segments.
        self.segments = [
            PathSegment(SQUARE_EDGE_SIZE, (SQUARE_EDGE_SIZE, 0), 1, 1),
            PathSegment(SQUARE_EDGE_SIZE, (SQUARE_EDGE_SIZE, SQUARE_EDGE_SIZE), 2, 0),
            PathSegment(SQUARE_EDGE_SIZE, (0, SQUARE_EDGE_SIZE), 3, 1),
            PathSegment(SQUARE_EDGE_SIZE, (0, 0), 0, 0)
        ]

        for i in range(0, 4):
            self.segments[i].setPerformanceParam(LINEAR_PART,
                                                 MAX_TIME,
                                                 DISTANCE_WEIGHT,
                                                 PATH_WEIGHT,
                                                 TIME_WEIGHT)

    def update(self, point, angle, time):
        """
        Update the metric given the current position of the robot.

        point: tuple containing x and z coordinates of the robot.
        angle: orientation, in radian, of the robot.
        time: seconds elapsed since the beginning of the benchmark.
        """
        if time < MAX_TOTAL_TIME:

            if self.storePoints:
                self.newPoints.append(point)

            # Updates the position of the robot in the current segment.
            self.segments[self.currentSegment].update(point, angle, time)

            # Checks if robot has stopped.
            if (self.segments[self.currentSegment].timeStopped(time) >
               MAX_STOP_TIME):
                self.robotHasFinished = True

            # Checks if robot has reached the goal of the current segment.
            # If yes, switch to the next segment (unless we are in the last
            # one).
            if (self.segments[self.currentSegment].isGoalReached() and
               self.currentSegment < 3):
                self.currentSegment = self.currentSegment + 1
        else:
            self.robotHasFinished = True

    def updateLabels(self, labelColor, supervisor, time):
        """
        Update the labels displaying the metric.

        labelColor: integer value containing the color of the labels.
        supervisor: a reference to the supervisor instance to use.
        time: seconds elapsed since the beginning of the benchmark.
        """
        performance = 0

        # For each segment, add the performance to the total, and displays
        # corresponding label.
        for i in range(0, 4):
            performance = performance + self.segments[i].getPerformance()
            labelStr = "Segment %d: %.3f" % (i,
                                             self.segments[i].getPerformance())
            supervisor.setLabel(i, labelStr, 0.01, 0.05 + 0.02 * i,
                                0.05, labelColor, 0,
                                "Lucida Console")

        # Computes average.
        performance = performance / 4

        # Displays remaining labels.
        supervisor.setLabel(4, "Elapsed time: %.3f" % time, 0.01,
                            0.01, 0.05, labelColor, 0,
                            "Lucida Console")
        supervisor.setLabel(5, "Current performance: %f" % performance,
                            0.01, 0.15, 0.05, labelColor, 0,
                            "Lucida Console")

    def isBenchmarkOver(self):
        """
        Check if the benchmark is over.

        Return true if the benchmark is over, or false otherwise.
        """
        return self.robotHasFinished

    def getSegmentPerformance(self, segment):
        """
        Return the performance for a segment.

        segment: index of the segment
        """
        if (segment < 0 or segment > 3):
            return 0

        return self.segments[segment].getPerformance()

    def getPerformance(self):
        """Return the overall performance."""
        performance = 0

        # For each segment, adds the grade to the total, and displays
        # corresponding label.
        for i in range(0, 4):
            performance = performance + self.segments[i].getPerformance()

        # Computes average.
        performance = performance / 4

        return performance

    def getWebMetricUpdate(self):
        """Return a message to update the metric on the web interface."""
        result = ''
        performance = 0

        # For each segment, adds the performance to the total, and displays
        # corresponding label.
        for i in range(0, 4):
            performance = performance + self.segments[i].getPerformance()
            result = result + ("%.4f|" % self.segments[i].getPerformance())

        result = result + ("%.4f" % (performance / 4))

        return result

    def getWebNewPoints(self):
        """
        Return messages to update the canvas on the web interface.

        This method returns a list of message that can be sent to the web
        interface.

        Depending on how many times update() was called since the last
        time this method was called, the list may be empty or have several
        messages in it.

        Note: if you did not enable the storing of points in the constructor,
        this method will always return an empty list.
        """
        result = []

        for point in self.newPoints:
            result.append("point:%4.3d:%4.3d" %
                          (int(point[0] * WEB_INTERFACE_SCALE),
                           int(point[1] * WEB_INTERFACE_SCALE)))

        self.newPoints = []

        return result
