"""Script to evaluate the performance of a real robot using the square_path metric."""

from square_metric import SquareMetric
import csv
import sys
import os.path

if len(sys.argv) < 2:
    print("Usage: python real_robot_performance.py <csv_file>")
    print("Example: python real_robot_performance.py sample.csv")
    sys.exit(0)

if not os.path.isfile(sys.argv[1]):
    print('"%s" is not a valid file! Please check the name and try again...' % sys.argv[1])
    sys.exit(0)

points = []

print("Parsing file...")
# Parsing input file
with open(sys.argv[1], "rb") as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if len(row) != 3:
            print("There should be exactly 3 elements per row, skipping incorrect row:")
            print(row)
        else:
            p = [0, 0, 0]
            valid = True

            for i in range(0, 3):
                try:
                    p[i] = float(row[i])
                except ValueError:
                    print("%s is not a valid number! Skipping incorrect row..." % row[i])
                    valid = False
            if valid:
                points.append(p)

metric = SquareMetric(False)

if not points:
    print("No points could be read from the input file. Aborting...")
else:
    print("Successfully read %d points from input file! Computing performance..." % len(points))

    orientation = 0

    # Main loop starts here.
    for point in points:

        # The orientation is only used to check when the benchmark is over.
        # In Webots we consider the benchmark over when the robot has stopped moving,
        # so we need to consider the orientation to avoid stopping the benchmark when
        # the robot is not changing position, but rotating.
        # Here we have a finite set of points so we can just run all the points and use
        # a dummy orientation that keeps incrementing.
        orientation = orientation + 1

        # Recovers current time and position of the robot.
        # Coordinates are swapped, because in Webots we have x and z instead of x and y for the
        # coordinates of the robot.
        pos2d = [point[1], point[0]]
        time = point[2]

        metric.update(pos2d, orientation, time)

    for i in range(0, 4):
        print("Performance for segment %d: %f" % (i + 1, metric.getSegmentPerformance(i)))

    print("Overall performance: %f" % metric.getPerformance())
