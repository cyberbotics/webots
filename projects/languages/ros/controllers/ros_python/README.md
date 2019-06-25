# ros_python

## Introduction

`ros_python` is a simple example demonstrating how you can write a Webots controller that implements a ROS node in Python.
It is a pure Python implementation, including all the necessary Python libraries, so that no ROS installation is needed on the machine running the Webots simulation.
Therefore, it runs on Windows, Linux and macOS, straight out of the box.

## Setup

In order to run this example, you will need to:

1. Run [roscore](https://wiki.ros.org/roscore) on some machine.
It may be the same machine running Webots or a different one.

2. If running on a different machine, you should edit the `runtime.ini` file to define the `ROS_MASTER_URI` value to point to your machine running `roscore`:
You will have to replace `localhost` with the IP address of the machine running `roscore`.

3. Set environment variables:
```
export ROS_MASTER_URI=http://localhost:11311  # replace this URI with the actual URI of the machine running roscore)
export ROS_PACKAGE_PATH=kinetic/share
export ROS_LOG_DIR=ros_controller_log
```

4. Launch `ros_controller.py`.

5. From a different terminal or the GUI, launch Webots, open the `ros_python.wbt` world file and run it.

6. You should see the Thymio II robot moving forward and stop before colliding with the wall.

## Explanation

`ros_python.py` is the Webots controller that is running a ROS node using the `rospy` ROS client library.
This controller actually publishes the value of its front distance sensor in a ROS topic named "sensor".
It also listens to the "motor" ROS topic.
Whenever it receives a value on this "motor" topic, it will apply it to the motors of the Thymio II robot.

`ros_controller.py` is a standalone ROS node that listens to the "sensor" topic published by `ros_python.py`.
It publishes a ROS topic named "motor" on which it publishes the value of 9 (motor velocity).
When it reads a sensor value greater than 100 on the "sensor" topic, it will publish a value of 0 on the "motor" topic.
Thus, the robot will move forward until it faces an obstacle.

The `kinetic` folder contains a minimal subset of the ROS kinetic distribution allowing to run the `rospy` package on Windows.
Similarly, the `python` folder contains a couple of Python modules needed to run `rospy`.
Both folders were copied and adapted from [here](https://github.com/Wei1234c/ROS_node_on_Windows).

Once you understood the principles behind this very simple example, you will be able to extend it to a more complex setup that will fit the requirements of your ROS project.
