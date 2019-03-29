"""universal_robot_ros controller."""

import rospy
from controller import Robot
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower

robot = Robot()
jointStatePublisher = JointStatePublisher(robot)
trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher)

timestep = int(robot.getBasicTimeStep())

rospy.init_node('ur_driver', disable_signals=True)

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
