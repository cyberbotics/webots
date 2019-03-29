"""universal_robot_ros controller."""

import rospy
from controller import Robot
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower

rospy.init_node('ur_driver', disable_signals=True)

robot = Robot()
jointStatePublisher = JointStatePublisher(robot)
trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher)
trajectoryFollower.start()

timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
