# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Joint state publisher."""

import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher(object):
    """Publish as a ROS topic the joint state."""

    jointNames = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    def __init__(self, robot):
        """Initialize the object."""
        self.motors = []
        self.sensors = []
        self.timestep = int(robot.getBasicTimeStep())
        self.last_joint_states = None
        for name in JointStatePublisher.jointNames:
            self.motors.append(robot.getMotor(name))
            self.sensors.append(robot.getPositionSensor(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

    def publish(self):
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "From real-time state data"  #TODO
        msg.name = JointStatePublisher.jointNames
        msg.position = []
        for sensor in self.sensors:
            msg.position.append(sensor.getValue())
        # for i, q in enumerate(stateRT.q_actual):
        #     msg.position.append(q + joint_offsets.get(joint_names[i], 0.0))
        msg.velocity = [0] * 6  #TODO stateRT.qd_actual
        msg.effort = [0] * 6
        self.publisher.publish(msg)
        self.last_joint_states = msg
