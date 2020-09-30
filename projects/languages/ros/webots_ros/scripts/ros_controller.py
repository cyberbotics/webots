#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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

"""
This is a simple example of a Python ROS node receiving sensor values and publishing motor commands (velocity)
to drive a robot and stop it before colliding with an obstacle.
"""

import rospy
from std_msgs.msg import Float64


def callback(data):
    global pub
    rospy.loginfo(rospy.get_caller_id() + 'Received sensor value: %s', data.data)
    if data.data > 100:
        pub.publish(0)
    else:
        pub.publish(9)


rospy.init_node('controller', anonymous=True)
pub = rospy.Publisher('motor', Float64, queue_size=10)
rospy.Subscriber("sensor", Float64, callback)
rospy.spin()
