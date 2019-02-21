# Software License Agreement (BSD License)
#
# Copyright (c) 2013-2014 Dariush Forouher
# All rights reserved.
#
# Based on code adapted from diagnostics_updater by Blaise Gassend
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from math import sqrt
import logging
import sys

from rosgraph_msgs.msg import TopicStatistics
import rospy

_logger = logging.getLogger('rospy.impl.statistics')


class SubscriberStatisticsLogger():
    """
    Class that monitors each subscriber.

    this class basically just keeps a collection of ConnectionStatisticsLogger.
    """

    @classmethod
    def is_enabled(cls):
        # disable statistics if node can't talk to parameter server
        # which is the case in unit tests
        try:
            return rospy.get_param("/enable_statistics", False)
        except Exception:
            return False

    def __init__(self, subscriber):
        self.subscriber_name = subscriber.name
        self.connections = dict()
        self.read_parameters()

    def read_parameters(self):
        """
        Fetch window parameters from parameter server
        """

        # Range of window length, in seconds
        self.min_elements = rospy.get_param("/statistics_window_min_elements", 10)
        self.max_elements = rospy.get_param("/statistics_window_max_elements", 100)

        # Range of acceptable messages in window.
        # Window size will be adjusted if number of observed is
        # outside this range.
        self.max_window = rospy.get_param("/statistics_window_max_size", 64)
        self.min_window = rospy.get_param("/statistics_window_min_size", 4)

    def callback(self, msg, publisher, stat_bytes):
        """
        This method is called for every message that has been received.

        @param msg: The message received.
        @param publisher: The name of the publisher node that sent the msg
        @param stat_bytes: A counter, how many bytes have been moved across
        this connection since it exists.

        This method just looks up the ConnectionStatisticsLogger for the specific connection
        between publisher and subscriber and delegates to statistics logging to that
        instance.
        """

        # /clock is special, as it is subscribed very early
        # also exclude /statistics to reduce noise.
        if self.subscriber_name == "/clock" or self.subscriber_name == "/statistics":
            return

        try:
            # create ConnectionStatisticsLogger for new connections
            logger = self.connections.get(publisher)
            if logger is None:
                logger = ConnectionStatisticsLogger(self.subscriber_name, rospy.get_name(), publisher)
                self.connections[publisher] = logger

            # delegate stuff to that instance
            logger.callback(self, msg, stat_bytes)
        except Exception as e:
            rospy.logerr("Unexpected error during statistics measurement: %s", str(e))

    def shutdown(self):
        for logger in self.connections.values():
            logger.shutdown()
        self.connections.clear()


class ConnectionStatisticsLogger():
    """
    Class that monitors lots of stuff for each connection.

    is created whenever a subscriber is created.
    is destroyed whenever its parent subscriber is destroyed.
    its lifecycle is therefore bound to its parent subscriber.
    """

    def __init__(self, topic, subscriber, publisher):
        """
        Constructor.

        @param topic: Name of the topic
        @param subscriber: Name of the subscriber
        @param publisher: Name of the publisher

        These three should uniquely identify the connection.
        """

        self.topic = topic
        self.subscriber = subscriber
        self.publisher = publisher

        self.pub = rospy.Publisher("/statistics", TopicStatistics, queue_size=10)

        # reset window
        self.last_pub_time = rospy.Time(0)
        self.pub_frequency = rospy.Duration(1.0)

        # timestamp age
        self.age_list_ = []

        # period calculations
        self.arrival_time_list_ = []

        self.last_seq_ = 0
        self.dropped_msgs_ = 0
        self.window_start = rospy.Time.now()

        # temporary variables
        self.stat_bytes_last_ = 0
        self.stat_bytes_window_ = 0

    def sendStatistics(self, subscriber_statistics_logger):
        """
        Send out statistics. Aggregate collected stats information.

        Currently done blocking. Might be moved to own thread later. But at the moment
        any computation done here should be rather quick.
        """
        curtime = rospy.Time.now()

        msg = TopicStatistics()
        msg.topic = self.topic
        msg.node_sub = self.subscriber
        msg.node_pub = self.publisher

        msg.window_start = self.window_start
        msg.window_stop = curtime

        # Calculate bytes since last message
        msg.traffic = self.stat_bytes_window_ - self.stat_bytes_last_

        msg.delivered_msgs = len(self.arrival_time_list_)
        msg.dropped_msgs = self.dropped_msgs_

        # we can only calculate message age if the messages did contain Header fields.
        if len(self.age_list_) > 0:
            msg.stamp_age_mean = rospy.Duration(sum(self.age_list_, rospy.Duration(0)).to_sec() / len(self.age_list_))
            variance = sum((rospy.Duration((msg.stamp_age_mean - value).to_sec() ** 2) for value in self.age_list_), rospy.Duration(0)) / len(self.age_list_)
            msg.stamp_age_stddev = rospy.Duration(sqrt(variance.to_sec()))
            msg.stamp_age_max = max(self.age_list_)
        else:
            msg.stamp_age_mean = rospy.Duration(0)
            msg.stamp_age_stddev = rospy.Duration(0)
            msg.stamp_age_max = rospy.Duration(0)

        # computer period/frequency. we need at least two messages within the window to do this.
        if len(self.arrival_time_list_) > 1:
            periods = [j - i for i, j in zip(self.arrival_time_list_[:-1], self.arrival_time_list_[1:])]
            msg.period_mean = rospy.Duration(sum(periods, rospy.Duration(0)).to_sec() / len(periods))
            variance = sum((rospy.Duration((msg.period_mean - value).to_sec() ** 2) for value in periods), rospy.Duration(0)) / len(periods)
            msg.period_stddev = rospy.Duration(sqrt(variance.to_sec()))
            msg.period_max = max(periods)
        else:
            msg.period_mean = rospy.Duration(0)
            msg.period_stddev = rospy.Duration(0)
            msg.period_max = rospy.Duration(0)

        self.pub.publish(msg)

        # adjust window, if message count is not appropriate.
        if len(self.arrival_time_list_) > subscriber_statistics_logger.max_elements and self.pub_frequency.to_sec() * 2 <= subscriber_statistics_logger.max_window:
            self.pub_frequency *= 2
        if len(self.arrival_time_list_) < subscriber_statistics_logger.min_elements and self.pub_frequency.to_sec() / 2 >= subscriber_statistics_logger.min_window:
            self.pub_frequency /= 2

        # clear collected stats, start new window.
        self.age_list_ = []
        self.arrival_time_list_ = []
        self.dropped_msgs_ = 0

        self.window_start = curtime

        self.stat_bytes_last_ = self.stat_bytes_window_

    def callback(self, subscriber_statistics_logger, msg, stat_bytes):
        """
        This method is called for every message, that is received on this
        subscriber.

        this callback will keep some statistics and publish the results
        periodically on a topic. the publishing should probably be done
        asynchronically in another thread.

        @param msg: The message, that has been received. The message has usually
        been already deserialized. However this is not always the case. (AnyMsg)
        @param stat_bytes: A counter, how many bytes have been moved across
        this connection since it exists.

        Any computing-heavy stuff should be done somewhere else, as this
        callback has to return before the message is delivered to the user.
        """

        arrival_time = rospy.Time.now()

        self.arrival_time_list_.append(arrival_time)

        self.stat_bytes_window_ = stat_bytes

        # rospy has the feature to subscribe a topic with AnyMsg which aren't deserialized.
        # Those subscribers won't have a header. But as these subscribers are rather rare
        # ("rostopic hz" is the only one I know of), I'm gonna ignore them.
        if msg._has_header:
            self.age_list_.append(arrival_time - msg.header.stamp)

            if self.last_seq_ + 1 != msg.header.seq:
                self.dropped_msgs_ = self.dropped_msgs_ + 1
            self.last_seq_ = msg.header.seq

        # send out statistics with a certain frequency
        if self.last_pub_time + self.pub_frequency < arrival_time:
            self.last_pub_time = arrival_time
            self.sendStatistics(subscriber_statistics_logger)

    def shutdown(self):
        self.pub.unregister()
