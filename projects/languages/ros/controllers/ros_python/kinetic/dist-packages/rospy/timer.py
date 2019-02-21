# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
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
#
# Revision $Id: __init__.py 12069 2010-11-09 20:31:55Z kwc $

import threading
import time

# for Time, Duration
import genpy

import rospy.core
import rospy.rostime

# author: tfield (Timers)
# author: kwc (Rate, sleep)

class Rate(object):
    """
    Convenience class for sleeping in a loop at a specified rate
    """
    
    def __init__(self, hz, reset=False):
        """
        Constructor.
        @param hz: hz rate to determine sleeping
        @type  hz: float
        @param reset: if True, timer is reset when rostime moved backward. [default: False]
        @type  reset: bool
        """
        # #1403
        self.last_time = rospy.rostime.get_rostime()
        self.sleep_dur = rospy.rostime.Duration(0, int(1e9/hz))
        self._reset = reset

    def _remaining(self, curr_time):
        """
        Calculate the time remaining for rate to sleep.
        @param curr_time: current time
        @type  curr_time: L{Time}
        @return: time remaining
        @rtype: L{Time}
        """
        # detect time jumping backwards
        if self.last_time > curr_time:
            self.last_time = curr_time

        # calculate remaining time
        elapsed = curr_time - self.last_time
        return self.sleep_dur - elapsed

    def remaining(self):
        """
        Return the time remaining for rate to sleep.
        @return: time remaining
        @rtype: L{Time}
        """
        curr_time = rospy.rostime.get_rostime()
        return self._remaining(curr_time)

    def sleep(self):
        """
        Attempt sleep at the specified rate. sleep() takes into
        account the time elapsed since the last successful
        sleep().
        
        @raise ROSInterruptException: if ROS shutdown occurs before
        sleep completes
        @raise ROSTimeMovedBackwardsException: if ROS time is set
        backwards
        """
        curr_time = rospy.rostime.get_rostime()
        try:
            sleep(self._remaining(curr_time))
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            if not self._reset:
                raise
            self.last_time = rospy.rostime.get_rostime()
            return
        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time

def sleep(duration):
    """
    sleep for the specified duration in ROS time. If duration
    is negative, sleep immediately returns.
    
    @param duration: seconds (or rospy.Duration) to sleep
    @type  duration: float or Duration
    @raise ROSInterruptException: if ROS shutdown occurs before sleep
    completes
    @raise ROSTimeMovedBackwardsException: if ROS time is set
    backwards
    """
    if rospy.rostime.is_wallclock():
        if isinstance(duration, genpy.Duration):
            duration = duration.to_sec()
        if duration < 0:
            return
        else:
            rospy.rostime.wallsleep(duration)
    else:
        initial_rostime = rospy.rostime.get_rostime()
        if not isinstance(duration, genpy.Duration):
            duration = genpy.Duration.from_sec(duration)

        rostime_cond = rospy.rostime.get_rostime_cond()

        # #3123
        if initial_rostime == genpy.Time(0):
            # break loop if time is initialized or node is shutdown
            while initial_rostime == genpy.Time(0) and \
                      not rospy.core.is_shutdown():
                with rostime_cond:
                    rostime_cond.wait(0.3)
                initial_rostime = rospy.rostime.get_rostime()

        sleep_t = initial_rostime + duration

        # break loop if sleep_t is reached, time moves backwards, or
        # node is shutdown
        while rospy.rostime.get_rostime() < sleep_t and \
              rospy.rostime.get_rostime() >= initial_rostime and \
                  not rospy.core.is_shutdown():
            with rostime_cond:
                rostime_cond.wait(0.5)

        if rospy.rostime.get_rostime() < initial_rostime:
            time_jump = (initial_rostime - rospy.rostime.get_rostime()).to_sec()
            rospy.core.logerr("ROS time moved backwards: %ss", time_jump)
            raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("ROS shutdown request")

class TimerEvent(object):
    """
    Constructor.
    @param last_expected: in a perfect world, this is when the previous callback should have happened
    @type  last_expected: rospy.Time
    @param last_real: when the callback actually happened
    @type  last_real: rospy.Time
    @param current_expected: in a perfect world, this is when the current callback should have been called
    @type  current_expected: rospy.Time
    @param last_duration: contains the duration of the last callback (end time minus start time) in seconds.
                          Note that this is always in wall-clock time.
    @type  last_duration: float
    """
    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected    = last_expected
        self.last_real        = last_real
        self.current_expected = current_expected
        self.current_real     = current_real
        self.last_duration    = last_duration

class Timer(threading.Thread):
    """
    Convenience class for calling a callback at a specified rate
    """

    def __init__(self, period, callback, oneshot=False, reset=False):
        """
        Constructor.
        @param period: desired period between callbacks
        @type  period: rospy.Duration
        @param callback: callback to be called
        @type  callback: function taking rospy.TimerEvent
        @param oneshot: if True, fire only once, otherwise fire continuously until shutdown is called [default: False]
        @type  oneshot: bool
        @param reset: if True, timer is reset when rostime moved backward. [default: False]
        @type  reset: bool
        """
        super(Timer, self).__init__()
        self._period   = period
        self._callback = callback
        self._oneshot  = oneshot
        self._reset = reset
        self._shutdown = False
        self.setDaemon(True)
        self.start()

    def shutdown(self):
        """
        Stop firing callbacks.
        """
        self._shutdown = True
        
    def run(self):
        r = Rate(1.0 / self._period.to_sec(), reset=self._reset)
        current_expected = rospy.rostime.get_rostime() + self._period
        last_expected, last_real, last_duration = None, None, None
        while not rospy.core.is_shutdown() and not self._shutdown:
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.core.is_shutdown():
                    break
                raise
            if self._shutdown:
                break
            current_real = rospy.rostime.get_rostime()
            start = time.time()
            self._callback(TimerEvent(last_expected, last_real, current_expected, current_real, last_duration))
            if self._oneshot:
                break
            last_duration = time.time() - start
            last_expected, last_real = current_expected, current_real
            current_expected += self._period
