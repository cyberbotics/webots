# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

"""
ROS Time representation, including Duration
"""
from __future__ import division
import numbers
import sys
import warnings


def _canon(secs, nsecs):
    # canonical form: nsecs is always positive, nsecs < 1 second
    secs_over = nsecs // 1000000000
    secs += secs_over
    nsecs -= secs_over * 1000000000
    return secs, nsecs


class TVal(object):
    """
    Base class of :class:`Time` and :class:`Duration` representations. Representation
    is secs+nanoseconds since epoch.
    """

    __slots__ = ['secs', 'nsecs']

    # mimic same API as messages when being introspected
    _slot_types = ['int32', 'int32']

    def __init__(self, secs=0, nsecs=0):
        """
        :param secs: seconds. If secs is a float, then nsecs must not be set or 0,
          larger seconds will be of type long on 32-bit systems, ``int/long/float``
        :param nsecs: nanoseconds, ``int``
        """
        if not isinstance(secs, numbers.Integral):
            # float secs constructor
            if nsecs != 0:
                raise ValueError("if secs is a float, nsecs cannot be set")
            float_secs = secs
            secs = int(float_secs)
            nsecs = int((float_secs - secs) * 1000000000)

        self.secs, self.nsecs = _canon(secs, nsecs)

    @classmethod
    def from_sec(cls, float_secs):
        """
        Create new TVal instance using time.time() value (float
        seconds)

        :param float_secs: time value in time.time() format, ``float``
        :returns: :class:`TVal` instance for specified time
        """
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return cls(secs, nsecs)

    def is_zero(self):
        """
        :returns: ``True`` if time is zero (secs and nsecs are zero), ``bool``
        """
        return self.secs == 0 and self.nsecs == 0
    
    def set(self, secs, nsecs):
        """
        Set time using separate secs and nsecs values
        
        :param secs: seconds since epoch, ``int``
        :param nsecs: nanoseconds since seconds, ``int``
        """
        self.secs = secs
        self.nsecs = nsecs

    def canon(self):
        """
        Canonicalize the field representation in this instance.  should
        only be used when manually setting secs/nsecs slot values, as
        in deserialization.
        """
        self.secs, self.nsecs = _canon(self.secs, self.nsecs)
        
    def to_sec(self):
        """
        :returns: time as float seconds (same as time.time() representation), ``float``
        """
        return float(self.secs) + float(self.nsecs) / 1e9

    def to_nsec(self):
        """
        :returns: time as nanoseconds, ``long``
        """
        return self.secs * int(1e9) + self.nsecs
        
    def __hash__(self):
        """
        Time values are hashable. Time values with identical fields have the same hash.
        """
        return hash((self.secs, self.nsecs))

    def __str__(self):
        return str(self.to_nsec())

    def __repr__(self):
        return "genpy.TVal[%d]"%self.to_nsec()

    def __nonzero__(self):
        """
        Return if time value is not zero
        """
        return self.secs != 0 or self.nsecs != 0
    __bool__ = __nonzero__

    def __lt__(self, other):
        """
        < test for time values
        """
        try:
            return self.__cmp__(other) < 0
        except TypeError:
            return NotImplemented
    def __le__(self, other):
        """
        <= test for time values
        """
        try:
            return self.__cmp__(other) <= 0
        except TypeError:
            return NotImplemented
    def __gt__(self, other):
        """
        > test for time values
        """
        try:
            return self.__cmp__(other) > 0
        except TypeError:
            return NotImplemented
    def __ge__(self, other):
        """
        >= test for time values
        """
        try:
            return self.__cmp__(other) >= 0
        except TypeError:
            return NotImplemented
    def __ne__(self, other):
        return not self.__eq__(other)
    def __cmp__(self, other):
        if not isinstance(other, TVal):
            raise TypeError("Cannot compare to non-TVal")
        a = self.to_nsec()
        b = other.to_nsec()
        return (a > b) - (a < b)

    def __eq__(self, other):
        if not isinstance(other, TVal):
            return False
        return self.to_nsec() == other.to_nsec()

class Time(TVal):
    """
    Time contains the ROS-wide 'time' primitive representation, which
    consists of two integers: seconds since epoch and nanoseconds since
    seconds. Time instances are mutable.
    """
    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        """
        Constructor: secs and nsecs are integers. You may prefer to use the static L{from_sec()} factory
        method instead.
        
        :param secs: seconds since epoch, ``int``
        :param nsecs: nanoseconds since seconds (since epoch), ``int``
        """
        super(Time, self).__init__(secs, nsecs)
        if self.secs < 0:
            raise TypeError("time values must be positive")

    def __getstate__(self):
        """
        support for Python pickling
        """
        return [self.secs, self.nsecs]

    def __setstate__(self, state):
        """
        support for Python pickling
        """
        self.secs, self.nsecs = state

    def to_time(self):
        """
        Get Time in time.time() format. alias of L{to_sec()}
        
        :returns: time in floating point secs (time.time() format), ``float``
        """
        return self.to_sec()

    def __hash__(self):
        return super(Time, self).__hash__()

    def __repr__(self):
        return "genpy.Time[%d]"%self.to_nsec()

    def __add__(self, other):
        """
        Add duration to this time
        
        :param other: :class:`Duration`
        """
        if not isinstance(other, Duration):
            return NotImplemented
        return Time(self.secs + other.secs, self.nsecs + other.nsecs)

    __radd__ = __add__

    def __sub__(self, other):
        """
        Subtract time or duration from this time
        :param other: :class:`Duration`/:class:`Time`
        :returns: :class:`Duration` if other is a :class:`Time`, :class:`Time` if other is a :class:`Duration`
        """
        if isinstance(other, Time):
            return Duration(self.secs - other.secs, self.nsecs - other.nsecs)
        elif isinstance(other, Duration):
            return Time(self.secs - other.secs, self.nsecs - other.nsecs)
        else:
            return NotImplemented

    def __cmp__(self, other):
        """
        Compare to another time
        :param other: :class:`Time`
        """
        if not isinstance(other, Time):
            raise TypeError("cannot compare to non-Time")
        a = self.to_nsec()
        b = other.to_nsec()
        return (a > b) - (a < b)

    def __eq__(self, other):
        """
        Equals test for Time. Comparison assumes that both time
        instances are in canonical representation; only compares fields.
        
        :param other: :class:`Time`
        """
        if not isinstance(other, Time):
            return False
        return self.secs == other.secs and self.nsecs == other.nsecs

class Duration(TVal):
    """
    Duration represents the ROS 'duration' primitive, which consists
    of two integers: seconds and nanoseconds. The Duration class
    allows you to add and subtract Duration instances, including
    adding and subtracting from :class:`Time` instances.
    """
    __slots__ = ['secs', 'nsecs']
    def __init__(self, secs=0, nsecs=0):
        """
        Create new Duration instance. secs and nsecs are integers and correspond to the ROS 'duration' primitive.

        :param secs: seconds, ``int``
        :param nsecs: nanoseconds, ``int``
        """
        super(Duration, self).__init__(secs, nsecs)

    def __getstate__(self):
        """
        support for Python pickling
        """
        return [self.secs, self.nsecs]

    def __setstate__(self, state):
        """
        support for Python pickling
        """
        self.secs, self.nsecs = state

    def __hash__(self):
        return super(Duration, self).__hash__()

    def __repr__(self):
        return "genpy.Duration[%d]"%self.to_nsec()

    def __neg__(self):
        """
        :returns: Negative value of this :class:`Duration`
        """
        return Duration(-self.secs, -self.nsecs)
    def __abs__(self):
        """
        Absolute value of this duration
        :returns: positive :class:`Duration`
        """
        if self.secs >= 0:
            return self
        return self.__neg__()

    def __add__(self, other):
        """
        Add duration to this duration, or this duration to a time, creating a new time value as a result.
        :param other: duration or time, ``Duration``/``Time``
        :returns: :class:`Duration` if other is a :class:`Duration`
          instance, :class:`Time` if other is a :class:`Time`
        """
        if isinstance(other, Duration):
            return Duration(self.secs+other.secs, self.nsecs+other.nsecs)
        else:
            return NotImplemented

    __radd__ = __add__

    def __sub__(self, other):
        """
        Subtract duration from this duration, returning a new duration
        :param other: duration
        :returns: :class:`Duration`
        """
        if not isinstance(other, Duration):
            return NotImplemented
        return Duration(self.secs-other.secs, self.nsecs-other.nsecs)        

    def __mul__(self, val):
        """
        Multiply this duration by an integer or float
        :param val: multiplication factor, ``int/float``
        :returns: :class:`Duration` multiplied by val
        """
        if isinstance(val, numbers.Integral):
            return Duration(self.secs * val, self.nsecs * val)
        elif isinstance(val, numbers.Real):
            return Duration.from_sec(self.to_sec() * val)
        else:
            return NotImplemented

    __rmul__ = __mul__

    def __floordiv__(self, val):
        """
        Floor divide this duration by an integer or float
        :param val: division factor ``int/float``, or :class:`Duration` to divide by
        :returns: :class:`Duration` divided by val - a :class:`Duration` if divided by a number, or a number if divided by a duration
        """
        if isinstance(val, numbers.Integral) or isinstance(val, numbers.Real):
            return Duration.from_sec(self.to_sec() // val)
        elif isinstance(val, Duration):
            return int(self.to_sec() // val.to_sec())
        else:
            return NotImplemented

    def __div__(self, val):
        """
        Divide this duration by an integer or float
        :param val: division factor ``int/float``, or :class:`Duration` to divide by
        :returns: :class:`Duration` divided by val - a :class:`Duration` if divided by a number, or a number if divided by a duration
        """
        if isinstance(val, numbers.Integral) or isinstance(val, numbers.Real):
            return Duration.from_sec(self.to_sec() / val)
        elif isinstance(val, Duration):
            return self.to_sec() / val.to_sec()
        else:
            return NotImplemented

    def __truediv__(self, val):
        """
        Divide this duration by an integer or float
        :param val: division factor ``int/float``, or :class:`Duration` to divide by
        :returns: :class:`Duration` divided by val - a :class:`Duration` if divided by a number, or a number if divided by a duration
        """
        if isinstance(val, numbers.Real):
            return Duration.from_sec(self.to_sec() / val)
        elif isinstance(val, Duration):
            return self.to_sec() / val.to_sec()
        else:
            return NotImplemented

    def __mod__(self, val):
        """
        Find the remainder when dividing this Duration by another Duration
        :returns: :class:`Duration` The remaining time after the division
        """
        if isinstance(val, Duration):
            return Duration.from_sec(self.to_sec() % val.to_sec())
        else:
            return NotImplemented

    def __divmod__(self, val):
        """
        Implements the builtin divmod for a pair of Durations
        :returns: ``int`` The floored result of the division
        :returns: :class:`Duration` The remaining time after the division
        """
        if isinstance(val, Duration):
            quotient, remainder = divmod(self.to_sec(), val.to_sec())
            return int(quotient), Duration.from_sec(remainder)
        else:
            return NotImplemented

    def __cmp__(self, other):
        if not isinstance(other, Duration):
            raise TypeError("Cannot compare to non-Duration")
        a = self.to_nsec()
        b = other.to_nsec()
        return (a > b) - (a < b)

    def __eq__(self, other):
        if not isinstance(other, Duration):
            return False
        return self.secs == other.secs and self.nsecs == other.nsecs
