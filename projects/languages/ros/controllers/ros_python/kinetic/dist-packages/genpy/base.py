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

################################################################################
# Primitive type handling for ROS builtin types

SIMPLE_TYPES_DICT = { #see python module struct
    'int8': 'b', 
    'uint8': 'B',
    # Python 2.6 adds in '?' for C99 _Bool, which appears equivalent to an uint8,
    # thus, we use uint8
    'bool': 'B',    
    'int16' : 'h',
    'uint16' : 'H',
    'int32' : 'i',
    'uint32' : 'I',
    'int64' : 'q',
    'uint64' : 'Q',
    'float32': 'f',
    'float64': 'd',
    # deprecated
    'char' : 'B', #unsigned
    'byte' : 'b', #signed
    }

## Simple types are primitives with fixed-serialization length
SIMPLE_TYPES = list(SIMPLE_TYPES_DICT.keys()) #py3k

def is_simple(type_):
    """
    :returns: ``True`` if type is a 'simple' type, i.e. is of
      fixed/known serialization length. This is effectively all primitive
      types except for string, ``bool``
    """
    return type_ in SIMPLE_TYPES

