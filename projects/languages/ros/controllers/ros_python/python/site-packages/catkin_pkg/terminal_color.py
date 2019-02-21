# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

"""Module to enable color terminal output."""

from __future__ import print_function

import os
import string

_ansi = {}


def ansi(key):
    """Return the escape sequence for a given ansi color key."""
    global _ansi
    return _ansi[key]


def enable_ANSI_colors():
    """Populate the global module dictionary `ansi` with ANSI escape sequences."""
    global _ansi
    color_order = [
        'black', 'red', 'green', 'yellow', 'blue', 'purple', 'cyan', 'white'
    ]
    short_colors = {
        'black': 'k', 'red': 'r', 'green': 'g', 'yellow': 'y', 'blue': 'b',
        'purple': 'p', 'cyan': 'c', 'white': 'w'
    }
    _ansi = {
        'escape': '\033', 'reset': 0, '|': 0,
        'boldon': 1, '!': 1, 'italicson': 3, '/': 3, 'ulon': 4, '_': 4,
        'invon': 7, 'boldoff': 22, 'italicsoff': 23,
        'uloff': 24, 'invoff': 27
    }

    # Convert plain numbers to escapes
    for key in _ansi:
        if key != 'escape':
            _ansi[key] = '{0}[{1}m'.format(_ansi['escape'], _ansi[key])

    # Foreground
    for index, color in enumerate(color_order):
        _ansi[color] = '{0}[{1}m'.format(_ansi['escape'], 30 + index)
        _ansi[color + 'f'] = _ansi[color]
        _ansi[short_colors[color] + 'f'] = _ansi[color + 'f']

    # Background
    for index, color in enumerate(color_order):
        _ansi[color + 'b'] = '{0}[{1}m'.format(_ansi['escape'], 40 + index)
        _ansi[short_colors[color] + 'b'] = _ansi[color + 'b']

    # Fmt sanitizers
    _ansi['atexclimation'] = '@!'
    _ansi['atfwdslash'] = '@/'
    _ansi['atunderscore'] = '@_'
    _ansi['atbar'] = '@|'


def disable_ANSI_colors():
    """Set all the ANSI escape sequences to empty strings, effectively disabling console colors."""
    global _ansi
    for key in _ansi:
        _ansi[key] = ''


# Default to ansi colors on
enable_ANSI_colors()
if os.name in ['nt']:
    disable_ANSI_colors()


class ColorTemplate(string.Template):
    delimiter = '@'


def sanitize(msg):
    """Sanitize the existing msg, use before adding color annotations."""
    msg = msg.replace('@', '@@')
    msg = msg.replace('{', '{{')
    msg = msg.replace('}', '}}')
    msg = msg.replace('@@!', '@{atexclimation}')
    msg = msg.replace('@@/', '@{atfwdslash}')
    msg = msg.replace('@@_', '@{atunderscore}')
    msg = msg.replace('@@|', '@{atbar}')
    return msg


def fmt(msg):
    """Replace color annotations with ansi escape sequences."""
    global _ansi
    msg = msg.replace('@!', '@{boldon}')
    msg = msg.replace('@/', '@{italicson}')
    msg = msg.replace('@_', '@{ulon}')
    msg = msg.replace('@|', '@{reset}')
    t = ColorTemplate(msg)
    return t.substitute(_ansi) + ansi('reset')
