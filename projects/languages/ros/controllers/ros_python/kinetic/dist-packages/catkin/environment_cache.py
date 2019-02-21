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

import ast
import os
import platform
import subprocess
import sys


def generate_environment_script(env_script):
    """
    Generates script code to cache environment changes of a script.
    This code assumes that the script does nothing else than changing
    variables that contain colon separated lists of PATHs, by
    replacing or prepending.

    :param env_script: str The path to the script which changes the environment
    :returns: list script lines
    """
    code = []
    _append_header(code)

    _append_comment(code, 'based on a snapshot of the environment before and after calling the setup script')
    _append_comment(code, 'it emulates the modifications of the setup script without recurring computations')

    # fetch current environment
    env = os.environ

    # fetch environment after calling setup
    python_code = 'import os; print(dict(os.environ))'
    output = subprocess.check_output([env_script, sys.executable, '-c', python_code])
    env_after = ast.literal_eval(output.decode('utf8'))

    # calculate added and modified environment variables
    added = {}
    modified = {}
    for key, value in env_after.items():
        if key not in env:
            added[key] = value
        elif env[key] != value:
            modified[key] = [env[key], value]

    code.append('')
    _append_comment(code, 'new environment variables')
    for key in sorted(added.keys()):
        _set_variable(code, key, added[key])

    code.append('')
    _append_comment(code, 'modified environment variables')
    for key in sorted(modified.keys()):
        (old_value, new_value) = modified[key]
        if new_value.endswith(os.pathsep + old_value):
            variable = ('$%s' if _is_not_windows() else '%%%s%%') % key
            _set_variable(code, key, new_value[:-len(old_value)] + variable)
        else:
            _set_variable(code, key, new_value)

    return code


def _is_not_windows():
    return platform.system() != 'Windows'


def _append_header(code):
    if _is_not_windows():
        code.append('#!/usr/bin/env sh')
    else:
        code.append('@echo off')

    _append_comment(code, 'generated from catkin/python/catkin/environment_cache.py')
    code.append('')


def _append_comment(code, value):
    if _is_not_windows():
        comment_prefix = '#'
    else:
        comment_prefix = 'REM'
    code.append('%s %s' % (comment_prefix, value))


def _set_variable(code, key, value):
    if _is_not_windows():
        code.append('export %s="%s"' % (key, value))
    else:
        code.append('set %s=%s' % (key, value))
