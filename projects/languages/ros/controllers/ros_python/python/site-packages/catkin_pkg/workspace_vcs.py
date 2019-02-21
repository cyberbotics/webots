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

from __future__ import print_function

import os
import subprocess


def get_repository_type(path):
    for vcs_type in ['bzr', 'git', 'hg', 'svn']:
        if os.path.isdir(os.path.join(path, '.%s' % vcs_type)):
            return vcs_type
    return None


def vcs_remotes(path, vcs_type=None):
    if vcs_type is None:
        vcs_type = get_repository_type(path)
    if vcs_type == 'git':
        output = subprocess.check_output(['git', 'remote', '-v'], cwd=path)
        return output.decode('utf-8').rstrip()
    elif vcs_type == 'hg':
        output = subprocess.check_output(['hg', 'paths'], cwd=path)
        return output.decode('utf-8').rstrip()
    elif vcs_type == 'svn':
        output = subprocess.check_output(['svn', 'info'], cwd=path)
        output = output.decode('utf-8').rstrip()
        for line in output.split(os.linesep):
            if line.startswith('URL: '):
                return line
        raise RuntimeError('Could not determine URL of svn working copy')
    else:
        raise RuntimeError('"remotes" command not supported for vcs type "%s"' % vcs_type)
