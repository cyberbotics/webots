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

from __future__ import print_function

import os

from genmsg import MsgGenerationException

## :param type_name str: Name of message type sans package,
## e.g. 'String'
## :returns str: name of python module for auto-generated code
def _module_name(type_name):
    return "_"+type_name
    
def write_modules(outdir):
    if not os.path.isdir(outdir):
        #TODO: warn?
        return 0
    types_in_dir = set([f[1:-3] for f in os.listdir(outdir)
                     if f.endswith('.py') and f != '__init__.py'])
    generated_modules = [_module_name(f) for f in sorted(types_in_dir)]
    write_module(outdir, generated_modules)
    return 0

def write_module(basedir, generated_modules):
    """
    Create a module file to mark directory for python

    :param base_dir: path to package, ``str``
    :param package: name of package to write module for, ``str``
    :param generated_modules: list of generated message modules,
      i.e. the names of the .py files that were generated for each
      .msg file. ``[str]``
    """
    if not os.path.exists(basedir):
        os.makedirs(basedir)
    elif not os.path.isdir(basedir):
        raise MsgGenerationException("file preventing the creating of module directory: %s"%dir)
    p = os.path.join(basedir, '__init__.py')
    with open(p, 'w') as f:
        for mod in generated_modules:
            f.write('from .%s import *\n'%mod)

    parent_init = os.path.dirname(basedir)
#    p = os.path.join(parent_init, '__init__.py')
#    if not os.path.exists(p):
#        #touch __init__.py in the parent package
#        with open(p, 'w') as f:
#            print("import pkgutil, os.path", file=f)
#            print("__path__ = pkgutil.extend_path(__path__, __name__)", file=f)
#            if srcdir is not None:
#                staticinit = '%s/%s/__init__.py' % (srcdir, package)
#                print("if os.path.isfile('%s'): execfile('%s')" % (staticinit, staticinit), file=f)
