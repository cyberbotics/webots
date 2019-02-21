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

from optparse import OptionParser

import os
import sys
import traceback
import genmsg
import genmsg.command_line

from genmsg import MsgGenerationException
from . generate_initpy import write_modules

def usage(progname):
    print("%(progname)s file(s)"%vars())

def genmain(argv, progname, gen):
    parser = OptionParser("%s file"%(progname))
    parser.add_option('--initpy', dest='initpy', action='store_true',
                      default=False)
    parser.add_option('-p', dest='package')
    parser.add_option('-o', dest='outdir')
    parser.add_option('-I', dest='includepath', action='append')
    options, args = parser.parse_args(argv)
    try:
        if options.initpy:
            if options.outdir:
                retcode = write_modules(options.outdir)
            else:
                parser.error("Missing args")
        else:
            if len(args) < 2:
                parser.error("please specify args")
            if not os.path.exists(options.outdir):
                # This script can be run multiple times in parallel. We
                # don't mind if the makedirs call fails because somebody
                # else snuck in and created the directory before us.
                try:
                    os.makedirs(options.outdir)
                except OSError as e:
                    if not os.path.exists(options.outdir):
                        raise
            search_path = genmsg.command_line.includepath_to_dict(options.includepath)
            retcode = gen.generate_messages(options.package, args[1:], options.outdir, search_path)
    except genmsg.InvalidMsgSpec as e:
        print("ERROR: ", e, file=sys.stderr)
        retcode = 1
    except MsgGenerationException as e:
        print("ERROR: ", e, file=sys.stderr)
        retcode = 2
    except Exception as e:
        traceback.print_exc()
        print("ERROR: ",e)
        retcode = 3
    sys.exit(retcode or 0)
