# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Chris Mansley, Open Source Robotics Foundation, Inc.
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
import socket
import sys

import rosgraph
import rospy

from .logger_level_service_caller import LoggerLevelServiceCaller
from .logger_level_service_caller import ROSConsoleException

NAME = 'rosconsole'


def error(status, msg):
    print("%s: error: %s" % (NAME, msg), file=sys.stderr)
    sys.exit(status)


def _get_cmd_list_optparse():
    from optparse import OptionParser

    usage = "usage: %prog list <node>"
    parser = OptionParser(usage=usage, prog=NAME)

    return parser


def _rosconsole_cmd_list(argv):
    args = argv[2:]
    parser = _get_cmd_list_optparse()
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error("you must specify a node to list loggers")
    elif len(args) > 1:
        parser.error("you may only specify one node to list")

    logger_level = LoggerLevelServiceCaller()

    loggers = logger_level.get_loggers(args[0])

    output = '\n'.join(loggers)
    print(output)


def _get_cmd_set_optparse():
    from optparse import OptionParser

    usage = "usage: %prog set <node> <logger> <level>"
    levels = ', '.join(LoggerLevelServiceCaller().get_levels())
    usage += "\n\n <level> must be one of [" + levels + "]"
    parser = OptionParser(usage=usage, prog=NAME)

    return parser


def _rosconsole_cmd_set(argv):
    args = argv[2:]
    parser = _get_cmd_set_optparse()
    (options, args) = parser.parse_args(args)

    if len(args) < 3:
        parser.error("you must specify a node, a logger and a level")

    logger_level = LoggerLevelServiceCaller()
    logger_level.get_loggers(args[0])

    if args[1] not in logger_level._current_levels:
        error(2, "node " + args[0] + " does not contain logger " + args[1])

    level = args[2].lower()
    if level not in logger_level.get_levels():
        parser.error("invalid level")

    logger_level.send_logger_change_message(args[0], args[1], args[2])


def _get_cmd_get_optparse():
    from optparse import OptionParser

    usage = "usage: %prog get <node> <logger>"
    parser = OptionParser(usage=usage, prog=NAME)

    return parser


def _rosconsole_cmd_get(argv):
    args = argv[2:]
    parser = _get_cmd_get_optparse()
    (options, args) = parser.parse_args(args)

    if len(args) < 2:
        parser.error("you must specify a node and a logger")

    logger_level = LoggerLevelServiceCaller()
    logger_level.get_loggers(args[0])

    if args[1] not in logger_level._current_levels:
        error(2, "node " + args[0] + " does not contain logger " + args[1])

    print(logger_level._current_levels[args[1]])


def _fullusage():
    print("""rosconsole is a command-line tool for configuring the logger level of ROS nodes.

Commands:
\trosconsole get\tdisplay level for a logger
\trosconsole list\tlist loggers for a node
\trosconsole set\tset level for a logger

Type rosconsole <command> -h for more detailed usage, e.g. 'rosconsole list -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))


def main(argv=None):
    if argv is None:
        argv = sys.argv

    # Initialize ourselves as a node, to ensure handling of namespace and
    # remapping arguments
    rospy.init_node('rosconsole', anonymous=True)
    argv = rospy.myargv(argv)

    # process argv
    if len(argv) == 1:
        _fullusage()

    try:
        command = argv[1]
        if command == 'get':
            _rosconsole_cmd_get(argv)
        elif command == 'list':
            _rosconsole_cmd_list(argv)
        elif command == 'set':
            _rosconsole_cmd_set(argv)
        else:
            _fullusage()
    except socket.error as e:
        error(1,
              "Network communication failed; most likely failed to communicate with master: %s" % e)
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        error(1, str(e))
    except ROSConsoleException as e:
        error(1, str(e))
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
