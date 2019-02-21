#!/usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import argparse
import os
import sys
import traceback

from .common import PACKAGE_FILE
from .rospack import ManifestManager, RosPack, RosStack, ResourceNotFound


# for < fuerte, retrieve from roscore file
def get_distro_name_from_roscore():
    '''
    This function only works for ROS Electric and older.
    For any newer ROS distro the information is provided
    in the ROS_DISTRO environment variable.
    '''
    rospack = RosPack()
    # there's some chance that the location of this file changes in the future
    try:
        roslaunch_dir = rospack.get_path('roslaunch')
        roscore_file = os.path.join(roslaunch_dir, 'roscore.xml')
        if not os.path.exists(roscore_file):
            return None
    except:
        return None

    import xml.dom.minidom
    try:
        dom = xml.dom.minidom.parse(roscore_file)
        tags = dom.getElementsByTagName("param")
        for t in tags:
            if t.hasAttribute('name') and t.getAttribute('name') == 'rosdistro':
                return t.getAttribute('value')
    except:
        traceback.print_exc()


def print_without_newline(argtext):
    """Print with no new line."""
    print(argtext, end='')


def main():
    parser = argparse.ArgumentParser(
        description='rosversion -d: Output the version of the given package\n'
        'rosversion package: Output the ROS distribution name',
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-s', '--skip-newline', action='store_true',
        help='Skip trailing newline')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        'package', nargs='?',
        help="The ROS package name (e.g. 'roscpp')")
    group.add_argument(
        '-d', '--distro', action='store_true',
        help='Output the ROS distribution name')

    args = parser.parse_args()

    printer = print_without_newline if args.skip_newline else print

    if args.distro:
        if 'ROS_DISTRO' in os.environ:
            distro_name = os.environ['ROS_DISTRO']
        else:
            distro_name = get_distro_name_from_roscore()
        if not distro_name:
            distro_name = '<unknown>'
        printer(distro_name)
        sys.exit(0)

    rosstack = RosStack()
    try:
        version = rosstack.get_stack_version(args.package)
    except ResourceNotFound as e:
        try:
            # hack to make it work with wet packages
            mm = ManifestManager(PACKAGE_FILE)
            path = mm.get_path(args.package)
            package_manifest = os.path.join(path, 'package.xml')
            if os.path.exists(package_manifest):
                from xml.etree.ElementTree import ElementTree
                try:
                    root = ElementTree(None, package_manifest)
                    version = root.findtext('version')
                except Exception:
                    pass
        except ResourceNotFound as e:
            print('Cannot locate [%s]' % args.package)
            sys.exit(1)

    if version is None:
        version = '<unversioned>'
    printer(version)
