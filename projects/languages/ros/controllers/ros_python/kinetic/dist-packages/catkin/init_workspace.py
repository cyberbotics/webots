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
import shutil
from catkin.workspace import get_source_paths, get_workspaces

def _symlink_or_copy(src, dst):
    """
    Creates a symlink at dst to src, or if not possible, attempts to copy.
    """
    # try to symlink file
    try:
        os.symlink(src, dst)
        print('Creating symlink "%s" pointing to "%s"' % (dst, src))
    except Exception as ex_symlink:
        # try to copy file
        try:
            shutil.copyfile(src, dst)
            print('Copying file from "%s" to "%s"' % (src, dst))
        except Exception as ex_copy:
            raise RuntimeError('Could neither symlink nor copy file "%s" to "%s":\n- %s\n- %s' % (src, dst, str(ex_symlink), str(ex_copy)))


def init_workspace(workspace_dir):
    """
    Create a toplevel CMakeLists.txt in the root of a workspace.

    The toplevel.cmake file is looked up either in the catkin
    workspaces contained in the CMAKE_PREFIX_PATH or relative to this
    file.  Then it tries to create a symlink first and if that fails
    copies the file.

    It installs ``manifest.xml`` to ``share/${PROJECT_NAME}``.

    .. note:: The symlink is absolute when catkin is found outside
      the workspace_dir (since that indicates a different workspace
      and it may change relative location to the workspace referenced
      as a parameter). The symlink is relative when catkin is part of
      the to-be-initialized workspace.

    :param workspace_dir: the path to the workspace where the
      CMakeLists.txt should be created
    :type workspace_dir: string
    """
    # verify that destination file does not exist
    dst = os.path.join(workspace_dir, 'CMakeLists.txt')
    if os.path.exists(dst):
        raise RuntimeError('File "%s" already exists' % dst)
    if os.path.islink(dst):
        print('Removing symlink "%s" which points to non-existing file' % dst)
        os.unlink(dst)

    src_file_path = None
    checked = []

    # look in to-be-initialized workspace first
    src = os.path.join(workspace_dir, 'catkin', 'cmake', 'toplevel.cmake')
    if os.path.isfile(src):
        src_file_path = os.path.relpath(src, workspace_dir)
    else:
        checked.append(src)

    # search for toplevel file in all workspaces
    if src_file_path is None:
        workspaces = get_workspaces()
        for workspace in workspaces:
            source_paths = get_source_paths(workspace)
            if len(source_paths) == 0:
                # try from install space
                src = os.path.join(workspace, 'catkin', 'cmake', 'toplevel.cmake')
                if os.path.isfile(src):
                    src_file_path = src
                    break
                else:
                    checked.append(src)
            else:
                # try from all source spaces
                for source_path in source_paths:
                    src = os.path.join(source_path, 'catkin', 'cmake', 'toplevel.cmake')
                    if os.path.isfile(src):
                        src_file_path = src
                        break
                    else:
                        checked.append(src)

    # search for toplevel file in relative locations
    if src_file_path is None:
        relative_cmake_paths = []
        # when catkin is in source space
        relative_cmake_paths.append(os.path.join('..', '..', 'cmake'))
        # when catkin is installed (with Python code in lib/pythonX.Y/[dist|site]-packages)
        relative_cmake_paths.append(os.path.join('..', '..', '..', '..', 'share', 'catkin', 'cmake'))
        # when catkin is installed (with Python code in lib/site-packages)
        relative_cmake_paths.append(os.path.join('..', '..', '..', 'share', 'catkin', 'cmake'))
        for relative_cmake_path in relative_cmake_paths:
            src = os.path.abspath(os.path.join(os.path.dirname(__file__), relative_cmake_path, 'toplevel.cmake'))
            if os.path.isfile(src):
                src_file_path = src
                break
            else:
                checked.append(src)

    if src_file_path is None:
        raise RuntimeError('Could neither find file "toplevel.cmake" in any workspace nor relative, checked the following paths:\n%s' % ('\n'.join(checked)))
    _symlink_or_copy(src_file_path, dst)
