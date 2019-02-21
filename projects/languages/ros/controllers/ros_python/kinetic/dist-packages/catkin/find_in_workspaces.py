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
from catkin.workspace import get_source_paths, get_workspaces
from catkin_pkg.packages import find_packages


def _get_valid_search_dirs(search_dirs, project):
    """
    compares param collection of search dirs with valid names, raises ValueError if invalid.
    maintains the order of param if any. If project is given other names are allowed than without.

    :param search_dirs: collection of foldernames (basename) to search for
    :param project: the project to search in or None
    :raises: ValueError
    """
    # define valid search folders
    valid_global_search_dirs = ['bin', 'etc', 'include', 'lib', 'share']
    valid_project_search_dirs = ['etc', 'include', 'libexec', 'share']

    valid_search_dirs = (valid_global_search_dirs
                         if project is None
                         else valid_project_search_dirs)
    if not search_dirs:
        search_dirs = valid_search_dirs
    else:
        # make search folders a list
        search_dirs = list(search_dirs)

        # determine valid search folders
        all_valid_search_dirs = set(valid_global_search_dirs).union(
            set(valid_project_search_dirs))

        # check folder name is known at all
        diff_dirs = set(search_dirs).difference(all_valid_search_dirs)
        if len(diff_dirs) > 0:
            raise ValueError('Unsupported search folders: ' +
                             ', '.join(['"%s"' % i for i in diff_dirs]))
        # check foldername works with project arg
        diff_dirs = set(search_dirs).difference(valid_search_dirs)
        if len(diff_dirs) > 0:
            msg = 'Searching %s a project can not be combined with the search folders:' % ('without' if project is None else 'for')
            raise ValueError(msg + ', '.join(['"%s"' % i for i in diff_dirs]))
    return search_dirs


# OUT is always a list of folders
#
# IN: project=None
# OUT: foreach ws in workspaces: foreach s in search_in: cand = ws[0] + s (+ path)
#      add cand to result list if it exists
#      is not defined for s == 'libexec', bailing out
#
# IN: project=not None
# OUT: foreach ws in workspaces: foreach s in search_in: cand = ws[0] + s + project (+ path)
#      except for s == 'share', cand is a list of two paths: ws[0] + s + project (+ path) and ws[1] + project (+ path)
#      add cand to result list if it exists
#      is not defined for s in ['bin', 'lib'], bailing out
def find_in_workspaces(search_dirs=None, project=None, path=None, _workspaces=None, considered_paths=None, first_matching_workspace_only=False, first_match_only=False, workspace_to_source_spaces=None, source_path_to_packages=None):
    '''
    Find all paths which match the search criteria.
    All workspaces are searched in order.
    Each workspace, each search_in subfolder, the project name and the path are concatenated to define a candidate path.
    If the candidate path exists it is appended to the result list.
    Note: the search might return multiple paths for 'share' from devel- and source-space.

    :param search_dir: The list of subfolders to search in (default contains all valid values: 'bin', 'etc', 'lib', 'libexec', 'share'), ``list``
    :param project: The project name to search for (optional, not possible with the global search_in folders 'bin' and 'lib'), ``str``
    :param path: The path, ``str``
    :param _workspaces: (optional, used for unit tests), the list of workspaces to use.
    :param considered_paths: If not None, function will append all path that were searched
    :param first_matching_workspace_only: if True returns all results found for first workspace with results
    :param first_match_only: if True returns first path found (supercedes first_matching_workspace_only)
    :param workspace_to_source_spaces: the dictionary is populated with mappings from workspaces to source paths, pass in the same dictionary to avoid repeated reading of the catkin marker file
    :param source_path_to_packages: the dictionary is populated with mappings from source paths to packages, pass in the same dictionary to avoid repeated crawling
    :raises ValueError: if search_dirs contains an invalid folder name
    :returns: List of paths
    '''
    search_dirs = _get_valid_search_dirs(search_dirs, project)
    if 'libexec' in search_dirs:
        search_dirs.insert(search_dirs.index('libexec'), 'lib')
    if _workspaces is None:
        _workspaces = get_workspaces()
    if workspace_to_source_spaces is None:
        workspace_to_source_spaces = {}
    if source_path_to_packages is None:
        source_path_to_packages = {}

    paths = []
    existing_paths = []
    try:
        for workspace in (_workspaces or []):
            for sub in search_dirs:
                # search in workspace
                p = os.path.join(workspace, sub)
                if project:
                    p = os.path.join(p, project)
                if path:
                    p = os.path.join(p, path)
                paths.append(p)
                if os.path.exists(p):
                    existing_paths.append(p)
                    if first_match_only:
                        raise StopIteration

                # for search in share also consider source spaces
                if project is not None and sub == 'share':
                    if workspace not in workspace_to_source_spaces:
                        workspace_to_source_spaces[workspace] = get_source_paths(workspace)
                    for source_path in workspace_to_source_spaces[workspace]:
                        if source_path not in source_path_to_packages:
                            source_path_to_packages[source_path] = find_packages(source_path)
                        matching_packages = [p for p, pkg in source_path_to_packages[source_path].items() if pkg.name == project]
                        if matching_packages:
                            p = source_path
                            if matching_packages[0] != os.curdir:
                                p = os.path.join(p, matching_packages[0])
                            if path is not None:
                                p = os.path.join(p, path)
                            paths.append(p)
                            if os.path.exists(p):
                                existing_paths.append(p)
                                if first_match_only:
                                    raise StopIteration

            if first_matching_workspace_only and existing_paths:
                break

    except StopIteration:
        pass

    if considered_paths is not None:
        considered_paths.extend(paths)

    return existing_paths
