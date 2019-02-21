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
#
# Revision $Id$
# $Author$

"""
Warning: do not use this library.  It is unstable and most of the routines
here have been superceded by other libraries (e.g. rospkg).  These
routines will likely be *deleted* in future releases.
"""

import os
import sys
import stat
import string

from subprocess import Popen, PIPE

from catkin.find_in_workspaces import find_in_workspaces as catkin_find
import rospkg

import roslib.manifest

SRC_DIR = 'src'

# aliases
ROS_PACKAGE_PATH = rospkg.environment.ROS_PACKAGE_PATH
ROS_ROOT = rospkg.environment.ROS_ROOT

class ROSPkgException(Exception):
    """
    Base class of package-related errors.
    """
    pass
class InvalidROSPkgException(ROSPkgException):
    """
    Exception that indicates that a ROS package does not exist
    """
    pass
class MultipleNodesException(ROSPkgException):
    """
    Exception that indicates that multiple ROS nodes by the same name are in the same package.
    """
    pass

# TODO: go through the code and eliminate unused methods -- there's far too many combos here

MANIFEST_FILE = 'manifest.xml'
PACKAGE_FILE = 'package.xml'

#
# Map package/directory structure
#

def get_dir_pkg(d):
    """
    Get the package that the directory is contained within. This is
    determined by finding the nearest parent manifest.xml file. This
    isn't 100% reliable, but symlinks can fool any heuristic that
    relies on ROS_ROOT.
    @param d: directory path
    @type  d: str
    @return: (package_directory, package) of the specified directory, or None,None if not in a package
    @rtype: (str, str)
    """
    #TODO: the realpath is going to create issues with symlinks, most likely

    parent = os.path.dirname(os.path.realpath(d))
    #walk up until we hit ros root or ros/pkg
    while not os.path.exists(os.path.join(d, MANIFEST_FILE)) and not os.path.exists(os.path.join(d, PACKAGE_FILE)) and parent != d:
        d = parent
        parent = os.path.dirname(d)
    if os.path.exists(os.path.join(d, MANIFEST_FILE)) or os.path.exists(os.path.join(d, PACKAGE_FILE)):
        pkg = os.path.basename(os.path.abspath(d))
        return d, pkg
    return None, None

_pkg_dir_cache = {}

def get_pkg_dir(package, required=True, ros_root=None, ros_package_path=None):
    """
    Locate directory package is stored in. This routine uses an
    internal cache.

    NOTE: cache does *not* rebuild if packages are relocated after
    this process is initiated.
    
    @param package: package name
    @type  package: str
    @param required: if True, an exception will be raised if the
    package directory cannot be located.
    @type  required: bool
    @param ros_root: if specified, override ROS_ROOT
    @type  ros_root: str
    @param ros_package_path: if specified, override ROS_PACKAGE_PATH
    @type  ros_package_path: str
    @return: directory containing package or None if package cannot be found and required is False.
    @rtype: str
    @raise InvalidROSPkgException: if required is True and package cannot be located
    """    

    #UNIXONLY
    #TODO: replace with non-rospack-based solution (e.g. os.walk())
    try:
        penv = os.environ.copy()
        if ros_root:
            ros_root = rospkg.environment._resolve_path(ros_root)
            penv[ROS_ROOT] = ros_root
        elif ROS_ROOT in os.environ:
            # record setting for _pkg_dir_cache
            ros_root = os.environ[ROS_ROOT]

        # determine rospack exe name
        rospack = 'rospack'

        if ros_package_path is not None:
            ros_package_path = rospkg.environment._resolve_paths(ros_package_path)
            penv[ROS_PACKAGE_PATH] = ros_package_path
        elif ROS_PACKAGE_PATH in os.environ:
            # record setting for _pkg_dir_cache
            ros_package_path = os.environ[ROS_PACKAGE_PATH]

        # update cache if we haven't. NOTE: we only get one cache
        if not _pkg_dir_cache:
            _read_rospack_cache(_pkg_dir_cache, ros_root, ros_package_path)
            
        # now that we've resolved the args, check the cache
        if package in _pkg_dir_cache:
            dir_, rr, rpp = _pkg_dir_cache[package]
            if rr == ros_root and rpp == ros_package_path:
                if os.path.isfile(os.path.join(dir_, MANIFEST_FILE)):
                    return dir_
                else:
                    # invalidate cache
                    _invalidate_cache(_pkg_dir_cache)
            
        rpout, rperr = Popen([rospack, 'find', package], \
                                 stdout=PIPE, stderr=PIPE, env=penv).communicate()

        pkg_dir = (rpout or '').strip()
        #python3.1 popen returns as bytes
        if (isinstance(pkg_dir, bytes)):
            pkg_dir = pkg_dir.decode()
        if not pkg_dir:
            raise InvalidROSPkgException("Cannot locate installation of package %s: %s. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(package, rperr.strip(), ros_root, ros_package_path))

        pkg_dir = os.path.normpath(pkg_dir)
        if not os.path.exists(pkg_dir):
            raise InvalidROSPkgException("Cannot locate installation of package %s: [%s] is not a valid path. ROS_ROOT[%s] ROS_PACKAGE_PATH[%s]"%(package, pkg_dir, ros_root, ros_package_path))
        elif not os.path.isdir(pkg_dir):
            raise InvalidROSPkgException("Package %s is invalid: file [%s] is in the way"%(package, pkg_dir))
        # don't update cache: this should only be updated from
        # rospack_cache as it will corrupt package list otherwise.
        #_pkg_dir_cache[package] = (pkg_dir, ros_root, ros_package_path)
        return pkg_dir
    except OSError as e:
        if required:
            raise InvalidROSPkgException("Environment configuration is invalid: cannot locate rospack (%s)"%e)
        return None
    except Exception as e:
        if required:
            raise
        return None

def _get_pkg_subdir_by_dir(package_dir, subdir, required=True, env=None):
    """
    @param required: if True, will attempt to  create the subdirectory
        if it does not exist. An exception will be raised  if this fails.
    @type  required: bool
    @param package_dir: directory of package
    @type  package_dir: str
    @param subdir: name of subdirectory to locate
    @type  subdir: str
    @param env: override os.environ dictionary    
    @type  env: dict
    @param required: if True, directory must exist    
    @type  required: bool
    @return: Package subdirectory if package exist, otherwise None.
    @rtype: str
    @raise InvalidROSPkgException: if required is True and directory does not exist
    """
    if env is None:
        env = os.environ
    try:
        if not package_dir:
            raise Exception("Cannot create a '%(subdir)s' directory in %(package_dir)s: package %(package) cannot be located"%locals())
        d = os.path.join(package_dir, subdir)
        if required and os.path.isfile(d):
            raise Exception("""Package '%(package)s' is improperly configured: 
file %(d)s is preventing the creation of a directory"""%locals())
        elif required and not os.path.isdir(d):
            try:
                os.makedirs(d) #lazy create
            except error:
                raise Exception("""Package '%(package)s' is improperly configured: 
Cannot create a '%(subdir)s' directory in %(package_dir)s.
Please check permissions and try again.
"""%locals())
        return d
    except Exception as e:
        if required:
            raise
        return None
    
def get_pkg_subdir(package, subdir, required=True, env=None):
    """
    @param required: if True, will attempt to create the subdirectory
        if it does not exist. An exception will be raised  if this fails.
    @type  required: bool
    @param package: name of package
    @type  package: str
    @param env: override os.environ dictionary
    @type  env: dict
    @param required: if True, directory must exist    
    @type  required: bool
    @return: Package subdirectory if package exist, otherwise None.
    @rtype: str
    @raise InvalidROSPkgException: if required is True and directory does not exist
    """
    if env is None:
        env = os.environ
    pkg_dir = get_pkg_dir(package, required, ros_root=env[ROS_ROOT]) 
    return _get_pkg_subdir_by_dir(pkg_dir, subdir, required, env)

#
# Map ROS resources to files
#

def resource_file(package, subdir, resource_name):
    """
    @param subdir: name of subdir -- these should be one of the
        string constants, e.g. MSG_DIR
    @type  subdir: str
    @return: path to resource in the specified subdirectory of the
        package, or None if the package does not exists
    @rtype: str
    @raise roslib.packages.InvalidROSPkgException: If package does not exist 
    """
    d = get_pkg_subdir(package, subdir, False)
    if d is None:
        raise InvalidROSPkgException(package)
    return os.path.join(d, resource_name)

def _update_rospack_cache(env=None):
    """
    Internal routine to update global package directory cache
    
    @return: True if cache is valid
    @rtype: bool
    """
    if env is None:
        env = os.environ
    cache = _pkg_dir_cache
    if cache:
        return True
    ros_root = env[ROS_ROOT]
    ros_package_path = env.get(ROS_PACKAGE_PATH, '')
    return _read_rospack_cache(cache, ros_root, ros_package_path)

def _invalidate_cache(cache):
    # I've only made this a separate routine because roslib.packages should really be using
    # the roslib.stacks cache implementation instead with the separate cache marker
    cache.clear()

def _read_rospack_cache(cache, ros_root, ros_package_path):
    """
    Read in rospack_cache data into cache. On-disk cache specifies a
    ROS_ROOT and ROS_PACKAGE_PATH, which must match the requested
    environment.
    
    @param cache: empty dictionary to store package list in. 
        If no cache argument provided, will use internal _pkg_dir_cache
        and will return cached answers if available.
        The format of the cache is {package_name: dir_path, ros_root, ros_package_path}.
    @type  cache: {str: str, str, str}
    @param ros_package_path: ROS_ROOT value
    @type  ros_root: str
    @param ros_package_path: ROS_PACKAGE_PATH value or '' if not specified
    @type  ros_package_path: str
    @return: True if on-disk cache matches and was loaded, false otherwise
    @rtype: bool
    """
    try:
        with open(os.path.join(rospkg.get_ros_home(), 'rospack_cache')) as f:
            for l in f.readlines():
                l = l[:-1]
                if not len(l):
                    continue
                if l[0] == '#':
                    # check that the cache matches our env
                    if l.startswith('#ROS_ROOT='):
                        if not l[len('#ROS_ROOT='):] == ros_root:
                            return False
                    elif l.startswith('#ROS_PACKAGE_PATH='):
                        if not l[len('#ROS_PACKAGE_PATH='):] == ros_package_path:
                            return False
                else:
                    cache[os.path.basename(l)] = l, ros_root, ros_package_path
        return True
    except:
        pass
    
def list_pkgs_by_path(path, packages=None, cache=None, env=None):
    """
    List ROS packages within the specified path.

    Optionally, a cache dictionary can be provided, which will be
    updated with the package->path mappings. list_pkgs_by_path() does
    NOT returned cached results -- it only updates the cache.
    
    @param path: path to list packages in
    @type  path: str
    @param packages: list of packages to append to. If package is
      already present in packages, it will be ignored.
    @type  packages: [str]
    @param cache: (optional) package path cache to update. Maps package name to directory path.
    @type  cache: {str: str}
    @return: complete list of package names in ROS environment. Same as packages parameter.
    @rtype: [str]
    """
    if packages is None:
        packages = []
    if env is None:
        env = os.environ
    # record settings for cache
    ros_root = env[ROS_ROOT]
    ros_package_path = env.get(ROS_PACKAGE_PATH, '')

    path = os.path.abspath(path)
    for d, dirs, files in os.walk(path, topdown=True):
        if MANIFEST_FILE in files:
            package = os.path.basename(d)
            if package not in packages:
                packages.append(package)
                if cache is not None:
                    cache[package] = d, ros_root, ros_package_path
            del dirs[:]
            continue #leaf
        elif 'rospack_nosubdirs' in files:
            del dirs[:]
            continue #leaf
        #small optimization
        elif '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')

        for sub_d in dirs:
            # followlinks=True only available in Python 2.6, so we
            # have to implement manually
            sub_p = os.path.join(d, sub_d)
            if os.path.islink(sub_p):
                packages.extend(list_pkgs_by_path(sub_p, cache=cache))
            
    return packages

def find_node(pkg, node_type, rospack=None):
    """
    Warning: unstable API due to catkin.

    Locate the executable that implements the node
    
    :param node_type: type of node, ``str``
    :returns: path to node or None if node is not in the package ``str``
    :raises: :exc:rospkg.ResourceNotFound` If package does not exist 
    """

    if rospack is None:
        rospack = rospkg.RosPack()
    return find_resource(pkg, node_type, filter_fn=_executable_filter, rospack=rospack)

def _executable_filter(test_path):
    s = os.stat(test_path)
    flags = stat.S_IRUSR | stat.S_IXUSR
    if os.name == 'nt' and os.path.splitext(test_path)[1] == '.py':
        flags = stat.S_IRUSR
    return (s.st_mode & flags) == flags

def _find_resource(d, resource_name, filter_fn=None):
    """
    subroutine of find_resource
    """
    matches = []
    # TODO: figure out how to generalize find_resource to take multiple resource name options
    if sys.platform in ['win32', 'cygwin']:
        # Windows logic requires more file patterns to resolve and is
        # not case-sensitive, so leave it separate

        # in the near-term, just hack in support for .exe/.bat/.py. In the long
        # term this needs to:
        #
        #  * parse PATHEXT to generate matches
        #  * perform case-insensitive compares against potential
        #    matches, in path-ext order

        # - We still have to look for bare node_type as user may have
        #   specified extension manually
        resource_name = resource_name.lower()
        patterns = [resource_name, resource_name+'.exe', resource_name+'.bat', resource_name+'.py']
        for p, dirs, files in os.walk(d):
            # case insensitive
            files = [f.lower() for f in files]
            for name in patterns:
                if name in files:
                    test_path = os.path.join(p, name)
                    if filter_fn is not None:
                        if filter_fn(test_path):
                            matches.append(test_path)
                    else:
                        matches.append(test_path)
            # remove .svn/.git/etc
            to_prune = [x for x in dirs if x.startswith('.')]
            for x in to_prune:
                dirs.remove(x)
    else: #UNIX            
        for p, dirs, files in os.walk(d):
            if resource_name in files:
                test_path = os.path.join(p, resource_name)
                if filter_fn is not None:
                    if filter_fn(test_path):
                        matches.append(test_path)
                else:
                    matches.append(test_path)
            # remove .svn/.git/etc
            to_prune = [x for x in dirs if x.startswith('.')]
            for x in to_prune:
                dirs.remove(x)
    return [os.path.abspath(m) for m in matches]

# TODO: this routine really belongs in rospkg, but the catkin-isms really, really don't
# belong in rospkg.  With more thought, they can probably be abstracted out so as
# to no longer be catkin-specific. 
def find_resource(pkg, resource_name, filter_fn=None, rospack=None):
    """
    Warning: unstable API due to catkin.

    Locate the file named resource_name in package, optionally
    matching specified filter.  find_resource() will return a list of
    matches, but only for a given scope.  If the resource is found in
    the binary build directory, it will only return matches in that
    directory; it will not return matches from the ROS_PACKAGE_PATH as
    well in this case.
    
    :param filter: function that takes in a path argument and
        returns True if the it matches the desired resource, ``fn(str)``
    :param rospack: `rospkg.RosPack` instance to use
    :returns: lists of matching paths for resource within a given scope, ``[str]``
    :raises: :exc:`rospkg.ResourceNotFound` If package does not exist 
    """

    # New resource-location policy in Fuerte, induced by the new catkin 
    # build system:
    #   (1) Use catkin_find to find libexec and share locations, look
    #       recursively there.  If the resource is found, done.
    #       Else continue:
    #   (2) If ROS_PACKAGE_PATH is set, look recursively there.  If the
    #       resource is found, done.  Else raise
    #
    # NOTE: package *must* exist on ROS_PACKAGE_PATH no matter what

    if rospack is None:
        rospack = rospkg.RosPack()

    # lookup package as it *must* exist
    pkg_path = rospack.get_path(pkg)

    source_path_to_packages = rospack.get_custom_cache('source_path_to_packages', {})

    # if found in binary dir, start with that.  in any case, use matches
    # from ros_package_path
    matches = []
    search_paths = catkin_find(
        search_dirs=['libexec', 'share'], project=pkg, first_matching_workspace_only=True,
        source_path_to_packages=source_path_to_packages)

    # persist mapping of packages in rospack instance
    if source_path_to_packages:
        rospack.set_custom_cache('source_path_to_packages', source_path_to_packages)

    for search_path in search_paths:
        matches.extend(_find_resource(search_path, resource_name, filter_fn=filter_fn))

    matches.extend(_find_resource(pkg_path, resource_name, filter_fn=filter_fn))

    # Uniquify the results, in case we found the same file twice, while keeping order
    unique_matches = []
    for match in matches:
        if match not in unique_matches:
            unique_matches.append(match)
    return unique_matches
