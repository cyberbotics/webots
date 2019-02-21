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

"""Library to find packages in the filesystem."""

import multiprocessing
import os

from .package import _get_package_xml
from .package import PACKAGE_MANIFEST_FILENAME
from .package import parse_package_string


def find_package_paths(basepath, exclude_paths=None, exclude_subspaces=False):
    """
    Crawls the filesystem to find package manifest files.

    When a subfolder contains either of the following files it is ignored:
    - ``AMENT_IGNORE``
    - ``CATKIN_IGNORE``
    - ``COLCON_IGNORE``

    :param basepath: The path to search in, ``str``
    :param exclude_paths: A list of paths which should not be searched, ``list``
    :param exclude_subspaces: The flag is subfolders containing a .catkin file should not be
        searched, ``bool``
    :returns: A list of relative paths containing package manifest files ``list``
    """
    paths = []
    real_exclude_paths = [os.path.realpath(p) for p in exclude_paths] if exclude_paths is not None else []
    for dirpath, dirnames, filenames in os.walk(basepath, followlinks=True):
        if 'AMENT_IGNORE' in filenames or \
            'CATKIN_IGNORE' in filenames or \
            'COLCON_IGNORE' in filenames or \
            os.path.realpath(dirpath) in real_exclude_paths or \
                (exclude_subspaces and '.catkin' in filenames):
            del dirnames[:]
            continue
        elif PACKAGE_MANIFEST_FILENAME in filenames:
            paths.append(os.path.relpath(dirpath, basepath))
            del dirnames[:]
            continue
        # filter out hidden directories in-place
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
    return paths


def find_packages(basepath, exclude_paths=None, exclude_subspaces=False, warnings=None):
    """
    Crawls the filesystem to find package manifest files and parses them.

    :param basepath: The path to search in, ``str``
    :param exclude_paths: A list of paths which should not be searched, ``list``
    :param exclude_subspaces: The flag is subfolders containing a .catkin file should not be
        searched, ``bool``
    :param warnings: Print warnings if None or return them in the given list
    :returns: A dict mapping relative paths to ``Package`` objects ``dict``
    :raises: :exc:RuntimeError` If multiple packages have the same name
    """
    packages = find_packages_allowing_duplicates(basepath, exclude_paths=exclude_paths, exclude_subspaces=exclude_subspaces, warnings=warnings)
    package_paths_by_name = {}
    for path, package in packages.items():
        if package.name not in package_paths_by_name:
            package_paths_by_name[package.name] = set()
        package_paths_by_name[package.name].add(path)
    duplicates = dict([(name, paths) for name, paths in package_paths_by_name.items() if len(paths) > 1])
    if duplicates:
        duplicates = ['Multiple packages found with the same name "%s":%s' % (name, ''.join(['\n- %s' % path_ for path_ in sorted(duplicates[name])])) for name in sorted(duplicates.keys())]
        raise RuntimeError('\n'.join(duplicates))
    return packages


class _PackageParser(object):

    def __init__(self, capture_warnings):
        self.capture_warnings = capture_warnings

    def __call__(self, xml_and_path_and_filename):
        xml, path, filename = xml_and_path_and_filename
        warnings = [] if self.capture_warnings else None
        parsed_package = parse_package_string(xml, filename=filename, warnings=warnings)
        return (path, parsed_package), warnings


def find_packages_allowing_duplicates(basepath, exclude_paths=None, exclude_subspaces=False, warnings=None):
    """
    Crawls the filesystem to find package manifest files and parses them.

    :param basepath: The path to search in, ``str``
    :param exclude_paths: A list of paths which should not be searched, ``list``
    :param exclude_subspaces: The flag is subfolders containing a .catkin file should not be
        searched, ``bool``
    :param warnings: Print warnings if None or return them in the given list
    :returns: A dict mapping relative paths to ``Package`` objects ``dict``
    """
    package_paths = find_package_paths(basepath, exclude_paths=exclude_paths, exclude_subspaces=exclude_subspaces)

    xmls = {}
    for path in package_paths:
        xmls[path] = _get_package_xml(os.path.join(basepath, path))

    data = [(v[0], k, v[1]) for k, v in xmls.items()]

    if not data:
        return {}

    parallel = len(data) > 100
    if parallel:
        try:
            pool = multiprocessing.Pool()
        except OSError:
            # On chroot environment, multiprocessing is not available
            # https://stackoverflow.com/questions/6033599/oserror-38-errno-38-with-multiprocessing
            parallel = False

    if not parallel:
        #  use sequential loop
        parsed_packages = {}
        for xml, path, filename in data:
            parsed_package = parse_package_string(
                xml, filename=filename, warnings=warnings)
            parsed_packages[path] = parsed_package
        return parsed_packages

    # use multiprocessing pool
    parser = _PackageParser(warnings is not None)
    try:
        path_parsed_packages, warnings_lists = zip(*pool.map(parser, data))
    finally:
        pool.close()
        pool.join()
    if parser.capture_warnings:
        map(warnings.extend, warnings_lists)
    return dict(path_parsed_packages)


def verify_equal_package_versions(packages):
    """
    Verify that all packages have the same version number.

    :param packages: The list of ``Package`` objects, ``list``
    :returns: The version number
    :raises: :exc:RuntimeError` If the version is not equal in all packages
    """
    version = None
    for package in packages:
        if version is None:
            version = package.version
        elif package.version != version:
            raise RuntimeError('Two packages have different version numbers (%s != %s):\n- %s\n- %s' % (package.version, version, package.filename, packages[0].filename))
    return version
