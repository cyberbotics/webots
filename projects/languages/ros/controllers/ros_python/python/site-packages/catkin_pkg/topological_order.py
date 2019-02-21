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

import copy
import os
import sys

from .packages import find_packages
from .workspaces import get_spaces


class _PackageDecorator(object):

    def __init__(self, package, path):
        self.package = package
        self.path = path
        self.is_metapackage = 'metapackage' in [e.tagname for e in self.package.exports]
        message_generators = [e.content for e in self.package.exports if e.tagname == 'message_generator']
        self.message_generator = message_generators[0] if message_generators else None
        # full includes direct build depends and recursive run_depends of these build_depends
        self.depends_for_topological_order = None

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        return getattr(self.package, name)

    def calculate_depends_for_topological_order(self, packages):
        """
        Set self.depends_for_topological_order to the recursive dependencies required for topological order.

        It contains all direct build- and buildtool dependencies and their recursive
        runtime dependencies. The set only contains packages which
        are in the passed packages dictionary.

        :param packages: dict of name to ``_PackageDecorator``
        """
        self.depends_for_topological_order = set()
        all_depends = self.package.build_depends + self.package.buildtool_depends + self.package.test_depends
        names = [d.name for d in all_depends if d.evaluated_condition]

        # collect all group dependencies
        for group_depend in self.package.group_depends:
            if group_depend.evaluated_condition:
                assert group_depend.members is not None, \
                    'Group members need to be determined before'
                names += group_depend.members

        # skip external dependencies, meaning names that are not known packages
        for name in [n for n in names if n in packages.keys()]:
            if not self.is_metapackage and packages[name].is_metapackage:
                print('WARNING: package "%s" should not depend on metapackage "%s" but on its packages instead' % (self.name, name), file=sys.stderr)
            if name in self.depends_for_topological_order:
                # avoid function call to improve performance
                # check within the loop since the set changes every cycle
                continue
            packages[name]._add_recursive_run_depends(packages, self.depends_for_topological_order)

    def _add_recursive_run_depends(self, packages, depends_for_topological_order):
        """
        Modify depends_for_topological_order argument by adding run_depends of self recursively.

        Only packages which are in the passed packages are added and recursed into.

        :param packages: dict of name to ``_PackageDecorator``
        :param depends_for_topological_order: set to be extended
        """
        depends_for_topological_order.add(self.package.name)
        package_names = packages.keys()
        names = [d.name for d in self.package.run_depends if d.evaluated_condition]

        for group_depend in self.package.group_depends:
            if group_depend.evaluated_condition:
                assert group_depend.members is not None, \
                    'Group members need to be determined before'
                names += group_depend.members

        for name in [n for n in names
                     if (n in package_names and
                         n not in depends_for_topological_order)]:
            packages[name]._add_recursive_run_depends(packages, depends_for_topological_order)


def topological_order(root_dir, whitelisted=None, blacklisted=None, underlay_workspaces=None):
    """
    Crawls the filesystem to find packages and uses their dependencies to return a topologically order list.

    When a circular dependency is detected, the last item in the returned list
    is a tuple with None and a string giving a superset of the guilty packages.

    :param root_dir: The path to search in, ``str``
    :param whitelisted: A list of whitelisted package names, ``list``
    :param blacklisted: A list of blacklisted package names, ``list``
    :param underlay_workspaces: A list of underlay workspaces of packages which might provide dependencies in case of partial workspaces, ``list``
    :returns: A list of tuples containing the relative path and a ``Package`` object, ``list``
    """
    packages = find_packages(root_dir)

    # find packages in underlayed workspaces
    underlay_packages = {}
    if underlay_workspaces:
        for workspace in reversed(underlay_workspaces):
            # since underlay workspace might be a devel space
            # consider spaces stored in the .catkin file
            spaces = get_spaces([workspace])
            for space in spaces:
                for path, package in find_packages(space).items():
                    underlay_packages[package.name] = (path, package)

    return topological_order_packages(packages, whitelisted=whitelisted, blacklisted=blacklisted, underlay_packages=dict(underlay_packages.values()))


def topological_order_packages(packages, whitelisted=None, blacklisted=None, underlay_packages=None):
    """
    Topologically orders packages.

    evaluate_conditions() will be called for each package.

    If group dependencies haven't determined their members yet
    extract_group_members() will be called for each group dependency to do so.

    First returning packages which have message generators and then
    the rest based on direct build-/buildtool_depends and indirect
    recursive run_depends.

    When a circular dependency is detected, the last item in the returned list
    is a tuple with None and a string giving a superset of the guilty packages.

    :param packages: A dict mapping relative paths to ``Package`` objects ``dict``
    :param whitelisted: A list of whitelisted package names, ``list``
    :param blacklisted: A list of blacklisted package names, ``list``
    :param underlay_packages: A dict mapping relative paths to ``Package`` objects ``dict``
    :returns: A list of tuples containing the relative path and a ``Package`` object, ``list``
    """
    decorators_by_name = {}
    for path, package in packages.items():
        # skip non-whitelisted packages
        if whitelisted and package.name not in whitelisted:
            continue
        # skip blacklisted packages
        if blacklisted and package.name in blacklisted:
            continue
        if package.name in decorators_by_name:
            path_with_same_name = decorators_by_name[package.name].path
            raise RuntimeError('Two packages with the same name "%s" in the workspace:\n- %s\n- %s' % (package.name, path_with_same_name, path))
        decorators_by_name[package.name] = _PackageDecorator(package, path)

    underlay_decorators_by_name = {}
    if underlay_packages:
        for path, package in underlay_packages.items():
            # skip overlayed packages
            if package.name in decorators_by_name:
                continue
            underlay_decorators_by_name[package.name] = _PackageDecorator(package, path)
        decorators_by_name.update(underlay_decorators_by_name)

    # evaluate conditions and determine group membership
    pkgs = [d.package for d in decorators_by_name.values()]
    for pkg in pkgs:
        pkg.evaluate_conditions(os.environ)
    for pkg in pkgs:
        for group_depend in pkg.group_depends:
            if group_depend.evaluated_condition:
                group_depend.extract_group_members(pkgs)

    # calculate transitive dependencies
    for decorator in decorators_by_name.values():
        decorator.calculate_depends_for_topological_order(decorators_by_name)

    tuples = _sort_decorated_packages(decorators_by_name)
    # remove underlay packages from result
    return [(path, package) for path, package in tuples if path is None or package.name not in underlay_decorators_by_name]


def _reduce_cycle_set(packages_orig):
    """
    Remove iteratively some packages from a set that are definitely not part of any cycle.

    When there is a cycle in the package dependencies,
    _sort_decorated_packages only knows the set of packages containing
    the cycle.
    :param packages: A dict mapping package name to ``_PackageDecorator`` objects ``dict``
    :returns: A list of package names from the input which could not easily be detected as not being part of a cycle.
    """
    assert(packages_orig)
    packages = copy.copy(packages_orig)
    last_depended = None
    while len(packages) > 0:
        depended = set()
        for name, decorator in packages.items():
            if decorator.depends_for_topological_order:
                depended = depended.union(decorator.depends_for_topological_order)
        for name in list(packages.keys()):
            if name not in depended:
                del packages[name]
        if last_depended:
            if last_depended == depended:
                return packages.keys()
        last_depended = depended


def _sort_decorated_packages(packages_orig):
    """
    Sorts packages according to dependency ordering.

    First considering the message generators and their recursive dependencies
    and then the rest of the packages.
    When a circle is detected, a tuple with None and a string giving a
    superset of the guilty packages.

    :param packages: A dict mapping package name to ``_PackageDecorator`` objects ``dict``
    :returns: A List of tuples containing the relative path and a ``Package`` object ``list``
    """
    packages = copy.deepcopy(packages_orig)

    # mark all packages which are (recursively) dependent on by message generators
    dependency_names_to_follow = set([name for name, decorator in packages.items() if decorator.message_generator])
    not_marked_package_names = set(packages.keys()) - dependency_names_to_follow
    while dependency_names_to_follow:
        pkg_name = dependency_names_to_follow.pop()
        for name in packages[pkg_name].depends_for_topological_order:
            if name in not_marked_package_names:
                # mark package
                packages[name].message_generator = True
                not_marked_package_names.remove(name)
                # queue for recursion
                dependency_names_to_follow.add(name)

    ordered_packages = []
    while len(packages) > 0:
        # find all packages without build dependencies
        message_generators = []
        non_message_generators = []
        for name, decorator in packages.items():
            if not decorator.depends_for_topological_order:
                if decorator.message_generator:
                    message_generators.append(name)
                else:
                    non_message_generators.append(name)
        # first choose message generators
        if message_generators:
            names = message_generators
        elif non_message_generators:
            names = non_message_generators
        else:
            # in case of a circular dependency pass a string with
            # the names list of remaining package names, with path
            # None to indicate cycle
            ordered_packages.append([None, ', '.join(sorted(_reduce_cycle_set(packages)))])
            break

        # alphabetic order only for convenience
        names.sort()

        # add first candidates to ordered list
        # do not add all candidates since removing the depends from the first might affect the next candidates
        name = names[0]
        ordered_packages.append([packages[name].path, packages[name].package])
        # remove package from further processing
        del packages[name]
        for package in packages.values():
            if name in package.depends_for_topological_order:
                package.depends_for_topological_order.remove(name)
    return ordered_packages
