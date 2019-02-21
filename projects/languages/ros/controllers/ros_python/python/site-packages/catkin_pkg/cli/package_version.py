from __future__ import print_function

import argparse
import os
import sys

from catkin_pkg.package_version import bump_version
from catkin_pkg.package_version import update_versions
from catkin_pkg.packages import find_packages, verify_equal_package_versions

# find the import relatively if available to work before installing catkin or overlaying installed version
if os.path.exists(os.path.join(os.path.dirname(__file__), '..', 'python', 'catkin', '__init__.py')):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))


def main():
    parser = argparse.ArgumentParser(description='Show or bump the version number in package.xml files.')
    parser.add_argument('path', nargs='?', default='.', help='The path to a parent folder which contains package.xml files (default: .)')
    parser.add_argument('--bump', choices=('major', 'minor', 'patch'), help='Which part of the version number to bump?')
    args = parser.parse_args()

    try:
        packages = find_packages(args.path)
        if not packages:
            print('No packages found', file=sys.stderr)
            sys.exit(1)
        version = verify_equal_package_versions(packages.values())

        # only print the version number
        if args.bump is None:
            print(version)

        else:
            # bump the version number
            new_version = bump_version(version, args.bump)
            update_versions(packages.keys(), new_version)
            print('%s -> %s' % (version, new_version))
    except Exception as e:
        sys.exit(str(e))
