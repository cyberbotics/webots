"""This script finds a catkin packages."""

from __future__ import print_function

import argparse
import os
import sys

from catkin_pkg.packages import find_packages


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='Find a catkin package')
    parser.add_argument('pkg', help='The name of the package')
    parser.add_argument('base_path', nargs='?', default=os.curdir, help='The base path to crawl for packages')

    args = parser.parse_args(argv)

    try:
        packages = find_packages(args.base_path)
        catkin_pkg = [path for path, p in packages.items() if p.name == args.pkg]
        if catkin_pkg:
            print(catkin_pkg[0])
        else:
            print("Could not find package '%s'." % args.pkg, file=sys.stderr)
            sys.exit(2)
    except RuntimeError as e:
        print('ERROR: ' + str(e), file=sys.stderr)
        sys.exit(1)
