"""This script tests REP-0132 changelog files."""

from __future__ import print_function

import argparse
import logging
import os
import sys

import catkin_pkg.changelog
from catkin_pkg.changelog import Changelog, CHANGELOG_FILENAME
from catkin_pkg.changelog import populate_changelog_from_rst


def main(sysargs=None):
    parser = argparse.ArgumentParser(
        description='Tests a REP-0132 %s' % CHANGELOG_FILENAME)
    parser.add_argument(
        'changelog_file',
        help='%s file to parse' % CHANGELOG_FILENAME,
        default='.',
        nargs='?')

    args = parser.parse_args(sysargs)

    if os.path.isdir(args.changelog_file):
        changelog_file = os.path.join(args.changelog_file, CHANGELOG_FILENAME)
        if not os.path.exists(changelog_file):
            print("No {0} file in given directory: '{1}'"
                  .format(CHANGELOG_FILENAME, args.changelog_file), file=sys.stderr)
            return 1
    else:
        changelog_file = args.changelog_file
        if not os.path.exists(changelog_file):
            print("{0} file given does not exist: '{1}'"
                  .format(CHANGELOG_FILENAME, args.changelog_file), file=sys.stderr)
            return 1

    if os.path.basename(changelog_file) != CHANGELOG_FILENAME:
        print('WARNING: changelog file name should be %s' % CHANGELOG_FILENAME)

    logging.basicConfig()
    catkin_pkg.changelog.log.setLevel(logging.DEBUG)
    changelog = Changelog()
    with open(changelog_file, 'r') as f:
        print(populate_changelog_from_rst(changelog, f.read()))
