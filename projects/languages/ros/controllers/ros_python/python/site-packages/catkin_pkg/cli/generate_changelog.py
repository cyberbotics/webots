"""This script generates REP-0132 CHANGELOG.rst files for git or hg repositories."""

from __future__ import print_function

import argparse
import logging
import os
import sys

from catkin_pkg.changelog import CHANGELOG_FILENAME
from catkin_pkg.changelog_generator import generate_changelog_file, generate_changelogs, get_all_changes, get_forthcoming_changes, update_changelogs
from catkin_pkg.changelog_generator_vcs import get_vcs_client
from catkin_pkg.packages import find_packages

try:
    raw_input
except NameError:
    raw_input = input  # noqa: A001


def prompt_continue(msg, default):
    """Prompt the user for continuation."""
    if default:
        msg += ' [Y/n]?'
    else:
        msg += ' [y/N]?'

    while True:
        response = raw_input(msg)
        if not response:
            response = 'y' if default else 'n'
        else:
            response = response.lower()

        if response in ['y', 'n']:
            return response == 'y'

        print("Response '%s' was not recognized, please use one of the following options: y, Y, n, N" % response, file=sys.stderr)


def main(sysargs=None):
    parser = argparse.ArgumentParser(description='Generate a REP-0132 %s' % CHANGELOG_FILENAME)
    parser.add_argument(
        '-a', '--all', action='store_true', default=False,
        help='Generate changelog for all versions instead of only the forthcoming one (only supported when no changelog file exists yet)')
    parser.add_argument(
        '--print-root', action='store_true', default=False,
        help='Output changelog content to the console as if there would be only one package in the root of the repository')
    parser.add_argument(
        '--skip-contributors', action='store_true', default=False,
        help='Skip adding the list of contributors to the changelog')
    parser.add_argument(
        '--skip-merges', action='store_true', default=False,
        help='Skip adding merge commits to the changelog')
    parser.add_argument(
        '-y', '--non-interactive', action='store_true', default=False,
        help="Run without user interaction, confirming all questions with 'yes'")
    args = parser.parse_args(sysargs)

    base_path = '.'
    logging.basicConfig(format='%(message)s', level=logging.DEBUG)

    vcs_client = get_vcs_client(base_path)

    if args.print_root:
        # printing status messages to stderr to allow piping the changelog to a file
        if args.all:
            print('Querying all tags and commit information...', file=sys.stderr)
            tag2log_entries = get_all_changes(vcs_client, skip_merges=args.skip_merges)
            print('Generating changelog output with all versions...', file=sys.stderr)
        else:
            print('Querying commit information since latest tag...', file=sys.stderr)
            tag2log_entries = get_forthcoming_changes(vcs_client, skip_merges=args.skip_merges)
            print('Generating changelog files with forthcoming version...', file=sys.stderr)
        print('', file=sys.stderr)
        data = generate_changelog_file('repository-level', tag2log_entries, vcs_client=vcs_client)
        print(data)
        return 0

    # find packages
    packages = find_packages(base_path)
    if not packages:
        raise RuntimeError('No packages found')
    print('Found packages: %s' % ', '.join(sorted(p.name for p in packages.values())))

    # check for missing changelogs
    missing_changelogs = []
    for pkg_path, package in packages.items():
        changelog_path = os.path.join(base_path, pkg_path, CHANGELOG_FILENAME)
        if not os.path.exists(changelog_path):
            missing_changelogs.append(package.name)

    if args.all and not missing_changelogs:
        raise RuntimeError('All packages already have a changelog. Either remove (some of) them before using --all or invoke the script without --all.')

    if args.all and len(missing_changelogs) != len(packages):
        ignored = set([p.name for p in packages.values()]) - set(missing_changelogs)
        print('The following packages already have a changelog file and will be ignored: %s' % ', '.join(sorted(ignored)), file=sys.stderr)

    # prompt to switch to --all
    if not args.all and missing_changelogs:
        print('Some of the packages have no changelog file: %s' % ', '.join(sorted(missing_changelogs)))
        print('You might consider to use --all to generate the changelogs for all versions (not only for the forthcoming version).')
        if not args.non_interactive and not prompt_continue('Continue without --all option', default=False):
            raise RuntimeError('Skipping generation, rerun the script with --all.')

    if args.all:
        print('Querying all tags and commit information...')
        tag2log_entries = get_all_changes(vcs_client, skip_merges=args.skip_merges)
        print('Generating changelog files with all versions...')
        generate_changelogs(base_path, packages, tag2log_entries, logger=logging, vcs_client=vcs_client, skip_contributors=args.skip_contributors)
    else:
        print('Querying commit information since latest tag...')
        tag2log_entries = get_forthcoming_changes(vcs_client, skip_merges=args.skip_merges)
        # separate packages with/without a changelog file
        packages_without = {pkg_path: package for pkg_path, package in packages.items() if package.name in missing_changelogs}
        if packages_without:
            print('Generating changelog files with forthcoming version...')
            generate_changelogs(base_path, packages_without, tag2log_entries, logger=logging, vcs_client=vcs_client, skip_contributors=args.skip_contributors)
        packages_with = {pkg_path: package for pkg_path, package in packages.items() if package.name not in missing_changelogs}
        if packages_with:
            print('Updating forthcoming section of changelog files...')
            update_changelogs(base_path, packages_with, tag2log_entries, logger=logging, vcs_client=vcs_client, skip_contributors=args.skip_contributors)
    print('Done.')
    print('Please review the extracted commit messages and consolidate the changelog entries before committing the files!')


def main_catching_runtime_error(*args, **kwargs):
    try:
        main(*args, **kwargs)
    except RuntimeError as e:
        print('ERROR: ' + str(e), file=sys.stderr)
        sys.exit(1)
