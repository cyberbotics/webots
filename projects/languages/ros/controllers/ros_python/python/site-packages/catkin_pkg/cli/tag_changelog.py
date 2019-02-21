"""This script renames the forthcoming section in changelog files with the upcoming version and the current date."""

from __future__ import print_function

import argparse
import datetime
import os
import re
import sys

from catkin_pkg.changelog import CHANGELOG_FILENAME, get_changelog_from_path
from catkin_pkg.changelog_generator import FORTHCOMING_LABEL
from catkin_pkg.package_version import bump_version
from catkin_pkg.packages import find_packages, verify_equal_package_versions

import docutils.core


def get_forthcoming_label(rst):
    document = docutils.core.publish_doctree(rst)
    forthcoming_label = None
    for child in document.children:
        title = None
        if isinstance(child, docutils.nodes.subtitle):
            title = child
        elif isinstance(child, docutils.nodes.section):
            section = child
            if len(section.children) > 0 and isinstance(section.children[0], docutils.nodes.title):
                title = section.children[0]
        if title and len(title.children) > 0 and isinstance(title.children[0], docutils.nodes.Text):
            title_text = title.children[0].rawsource
            if FORTHCOMING_LABEL.lower() in title_text.lower():
                if forthcoming_label:
                    raise RuntimeError('Found multiple forthcoming sections')
                forthcoming_label = title_text
    return forthcoming_label


def rename_section(data, old_label, new_label):
    valid_section_characters = '!"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~'

    def replace_section(match):
        section_char = match.group(2)[0]
        return new_label + '\n' + section_char * len(new_label)
    pattern = '^(' + re.escape(old_label) + ')\n([' + re.escape(valid_section_characters) + ']+)$'
    data, count = re.subn(pattern, replace_section, data, flags=re.MULTILINE)
    if count == 0:
        raise RuntimeError('Could not find section')
    if count > 1:
        raise RuntimeError('Found multiple matching sections')
    return data


def main(sysargs=None):
    parser = argparse.ArgumentParser(description='Tag the forthcoming section in the changelog files with an upcoming version number')
    parser.add_argument('--bump', choices=('major', 'minor', 'patch'), default='patch', help='Which part of the version number to bump? (default: %(default)s)')
    args = parser.parse_args(sysargs)

    base_path = '.'

    # find packages
    packages = find_packages(base_path)
    if not packages:
        raise RuntimeError('No packages found')
    print('Found packages: %s' % ', '.join([p.name for p in packages.values()]))

    # fetch current version and verify that all packages have same version number
    old_version = verify_equal_package_versions(packages.values())
    new_version = bump_version(old_version, args.bump)
    print('Tag version %s' % new_version)

    # check for changelog entries
    changelogs = []
    missing_forthcoming = []
    already_tagged = []
    for pkg_path, package in packages.items():
        changelog_path = os.path.join(base_path, pkg_path, CHANGELOG_FILENAME)
        if not os.path.exists(changelog_path):
            missing_forthcoming.append(package.name)
            continue
        changelog = get_changelog_from_path(changelog_path, package.name)
        if not changelog:
            missing_forthcoming.append(package.name)
            continue
        # check that forthcoming section exists
        forthcoming_label = get_forthcoming_label(changelog.rst)
        if not forthcoming_label:
            missing_forthcoming.append(package.name)
            continue
        # check that new_version section does not exist yet
        try:
            changelog.get_content_of_version(new_version)
            already_tagged.append(package.name)
            continue
        except KeyError:
            pass
        changelogs.append((package.name, changelog_path, changelog, forthcoming_label))
    if missing_forthcoming:
        print('The following packages do not have a forthcoming section in their changelog file: %s' % ', '.join(sorted(missing_forthcoming)), file=sys.stderr)
    if already_tagged:
        print("The following packages do already have a section '%s' in their changelog file: %s" % (new_version, ', '.join(sorted(already_tagged))), file=sys.stderr)

    # rename forthcoming sections to new_version including current date
    new_changelog_data = []
    new_label = '%s (%s)' % (new_version, datetime.date.today().isoformat())
    for (pkg_name, changelog_path, changelog, forthcoming_label) in changelogs:
        print("Renaming section '%s' to '%s' in package '%s'..." % (forthcoming_label, new_label, pkg_name))
        data = rename_section(changelog.rst, forthcoming_label, new_label)
        new_changelog_data.append((changelog_path, data))

    print('Writing updated changelog files...')
    for (changelog_path, data) in new_changelog_data:
        with open(changelog_path, 'wb') as f:
            f.write(data.encode('utf-8'))
