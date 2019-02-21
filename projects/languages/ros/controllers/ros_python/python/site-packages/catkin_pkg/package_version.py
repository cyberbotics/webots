# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
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

import datetime
import os
import re

from catkin_pkg.changelog_generator import FORTHCOMING_LABEL
import docutils.core


def bump_version(version, bump='patch'):
    """
    Increases version number.

    :param str version: must be in version format "int.int.int"
    :param str bump: one of 'patch, minor, major'
    :returns: version with the given part increased, and all inferior parts reset to 0
    :rtype: str
    :raises ValueError: if the version string is not in the format x.y.z
    """
    # split the version number
    match = re.match('^(\d+)\.(\d+)\.(\d+)$', version)
    if match is None:
        raise ValueError('Invalid version string, must be int.int.int: "%s"' % version)
    new_version = match.groups()
    new_version = [int(x) for x in new_version]
    # find the desired index
    idx = {'major': 0, 'minor': 1, 'patch': 2}[bump]
    # increment the desired part
    new_version[idx] += 1
    # reset all parts behind the bumped part
    new_version = new_version[:idx + 1] + [0 for x in new_version[idx + 1:]]
    return '%d.%d.%d' % tuple(new_version)


def _replace_version(package_str, new_version):
    """
    Replace the version tag in contents if there is only one instance.

    :param str package_str: contents of package.xml
    :param str new_version: version number
    :returns: new package.xml
    :rtype: str
    :raises RuntimeError:
    """
    # try to replace contens
    new_package_str, number_of_subs = re.subn('<version([^<>]*)>[^<>]*</version>', '<version\g<1>>%s</version>' % new_version, package_str)
    if number_of_subs != 1:
        raise RuntimeError('Illegal number of version tags: %s' % (number_of_subs))
    return new_package_str


def _check_for_version_comment(package_str, new_version):
    """
    Check if a comment is present behind the version tag and return it.

    :param str package_str: contents of package.xml
    :param str new_version: version number
    :returns: comment if available, else None
    :rtype: str
    """
    version_tag = '>%s</version>' % new_version
    pattern = '%s[ \t]*%s *(.+) *%s' % (re.escape(version_tag), re.escape('<!--'), re.escape('-->'))
    comment = re.search(pattern, package_str)
    if comment:
        comment = comment.group(1)
    return comment


def update_versions(paths, new_version):
    """
    Bulk replace of version: searches for package.xml files directly in given folders and replaces version tag within.

    :param list paths: folder names
    :param str new_version: version string "int.int.int"
    :raises RuntimeError: if any one package.xml cannot be updated
    """
    files = {}
    for path in paths:
        package_path = os.path.join(path, 'package.xml')
        with open(package_path, 'r') as f:
            package_str = f.read()
        try:
            new_package_str = _replace_version(package_str, new_version)
            comment = _check_for_version_comment(new_package_str, new_version)
            if comment:
                print('NOTE: The package manifest "%s" contains a comment besides the version tag:\n  %s' % (path, comment))
        except RuntimeError as rue:
            raise RuntimeError('Could not bump version number in file %s: %s' % (package_path, str(rue)))
        files[package_path] = new_package_str
    # if all replacements successful, write back modified package.xml
    for package_path, new_package_str in files.items():
        with open(package_path, 'w') as f:
            f.write(new_package_str)


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


def update_changelog_sections(changelogs, new_version):
    # rename forthcoming sections to new_version including current date
    new_changelog_data = {}
    new_label = '%s (%s)' % (new_version, datetime.date.today().isoformat())
    for pkg_name, (changelog_path, changelog, forthcoming_label) in changelogs.items():
        data = rename_section(changelog.rst, forthcoming_label, new_label)
        new_changelog_data[changelog_path] = data

    for changelog_path, data in new_changelog_data.items():
        with open(changelog_path, 'wb') as f:
            f.write(data.encode('utf-8'))


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
