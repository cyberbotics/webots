# Software License Agreement (BSD License)
#
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

"""
Generate/update ROS changelog files.

The Changelog format is described in REP-0132:

http://ros.org/reps/rep-0132.html
"""

import os
import re

from catkin_pkg.changelog import CHANGELOG_FILENAME
from catkin_pkg.changelog_generator_vcs import Tag

FORTHCOMING_LABEL = 'Forthcoming'


def get_all_changes(vcs_client, skip_merges=False):
    tags = _get_version_tags(vcs_client)

    # query all log entries per tag range
    tag2log_entries = {}
    previous_tag = Tag(None)
    for tag in sorted_tags(tags):
        log_entries = vcs_client.get_log_entries(
            from_tag=previous_tag.name, to_tag=tag.name, skip_merges=skip_merges)
        tag2log_entries[previous_tag] = log_entries
        previous_tag = tag
    log_entries = vcs_client.get_log_entries(
        from_tag=previous_tag.name, to_tag=None, skip_merges=skip_merges)
    tag2log_entries[previous_tag] = log_entries
    return tag2log_entries


def get_forthcoming_changes(vcs_client, skip_merges=False):
    tags = _get_version_tags(vcs_client)
    latest_tag_name = _get_latest_version_tag_name(vcs_client)

    # query log entries since latest tag only
    tag2log_entries = {}
    from_tag = Tag(None)
    to_tag = Tag(latest_tag_name)
    for tag in sorted_tags(tags):
        if to_tag.name is None:
            to_tag = tag
        # ignore non-forthcoming log entries but keep version to identify injection point of forthcoming
        tag2log_entries[tag] = None
    log_entries = vcs_client.get_log_entries(
        from_tag=from_tag.name, to_tag=to_tag.name, skip_merges=skip_merges)
    tag2log_entries[from_tag] = log_entries
    return tag2log_entries


def _get_version_tags(vcs_client):
    # get all tags in descending order
    tags = vcs_client.get_tags()
    version_tags = [t for t in tags if re.match(r'^\d+\.\d+.\d+$', t.name)]
    return version_tags


def _get_latest_version_tag_name(vcs_client):
    # get latest tag
    tag_name = vcs_client.get_latest_tag_name()
    if not re.match(r'^\d+\.\d+.\d+$', tag_name):
        raise RuntimeError(
            "The tag name '{}' doesn't match the version pattern x.y.z".format(tag_name))
    return tag_name


def generate_changelogs(base_path, packages, tag2log_entries, logger=None, vcs_client=None, skip_contributors=False):
    for pkg_path, package in packages.items():
        changelog_path = os.path.join(base_path, pkg_path, CHANGELOG_FILENAME)
        if os.path.exists(changelog_path):
            continue
        # generate package specific changelog file
        if logger:
            logger.debug("- creating '%s'" % os.path.join(pkg_path, CHANGELOG_FILENAME))
        pkg_tag2log_entries = filter_package_changes(tag2log_entries, pkg_path)
        data = generate_changelog_file(package.name, pkg_tag2log_entries, vcs_client=vcs_client, skip_contributors=skip_contributors)
        with open(changelog_path, 'wb') as f:
            f.write(data.encode('utf-8'))


def update_changelogs(base_path, packages, tag2log_entries, logger=None, vcs_client=None, skip_contributors=False):
    for pkg_path in packages.keys():
        # update package specific changelog file
        if logger:
            logger.debug("- updating '%s'" % os.path.join(pkg_path, CHANGELOG_FILENAME))
        pkg_tag2log_entries = filter_package_changes(tag2log_entries, pkg_path)
        changelog_path = os.path.join(base_path, pkg_path, CHANGELOG_FILENAME)
        with open(changelog_path, 'rb') as f:
            data = f.read().decode('utf-8')
        data = update_changelog_file(data, pkg_tag2log_entries, vcs_client=vcs_client, skip_contributors=skip_contributors)
        with open(changelog_path, 'wb') as f:
            f.write(data.encode('utf-8'))


def filter_package_changes(tag2log_entries, pkg_path):
    pkg_tag2log_entries = {}
    # collect all log entries relevant for this package
    for tag, log_entries in tag2log_entries.items():
        if log_entries is None:
            pkg_log_entries = None
        else:
            pkg_log_entries = []
            for log_entry in log_entries:
                if log_entry.affects_path(pkg_path):
                    pkg_log_entries.append(log_entry)
        pkg_tag2log_entries[tag] = pkg_log_entries
    return pkg_tag2log_entries


def generate_changelog_file(pkg_name, tag2log_entries, vcs_client=None, skip_contributors=False):
    blocks = []
    blocks.append(generate_package_headline(pkg_name))

    for tag in sorted_tags(tag2log_entries.keys()):
        log_entries = tag2log_entries[tag]
        if log_entries is not None:
            blocks.append(generate_version_block(tag.name, tag.timestamp, log_entries, vcs_client=vcs_client, skip_contributors=skip_contributors))

    return '\n'.join(blocks)


def update_changelog_file(data, tag2log_entries, vcs_client=None, skip_contributors=False):
    tags = sorted_tags(tag2log_entries.keys())
    for i, tag in enumerate(tags):
        log_entries = tag2log_entries[tag]
        if log_entries is None:
            continue
        content = generate_version_content(log_entries, vcs_client=vcs_client, skip_contributors=skip_contributors)

        # check if version section exists
        match = get_version_section_match(data, tag.name)
        if match:
            # prepend content to existing section
            data = prepend_version_content(data, tag.name, content)
            assert data is not None
        else:
            # find injection point of earliest following version
            for next_tag in list(tags)[i:]:
                match = get_version_section_match(data, next_tag.name)
                if match:
                    block = generate_version_block(tag.name, tag.timestamp, log_entries, vcs_client=vcs_client, skip_contributors=skip_contributors)
                    data = data[:match.start()] + block + '\n' + data[match.start():]
                    break
            if not match:
                if tag.name is None:
                    raise RuntimeError('Could not find section "%s"' % next_tag.name)
                else:
                    raise RuntimeError('Could neither find section "%s" nor any other section' % tag.name)
        return data


def get_version_section_match(data, version):
    pattern = get_version_section_pattern(version)
    matches = re.finditer(pattern, data, flags=re.MULTILINE)
    matches = list(matches)
    if len(matches) > 1:
        raise RuntimeError('Found multiple matching sections')
    return matches[0] if matches else None


def get_version_section_pattern(version):
    valid_section_characters = '!"#$%&\'()*+,-./:;<=>?@[\\]^_`{|}~'
    headline = get_version_headline(version, None)
    pattern = '^(' + re.escape(headline) + '( \([0-9 \-:|+]+\))?)\n([' + re.escape(valid_section_characters) + ']+)\n?$'
    return pattern


def prepend_version_content(data, version, content):
    pattern = get_version_section_pattern(version)

    def replace_section(match):
        headline = match.group(1)
        section = match.group(3)
        data = content.rstrip()
        if data:
            data += '\n'
        return headline + '\n' + section + '\n' + data

    data, count = re.subn(pattern, replace_section, data, flags=re.MULTILINE)
    if count > 1:
        raise RuntimeError('Found multiple matching sections')
    return data if count == 1 else None


def sorted_tags(tags):
    # first return the forthcoming tag
    for tag in tags:
        if not tag.name:
            yield tag
    # then return the tags in descending order
    name_and_tag = [(t.name, t) for t in tags if t.name]
    name_and_tag.sort(key=lambda x: [int(y) for y in x[0].split('.')])
    name_and_tag.reverse()
    for (_, tag) in name_and_tag:
        yield tag


def generate_package_headline(pkg_name):
    headline = 'Changelog for package %s' % pkg_name
    section_marker = '^' * len(headline)
    return '%s\n%s\n%s\n' % (section_marker, headline, section_marker)


def generate_version_block(version, timestamp, log_entries, vcs_client=None, skip_contributors=False):
    data = generate_version_headline(version, timestamp)
    data += generate_version_content(log_entries, vcs_client=vcs_client, skip_contributors=skip_contributors)
    return data


def generate_version_headline(version, timestamp):
    headline = get_version_headline(version, timestamp)
    return '%s\n%s\n' % (headline, '-' * len(headline))


def get_version_headline(version, timestamp):
    if not version:
        return FORTHCOMING_LABEL
    headline = version
    if timestamp:
        headline += ' (%s)' % timestamp
    return headline


def generate_version_content(log_entries, vcs_client=None, skip_contributors=False):
    data = ''
    all_authors = set()
    for entry in log_entries:
        msg = entry.msg
        lines = msg.splitlines()
        lines = [l.strip() for l in lines]
        lines = [l for l in lines if l]
        lines = [escape_trailing_underscores(l) for l in lines]
        data += '* %s\n' % (replace_repository_references(lines[0], vcs_client=vcs_client) if lines else '')
        for line in lines[1:]:
            data += '  %s\n' % replace_repository_references(line, vcs_client=vcs_client)
        all_authors.add(entry.author)
    if all_authors and not skip_contributors:
        data += '* Contributors: %s\n' % ', '.join(sorted(all_authors))
    return data


def escape_trailing_underscores(line):
    if line.endswith('_'):
        line = line[:-1] + '\_'
    # match words ending with an underscore which are not followed by another word
    # and insert a backslash before the underscore to escape it
    line = re.sub(r'(\w+)_([^\w])', '\\1\\_\\2', line)
    return line


def replace_repository_references(line, vcs_client=None):
    if vcs_client:
        line = vcs_client.replace_repository_references(line)
    return line
