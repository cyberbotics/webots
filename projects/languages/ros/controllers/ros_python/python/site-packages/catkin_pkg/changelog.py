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
Processes ROS changelogs so that they can be used in binary packaging.

The Changelog format is described in REP-0132:

http://ros.org/reps/rep-0132.html
"""

from __future__ import print_function
from __future__ import unicode_literals

import logging
import os
import re
import sys

import dateutil.parser
import docutils
import docutils.core
import pkg_resources

_py3 = sys.version_info[0] >= 3

try:
    _unicode = unicode
except NameError:
    _unicode = str

__author__ = 'William Woodall'
__email__ = 'william@osrfoundation.org'
__maintainer__ = 'William Woodall'

log = logging.getLogger('changelog')

CHANGELOG_FILENAME = 'CHANGELOG.rst'

example_rst = """\
^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package foo
^^^^^^^^^^^^^^^^^^^^^^^^^

0.1
===
Free form text about this minor release.

0.1.27 (forthcoming)
--------------------
* Great new feature

0.1.26 (2012-12-26)
-------------------
* Utilizes caching to improve query performance (fix https://github.com/ros/ros_comm/pull/2)
* Simplified API calls based on (https://github.com/ros/robot_model):

  * Note that these changes are based on REP 192
  * Also they fix a problem related to initialization

* Fixed synchronization issue on startup

.. not mentioning secret feature on purpose

0.1.25 (2012-11-25)
-------------------

- Added thread safety
- Replaced custom XML parser with `TinyXML <http://www.grinninglizard.com/tinyxml/>`_.
- Fixed regression introduced in 0.1.22
- New syntax for foo::

    foo('bar')

- Added a safety check for XML parsing

----

The library should now compile under ``Win32``

0.1.0 (2012-10-01)
------------------

*First* public **stable** release

0.0
===

0.0.1 (2012-01-31)
------------------

1. Initial release
2. Initial bugs
"""


def bullet_list_class_from_docutils(bullet_list, bullet_type=None):
    """
    Process elements of bullet list into an encapsulating class.

    :param bullet_list: ``docutils.nodes.bullet_list`` list to be processed
    :param bullet_type: ``str`` either 'bullet' or 'enumerated'
    :returns: ``BulletList`` object representing a docutils bullet_list
    """
    content = BulletList(bullet_type=bullet_type)
    for child in bullet_list.children:
        if isinstance(child, docutils.nodes.list_item):
            content.bullets.append(mixed_text_from_docutils(child))
        else:
            log.debug("Skipped bullet_list child: '{0}'".format(child))
    return content


def mixed_text_from_docutils(node):
    """
    Take most Text-ish docutils objects and converts them to MixedText.

    :param node: ``docutils.nodes.{paragraph, list_item, ...}`` text-ish
    :returns: ``MixedText`` representing the given docutils object
    """
    content = MixedText()
    for child in node.children:
        if isinstance(child, docutils.nodes.paragraph):
            content.texts.extend(mixed_text_from_docutils(child).texts)
        elif isinstance(child, docutils.nodes.Text):
            content.texts.append(child.astext())
        elif isinstance(child, docutils.nodes.reference):
            content.texts.append(reference_from_docutils(child))
        elif isinstance(child, docutils.nodes.emphasis):
            content.texts.append('*{0}*'.format(child.astext()))
        elif isinstance(child, docutils.nodes.strong):
            content.texts.append('**{0}**'.format(child.astext()))
        elif isinstance(child, docutils.nodes.literal):
            content.texts.append('``{0}``'.format(child.astext()))
        elif isinstance(child, docutils.nodes.literal_block):
            content.texts.append('\n\n    ' + child.astext() + '\n')
        elif isinstance(child, docutils.nodes.target):
            pass
        elif isinstance(child, docutils.nodes.system_message):
            log.debug('Skipping system_message: {0}'.format(child))
        elif isinstance(child, docutils.nodes.bullet_list):
            content.texts.append(bullet_list_class_from_docutils(child))
        else:
            try:
                # Try to add it as plain text
                log.debug("Trying to add {0}'s child of type {1}: '{2}'"
                          .format(type(node), type(child), child))
                content.texts.append(child.astext())
            except AttributeError:
                log.debug("Ignored {0} child of type {1}: '{2}'"
                          .format(type(node), type(child), child))
    return content


def get_changelog_from_path(path, package_name=None):
    """
    Changelog factory, which reads a changelog file into a class.

    :param path: ``str`` the path of the changelog including or excluding the filename CHANGELOG.rst
    :param package_name: ``str`` the package name
    :returns: ``Changelog`` changelog class or None if file was not readable
    """
    changelog = Changelog(package_name)
    if os.path.isdir(path):
        path = os.path.join(path, CHANGELOG_FILENAME)
    try:
        with open(path, 'rb') as f:
            populate_changelog_from_rst(changelog, f.read().decode('utf-8'))
    except IOError:
        return None
    return changelog


def populate_changelog_from_rst(changelog, rst):
    """
    Changelog factory, which converts the raw ReST into a class.

    :param changelog: ``Changelog`` changelog to be populated
    :param rst: ``str`` raw ReST changelog
    :returns: ``Changelog`` changelog that was populated
    """
    document = docutils.core.publish_doctree(rst)
    processes_changelog_children(changelog, document.children)
    changelog.rst = rst
    return changelog


def processes_changelog_children(changelog, children):
    """
    Process docutils children into a REP-0132 changelog instance.

    Recurse into sections, check (sub-)titles if they are valid versions.

    :param changelog: ``Changelog`` changelog to be populated
    :param section: ``docutils.nodes.section`` section to be processed
    :returns: ``Changelog`` changelog that was populated
    """
    for i, child in enumerate(children):
        if isinstance(child, docutils.nodes.section):
            processes_changelog_children(changelog, child.children)
        elif isinstance(child, docutils.nodes.title) or isinstance(child, docutils.nodes.subtitle):
            version, date = None, None
            # See if the title has a text element in it
            if len(child.children) > 0 and isinstance(child.children[0], docutils.nodes.Text):
                # Extract version and date from (sub-)title
                title_text = child.children[0].rawsource
                try:
                    version, date = version_and_date_from_title(title_text)
                except InvalidSectionTitle:
                    # Catch invalid section titles
                    log.debug("Ignored non-compliant title: '{0}'".format(title_text))
                    continue
            valid_section = None not in (version, date)
            if valid_section:
                contents = []
                # For each remaining sibling
                for child in children[i + 1:]:
                    # Skip sections (nesting of valid sections not allowed)
                    if isinstance(child, docutils.nodes.section):
                        log.debug("Ignored section child: '{0}'".format(child))
                        continue
                    # Skip title
                    if isinstance(child, docutils.nodes.title):
                        continue
                    # Skip comments
                    if isinstance(child, docutils.nodes.comment):
                        log.debug("Ignored section child: '{0}'".format(child))
                        continue
                    # Process other elements into the contents
                    if isinstance(child, docutils.nodes.bullet_list):
                        contents.append(bullet_list_class_from_docutils(child))
                    elif isinstance(child, docutils.nodes.enumerated_list):
                        contents.append(bullet_list_class_from_docutils(child, bullet_type='enumerated'))
                    elif isinstance(child, docutils.nodes.transition):
                        contents.append(Transition())
                    elif isinstance(child, docutils.nodes.paragraph):
                        contents.append(mixed_text_from_docutils(child))
                    else:
                        log.debug("Skipped section child: '{0}'".format(child))
                changelog.add_version_section(version, date, contents)
                break
            else:
                log.debug("Ignored non-compliant title: '{0}'".format(child))


def reference_from_docutils(reference):
    """
    Turn a reference element into a ``Reference``.

    :param reference: ``docutils.nodes.reference`` reference element
    :returns: ``Reference`` simpler object representing the reference
    """
    name, refuri = None, None
    for pair in reference.attlist():
        if pair[0] == 'name':
            name = pair[1]
        if pair[0] == 'refuri':
            refuri = pair[1]
    return Reference(name, refuri)


def version_and_date_from_title(title):
    """
    Split a section title into version and date if possible.

    :param title: ``str`` raw section title to be processed
    :returns: ``(str, datetime.datetime)``
    :raises: ``InvalidSectionTitle`` for non REP-0132 section titles
    """
    match = re.search(r'^([0-9]+\.[0-9]+\.[0-9]+)[ ]\((.+)\)$', title)
    if match is None:
        raise InvalidSectionTitle(title)
    version, date_str = match.groups()
    try:
        date = dateutil.parser.parse(date_str)
    except (ValueError, TypeError) as e:
        # Catch invalid dates
        log.debug("Error parsing date ({0}): '{1}'".format(date_str, e))
        raise InvalidSectionTitle(title)
    return version, date


class BulletList(object):
    """Represent a bulleted list of text."""

    def __init__(self, bullets=None, bullet_type=None):
        """
        Initialize BulletList.

        :param bullets: ``list(MixedText)`` list of text bullets
        :param bullet_type: ``str`` either 'bullet' or 'enumerated'
        """
        bullet_type = 'bullet' if bullet_type is None else bullet_type
        if bullet_type not in ['bullet', 'enumerated']:
            raise RuntimeError("Invalid bullet type: '{0}'".format(bullet_type))
        self.bullets = bullets or []
        self.bullet_type = bullet_type

    def __iter__(self):
        for bullet in self.bullets:
            yield bullet

    def __str__(self):
        value = self.__unicode__()
        if not _py3:
            value = value.encode('ascii', 'replace')
        return value

    def __unicode__(self):
        return self.as_txt()

    def as_rst(self):
        return self.as_txt(indent='', use_hyphen_bullet=True)

    def as_txt(self, indent='', use_hyphen_bullet=False):
        bullet = '*' if self.bullet_type == 'bullet' else '#'
        if use_hyphen_bullet and bullet == '*':
            bullet = '-'
        b = self.bullet_generator(bullet)
        i = indent
        n = '\n' + i + '  '
        lines = [i + next(b) + _unicode(l).replace('\n', n) for l in self]
        return '\n'.join(lines)

    def bullet_generator(self, bullet):
        if '#' == bullet:
            bullets = [str(i) + '. ' for i in range(1, len(self.bullets) + 1)]
        else:
            bullets = [bullet + ' '] * len(self.bullets)
        for b in bullets:
            yield b


class Changelog(object):
    """Represents a REP-0132 changelog."""

    def __init__(self, package_name=None):
        self.__package_name = package_name
        self.__versions = []
        self.__parsed_versions = []
        self.__dates = {}
        self.__content = {}
        self.__rst = ''

    def __str__(self):
        value = self.__unicode__()
        if not _py3:
            value = value.encode('ascii', 'replace')
        return value

    def __unicode__(self):
        msg = []
        if self.__package_name:
            msg.append("Changelog for package '{0}'".format(self.package_name))
        for version, date, content in self.foreach_version(reverse=True):
            msg.append('  ' + version + ' ({0}):'.format(date))
            for item in content:
                msg.extend(['    ' + i for i in _unicode(item).splitlines()])
        return '\n'.join(msg)

    @property
    def package_name(self):
        return self.__package_name

    @package_name.setter
    def package_name(self, package_name):
        self.__package_name = package_name

    @property
    def rst(self):
        return self.__rst

    @rst.setter
    def rst(self, rst):
        self.__rst = rst

    def add_version_section(self, version, date, contents):
        """
        Add a version section.

        :param version: ``str`` version as a string
        :param date: ``datetime.datetime`` version date
        :param contents: ``list(list([str|Reference]))``` contents as a list
          of lists which contain a combination of ``str`` and
          ``Reference`` objects
        :returns: None
        """
        if version in self.__versions:
            raise DuplicateVersionsException(version)
        self.__parsed_versions.append(pkg_resources.parse_version(version))
        self.__parsed_versions = sorted(self.__parsed_versions)
        # Cannot go parsed -> str, so sorting must be done by comparison
        new_versions = [None] * len(self.__parsed_versions)
        for v in self.__versions + [version]:
            parsed_v = pkg_resources.parse_version(v)
            index = self.__parsed_versions.index(parsed_v)
            if index == -1:
                raise RuntimeError('Inconsistent internal version storage state')
            new_versions[index] = v
        self.__versions = new_versions
        self.__dates[version] = date
        self.__content[version] = contents

    def foreach_version(self, reverse=False):
        """
        Create a generator for iterating over the versions, dates and content.

        Versions are stored and iterated in order.

        :param reverse: ``bool`` if True then the iteration is reversed
        :returns: ``generator`` for iterating over versions, dates and content
        """
        for version in reversed(self.__versions) if reverse else self.__versions:
            yield version, self.__dates[version], self.__content[version]

    def get_date_of_version(self, version):
        """Return date of a given version as a ``datetime.datetime``."""
        if version not in self.__versions:
            raise KeyError("No date for version '{0}'".format(version))
        return self.__dates[version]

    def get_content_of_version(self, version):
        """
        Return changelog content for a given version.

        :param version: ``str`` version
        :returns: ``list(list([str|Reference]))`` content expanded
        """
        if version not in self.__versions:
            raise KeyError("No content for version '{0}'".format(version))
        return self.__content[version]


class DuplicateVersionsException(Exception):
    """Raised when more than one section per version is given."""

    def __init__(self, version):
        self.version = version
        Exception.__init__(self, "Version '{0}' is specified twice".format(version))


class InvalidSectionTitle(Exception):
    """raised on non REP-0132 section titles."""

    def __init__(self, title):
        self.title = title
        msg = "Section title does not conform to REP-0132: '{0}'".format(title)
        Exception.__init__(self, msg)


class MixedText(object):
    """Represents text mixed with references and nested bullets."""

    def __init__(self, texts=[]):
        self.texts = list(texts)

    def __iter__(self):
        for text in self.texts:
            yield text

    def __str__(self):
        value = self.__unicode__()
        if not _py3:
            value = value.encode('ascii', 'replace')
        return value

    def __unicode__(self):
        return self.to_txt()

    def to_txt(self, bullet_indent='  '):
        lines = []
        for t in self:
            if isinstance(t, BulletList):
                bullets = [bullet_indent + x for x in _unicode(t).splitlines()]
                bullets = ['', ''] + bullets + ['']
                lines.extend('\n'.join(bullets))
            else:
                lines.append(_unicode(t))
        return ''.join(lines)


class Reference(object):
    """Represents a piece of text with an associated link."""

    def __init__(self, text, link):
        self.text = text
        self.link = link

    def __str__(self):
        value = self.__unicode__()
        if not _py3:
            value = value.encode('ascii', 'replace')
        return value

    def __unicode__(self):
        return self.as_txt()

    def as_rst(self):
        """Self as rst (unicode)."""
        if self.text is None:
            return _unicode(self.link)
        return '`{0} <{1}>`_'.format(self.text, self.link)

    def as_txt(self):
        """Self formatted for plain text (unicode)."""
        if self.text is None:
            return _unicode(self.link)
        return '{0} <{1}>'.format(self.text, self.link)


class Transition(object):
    """Represents a trasition element from ReST."""

    def __str__(self):
        value = self.__unicode__()
        if not _py3:
            value = value.encode('ascii', 'replace')
        return value

    def __unicode__(self):
        return '-' * 20

    def __iter__(self):
        yield self.unicode()


def __test():
    package_name = 'foo'
    changelog = Changelog(package_name)
    print(populate_changelog_from_rst(changelog, example_rst))


if __name__ == '__main__':
    logging.basicConfig()
    log.setLevel(logging.DEBUG)
    __test()
