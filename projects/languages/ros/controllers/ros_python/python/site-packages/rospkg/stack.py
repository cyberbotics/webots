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

"""
Library for processing stack.xml created post-catkin
"""

import collections
import os
import xml.dom.minidom as dom

# as defined on http://ros.org/doc/fuerte/api/catkin/html/stack_xml.html
REQUIRED = ['name', 'version', 'description', 'author', 'maintainer', 'license', 'copyright']
ALLOWXHTML = ['description']
OPTIONAL = ['description_brief', 'version_abi', 'url', 'review_notes', 'review_status', 'build_depends', 'depends', 'build_type', 'message_generator', 'review']

LISTED_ATTRIBUTES = {'Author': ['name', 'email'], 'Maintainer': ['name', 'email'], 'Depend': ['name', 'version']}

VALID = REQUIRED + OPTIONAL


class InvalidStack(Exception):
    pass


def _get_nodes_by_name(n, name):
    return [t for t in n.childNodes if t.nodeType == t.ELEMENT_NODE and t.tagName == name]


def _check_optional(name, allowXHTML=False):
    """
    Validator for optional elements.

    :raise: :exc:`InvalidStack` If validation fails
    """
    def check(n, filename):
        n = _get_nodes_by_name(n, name)
        if len(n) > 1:
            raise InvalidStack("Invalid stack.xml file [%s]: must have at most one '%s' element" % (filename, name))
        if n:
            if allowXHTML:
                return ''.join([x.toxml() for x in n[0].childNodes])
            return _get_text(n[0].childNodes).strip()
    return check


def _check_required(name, allowXHTML=False):
    """
    Validator for required elements.

    :raise: :exc:`InvalidStack` If validation fails
    """
    def check(n, filename):
        n = _get_nodes_by_name(n, name)
        if len(n) != 1:
            raise InvalidStack("Invalid stack.xml file [%s]: must have exactly one '%s' element" % (filename, name))
        if allowXHTML:
            return ''.join([x.toxml() for x in n[0].childNodes])
        return _get_text(n[0].childNodes).strip()
    return check


def _check_depends(n, key, filename):
    """
    Validator for stack.xml depends.
    :raise: :exc:`InvalidStack` If validation fails
    """
    nodes = _get_nodes_by_name(n, key)
    return set([_get_text(n.childNodes).strip() for n in nodes])


def _build_listed_attributes(n, key, object_type):
    """
    Validator for stack.xml depends.
    :raise: :exc:`InvalidStack` If validation fails
    """
    members = set()
    for node in _get_nodes_by_name(n, key):
        # The first field is always supposed to be the value
        attribute_dict = {}
        for field in object_type._fields:
            try:
                attribute_dict[field] = node.getAttribute(field)
            except:
                pass
        attribute_dict[object_type._fields[0]] = _get_text(node.childNodes).strip()
        members.add(object_type(**attribute_dict))
    return members


def _attrs(node):
    attrs = {}
    for k in node.attributes.keys():
        attrs[k] = node.attributes.get(k).value
    return attrs


def _check(name):
    """
    Generic validator for text-based tags.
    """
    if name in REQUIRED:
        return _check_required(name, name in ALLOWXHTML)
    elif name in OPTIONAL:
        return _check_optional(name, name in ALLOWXHTML)


class Stack(object):
    """
    Object representation of a ROS ``stack.xml`` file
    """
    __slots__ = [
        'name', 'version', 'description', 'authors', 'maintainers', 'license', 'copyright',
        'description_brief', 'version_abi', 'url', 'review_notes', 'review_status',
        'build_depends', 'depends', 'build_type', 'build_type_file', 'message_generator',
        'unknown_tags']

    def __init__(self, filename=None):
        """
        :param filename: location of stack.xml.  Necessary if
          converting ``${prefix}`` in ``<export>`` values, ``str``.
        """
        self.description = self.description_brief = self.name = \
            self.version = self.version_abi = \
            self.license = self.copyright = ''
        self.url = ''
        self.authors = []
        self.maintainers = []
        self.depends = []
        self.build_depends = []
        self.review_notes = self.review_status = ''
        self.build_type = 'cmake'
        self.build_type_file = ''
        self.message_generator = ''

        # store unrecognized tags during parsing
        self.unknown_tags = []


def _get_text(nodes):
    """
    DOM utility routine for getting contents of text nodes
    """
    return "".join([n.data for n in nodes if n.nodeType == n.TEXT_NODE])


def parse_stack_file(stack_path):
    """
    Parse stack file.

    :param stack_path: The path of the stack.xml file

    :returns: return :class:`Stack` instance, populated with parsed fields
    :raises: :exc:`InvalidStack`
    :raises: :exc:`IOError`
    """
    if not os.path.isfile(stack_path):
        raise IOError("Invalid/non-existent stack.xml file: %s" % (stack_path))

    with open(stack_path, 'r') as f:
        return parse_stack(f.read(), stack_path)


def parse_stack(string, filename):
    """
    Parse stack.xml string contents.

    :param string: stack.xml contents, ``str``
    :param filename: full file path for debugging, ``str``
    :returns: return parsed :class:`Stack`
    """
    # Create some classes to hold some members
    new_tuples = {}
    for key, members in LISTED_ATTRIBUTES.items():
        new_tuples[key] = collections.namedtuple(key, members)

    try:
        d = dom.parseString(string)
    except Exception as e:
        raise InvalidStack("[%s] invalid XML: %s" % (filename, e))

    s = Stack()
    p = _get_nodes_by_name(d, 'stack')
    if len(p) != 1:
        raise InvalidStack("stack.xml [%s] must have a single 'stack' element" % (filename))
    p = p[0]
    for attr in [
        'name', 'version', 'description',
        'license', 'copyright', 'url', 'build_type', 'message_generator'
    ]:
        val = _check(attr)(p, filename)
        if val:
            setattr(s, attr, val)

    try:
        tag = _get_nodes_by_name(p, 'description')[0]
        s.description_brief = tag.getAttribute('brief') or ''
    except:
        # means that 'description' tag is missing
        pass

    s.authors = _build_listed_attributes(p, 'author', new_tuples['Author'])
    s.maintainers = _build_listed_attributes(p, 'maintainer', new_tuples['Maintainer'])
    s.depends = _build_listed_attributes(p, 'depends', new_tuples['Depend'])
    s.build_depends = _build_listed_attributes(p, 'build_depends', new_tuples['Depend'])

    try:
        tag = _get_nodes_by_name(p, 'review')[0]
        s.review_status = tag.getAttribute('status') or ''
    except:
        pass  # stack.xml is missing optional 'review status' tag

    try:
        tag = _get_nodes_by_name(p, 'review')[0]
        s.review_notes = tag.getAttribute('notes') or ''
    except:
        pass  # stack.xml is missing optional 'review notes' tag

    try:
        tag = _get_nodes_by_name(p, 'build_type')[0]
        s.build_type_file = tag.getAttribute('file') or ''
    except:
        pass  # stack.xml is missing optional 'build_type file' tag

    # store unrecognized tags
    s.unknown_tags = [e.nodeName for e in p.childNodes if e.nodeType == e.ELEMENT_NODE and e.tagName not in VALID]
    if s.unknown_tags:
        raise InvalidStack("stack.xml [%s] must be cleaned up from %s" % (filename, str(s.unknown_tags)))
    return s
