# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
Library for processing 'manifest' files, i.e. manifest.xml and
stack.xml.
"""

import os
import sys
import xml.dom.minidom as dom

from .common import MANIFEST_FILE, PACKAGE_FILE, STACK_FILE

# stack.xml and manifest.xml have the same internal tags right now
REQUIRED = ['license']
ALLOWXHTML = ['description']
OPTIONAL = ['author', 'logo', 'url', 'brief', 'description', 'status',
            'notes', 'depend', 'rosdep', 'export', 'review',
            'versioncontrol', 'platform', 'version', 'rosbuild2',
            'catkin']
VALID = REQUIRED + OPTIONAL


class InvalidManifest(Exception):
    pass


def _get_nodes_by_name(n, name):
    return [t for t in n.childNodes if t.nodeType == t.ELEMENT_NODE and t.tagName == name]


def _check_optional(name, allowXHTML=False, merge_multiple=False):
    """
    Validator for optional elements.

    :raise: :exc:`InvalidManifest` If validation fails
    """
    def check(n, filename):
        n = _get_nodes_by_name(n, name)
        if len(n) > 1 and not merge_multiple:
            raise InvalidManifest("Invalid manifest file [%s]: must have a single '%s' element" % (filename, name))
        if n:
            values = []
            for child in n:
                if allowXHTML:
                    values.append(''.join([x.toxml() for x in child.childNodes]))
                else:
                    values.append(_get_text(child.childNodes).strip())
            return ', '.join(values)
    return check


def _check_required(name, allowXHTML=False, merge_multiple=False):
    """
    Validator for required elements.

    :raise: :exc:`InvalidManifest` If validation fails
    """
    def check(n, filename):
        n = _get_nodes_by_name(n, name)
        if not n:
            return ''
        if len(n) != 1 and not merge_multiple:
            raise InvalidManifest("Invalid manifest file: must have only one '%s' element" % name)
        values = []
        for child in n:
            if allowXHTML:
                values.append(''.join([x.toxml() for x in child.childNodes]))
            else:
                values.append(_get_text(child.childNodes).strip())
        return ', '.join(values)
    return check


def _check_platform(n, filename):
    """
    Validator for manifest platform.
    :raise: :exc:`InvalidManifest` If validation fails
    """
    platforms = _get_nodes_by_name(n, 'platform')
    try:
        vals = [(p.attributes['os'].value, p.attributes['version'].value, p.getAttribute('notes')) for p in platforms]
    except KeyError as e:
        raise InvalidManifest("<platform> tag is missing required '%s' attribute" % str(e))
    return [Platform(*v) for v in vals]


def _check_depends(type_, n, filename):
    """
    Validator for manifest depends.
    :raise: :exc:`InvalidManifest` If validation fails
    """
    nodes = _get_nodes_by_name(n, 'depend')
    # TDS 20110419:  this is a hack.
    # rosbuild2 has a <depend thirdparty="depname"/> tag,
    # which is confusing this subroutine with
    # KeyError: 'package'
    # for now, explicitly don't consider thirdparty depends
    depends = [e.attributes for e in nodes if 'thirdparty' not in e.attributes.keys()]
    try:
        depend_names = [d[type_].value for d in depends]
    except KeyError:
        raise InvalidManifest("Invalid manifest file [%s]: depends is missing '%s' attribute" % (filename, type_))

    return [Depend(name, type_) for name in depend_names]


def _check_rosdeps(n, filename):
    """
    Validator for stack rosdeps.

    :raises: :exc:`InvalidManifest` If validation fails
    """
    try:
        nodes = _get_nodes_by_name(n, 'rosdep')
        rosdeps = [e.attributes for e in nodes]
        names = [d['name'].value for d in rosdeps]
        return [RosDep(n) for n in names]
    except KeyError:
        raise InvalidManifest("invalid rosdep tag in [%s]" % (filename))


def _attrs(node):
    attrs = {}
    for k in node.attributes.keys():
        attrs[k] = node.attributes.get(k).value
    return attrs


def _check_exports(n, filename):
    ret_val = []
    for e in _get_nodes_by_name(n, 'export'):
        elements = [c for c in e.childNodes if c.nodeType == c.ELEMENT_NODE]
        ret_val.extend([Export(t.tagName, _attrs(t), _get_text(t.childNodes)) for t in elements])
    return ret_val


def _check(name, merge_multiple=False):
    """
    Generic validator for text-based tags.
    """
    if name in REQUIRED:
        if name in ALLOWXHTML:
            return _check_required(name, True, merge_multiple)
        return _check_required(name, merge_multiple=merge_multiple)
    elif name in OPTIONAL:
        if name in ALLOWXHTML:
            return _check_optional(name, True, merge_multiple)
        return _check_optional(name, merge_multiple=merge_multiple)


class Export(object):
    """
    Manifest 'export' tag
    """

    def __init__(self, tag, attrs, str):
        """
        Create new export instance.
        :param tag: name of the XML tag
        @type  tag: str
        :param attrs: dictionary of XML attributes for this export tag
        @type  attrs: dict
        :param str: string value contained by tag, if any
        @type  str: str
        """
        self.tag = tag
        self.attrs = attrs
        self.str = str

    def get(self, attr):
        """
        :returns: value of attribute or ``None`` if attribute not set, ``str``
        """
        return self.attrs.get(attr, None)


class Platform(object):
    """
    Manifest 'platform' tag
    """
    __slots__ = ['os', 'version', 'notes']

    def __init__(self, os_, version, notes=None):
        """
        Create new depend instance.
        :param os_: OS name. must be non-empty, ``str``
        :param version: OS version. must be non-empty, ``str``
        :param notes: (optional) notes about platform support, ``str``
        """
        if not os_:
            raise ValueError("bad 'os' attribute")
        if not version:
            raise ValueError("bad 'version' attribute")
        self.os = os_
        self.version = version
        self.notes = notes

    def __str__(self):
        return "%s %s" % (self.os, self.version)

    def __repr__(self):
        return "%s %s" % (self.os, self.version)

    def __eq__(self, obj):
        """
        Override equality test. notes *are* considered in the equality test.
        """
        if not isinstance(obj, Platform):
            return False
        return self.os == obj.os and self.version == obj.version and self.notes == obj.notes

    def __hash__(self):
        """
        :returns: an integer, which must be the same for two equal instances.

        Since __eq__ is defined, Python 3 requires that this class also provide a __hash__ method.
        """
        return hash(self.os) ^ hash(self.version) ^ hash(self.notes)


class Depend(object):
    """
    Manifest 'depend' tag
    """
    __slots__ = ['name', 'type']

    def __init__(self, name, type_):
        """
        Create new depend instance.
        :param name: dependency name (e.g. package/stack). Must be non-empty
        @type  name: str
        :param type_: dependency type, e.g. 'package', 'stack'.  Must be non-empty.
        @type  type_: str

        @raise ValueError: if parameters are invalid
        """
        if not name:
            raise ValueError("bad '%s' attribute" % (type_))
        if not type_:
            raise ValueError("type_ must be specified")
        self.name = name
        self.type = type_

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

    def __eq__(self, obj):
        if not isinstance(obj, Depend):
            return False
        return self.name == obj.name and self.type == obj.type

    def __hash__(self):
        """
        :returns: an integer, which must be the same for two equal instances.

        Since __eq__ is defined, Python 3 requires that this class also provide a __hash__ method.
        """
        return hash(self.name) ^ hash(self.type)


class RosDep(object):
    """
    Manifest 'rosdep' tag
    """
    __slots__ = ['name']

    def __init__(self, name):
        """
        Create new rosdep instance.

        :param name: dependency name. Must be non-empty. ``str``
        """
        if not name:
            raise ValueError("bad 'name' attribute")
        self.name = name


class Manifest(object):
    """
    Object representation of a ROS manifest file (``manifest.xml`` and ``stack.xml``)
    """
    __slots__ = [
        'description', 'brief',
        'author', 'license', 'license_url', 'url',
        'depends', 'rosdeps', 'platforms',
        'exports', 'version',
        'status', 'notes',
        'unknown_tags', 'type', 'filename',
        'is_catkin']

    def __init__(self, type_='package', filename=None, is_catkin=False):
        """
        :param type: `'package'` or `'stack'`
        :param filename: location of manifest file.  Necessary if
          converting ``${prefix}`` in ``<export>`` values, ``str``.
        """
        self.description = self.brief = self.author = \
            self.license = self.license_url = \
            self.url = self.status = \
            self.version = self.notes = ''
        self.depends = []
        self.rosdeps = []
        self.exports = []
        self.platforms = []
        self.is_catkin = is_catkin

        self.type = type_
        self.filename = filename

        # store unrecognized tags during parsing
        self.unknown_tags = []

    def get_export(self, tag, attr, convert=True):
        """
        :param tag: Name of XML tag to retrieve, ``str``
        :param attr: Name of XML attribute to retrieve from tag, ``str``
        :param convert: If ``True``, interpret variables (e.g. ``${prefix}``) export values.
        :returns: exports that match the specified tag and attribute, e.g. 'python', 'path'. ``[str]``
        """
        vals = [e.get(attr) for e in self.exports if e.tag == tag if e.get(attr) is not None]
        if convert:
            if not self.filename:
                raise ValueError("cannot convert export values when filename for Manifest is not set")
            prefix = os.path.dirname(self.filename)
            vals = [v.replace('${prefix}', prefix) for v in vals]
        return vals


def _get_text(nodes):
    """
    DOM utility routine for getting contents of text nodes
    """
    return "".join([n.data for n in nodes if n.nodeType == n.TEXT_NODE])


_static_rosdep_view = None


def parse_manifest_file(dirpath, manifest_name, rospack=None):
    """
    Parse manifest file (package, stack).  Type will be inferred from manifest_name.

    :param dirpath: directory of manifest file, ``str``
    :param manifest_name: ``MANIFEST_FILE`` or ``STACK_FILE``, ``str``
    :param rospack: a RosPack instance to identify local packages as ROS packages

    :returns: return :class:`Manifest` instance, populated with parsed fields
    :raises: :exc:`InvalidManifest`
    :raises: :exc:`IOError`
    """
    filename = os.path.join(dirpath, manifest_name)
    if not os.path.isfile(filename):
        # hack for backward compatibility
        package_filename = os.path.join(dirpath, PACKAGE_FILE)
        if not os.path.isfile(package_filename):
            raise IOError("Invalid/non-existent manifest file: %s" % filename)
        manifest = Manifest(filename=filename, is_catkin=True)

        # extract all information from package.xml
        from catkin_pkg.package import parse_package
        p = parse_package(package_filename)
        # put these into manifest
        manifest.description = p.description
        manifest.author = ', '.join([('Maintainer: %s' % str(m)) for m in p.maintainers] + [str(a) for a in p.authors])
        manifest.license = ', '.join(p.licenses)
        if p.urls:
            manifest.url = str(p.urls[0])
        manifest.version = p.version
        for export in p.exports:
            manifest.exports.append(Export(export.tagname, export.attributes, export.content))

        # split ros and system dependencies (using rosdep)
        try:
            from rosdep2.rospack import init_rospack_interface, is_ros_package, is_system_dependency, is_view_empty
            global _static_rosdep_view
            # initialize rosdep view once
            if _static_rosdep_view is None:
                _static_rosdep_view = init_rospack_interface()
                if is_view_empty(_static_rosdep_view):
                    sys.stderr.write("the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'\n")
                    _static_rosdep_view = False
            if _static_rosdep_view:
                depends = set([])
                rosdeps = set([])
                for d in (p.buildtool_depends + p.build_depends + p.run_depends + p.test_depends):
                    if (rospack and d.name in rospack.list()) or is_ros_package(_static_rosdep_view, d.name):
                        depends.add(d.name)
                    if is_system_dependency(_static_rosdep_view, d.name):
                        rosdeps.add(d.name)
                for name in depends:
                    manifest.depends.append(Depend(name, 'package'))
                for name in rosdeps:
                    manifest.rosdeps.append(RosDep(name))
        except ImportError:
            pass

        return manifest

    with open(filename, 'rb') as f:
        data = f.read()
    if sys.version_info[0] >= 3:
        data = data.decode('utf-8')
    return parse_manifest(manifest_name, data, filename)


def parse_manifest(manifest_name, string, filename='string'):
    """
    Parse manifest string contents.

    :param manifest_name: ``MANIFEST_FILE`` or ``STACK_FILE``, ``str``
    :param string: manifest.xml contents, ``str``
    :param filename: full file path for debugging, ``str``
    :returns: return parsed :class:`Manifest`
    """
    if manifest_name == MANIFEST_FILE:
        type_ = 'package'
    elif manifest_name == STACK_FILE:
        type_ = 'stack'

    try:
        d = dom.parseString(string)
    except Exception as e:
        raise InvalidManifest("[%s] invalid XML: %s" % (filename, e))

    m = Manifest(type_, filename)
    p = _get_nodes_by_name(d, type_)
    if len(p) != 1:
        raise InvalidManifest("manifest [%s] must have a single '%s' element" % (filename, type_))
    p = p[0]
    m.description = _check('description')(p, filename)
    m.brief = ''
    try:
        tag = _get_nodes_by_name(p, 'description')[0]
        m.brief = tag.getAttribute('brief') or ''
    except:
        # means that 'description' tag is missing
        pass

    m.depends = _check_depends(type_, p, filename)
    m.rosdeps = _check_rosdeps(p, filename)
    m.platforms = _check_platform(p, filename)
    m.exports = _check_exports(p, filename)
    m.license = _check('license')(p, filename)
    m.license_url = ''
    try:
        tag = _get_nodes_by_name(p, 'license')[0]
        m.license_url = tag.getAttribute('url') or ''
    except:
        pass  # manifest is missing required 'license' tag

    m.status = 'unreviewed'
    try:
        tag = _get_nodes_by_name(p, 'review')[0]
        m.status = tag.getAttribute('status') or ''
    except:
        pass  # manifest is missing optional 'review status' tag

    m.notes = ''
    try:
        tag = _get_nodes_by_name(p, 'review')[0]
        m.notes = tag.getAttribute('notes') or ''
    except:
        pass  # manifest is missing optional 'review notes' tag

    m.author = _check('author', True)(p, filename)
    m.url = _check('url')(p, filename)
    m.version = _check('version')(p, filename)

    # do some validation on what we just parsed
    if type_ == 'stack':
        if m.exports:
            raise InvalidManifest("stack manifests are not allowed to have exports")
        if m.rosdeps:
            raise InvalidManifest("stack manifests are not allowed to have rosdeps")

    m.is_catkin = bool(_get_nodes_by_name(p, 'catkin')) or bool(_get_nodes_by_name(p, 'name'))

    # store unrecognized tags
    m.unknown_tags = [e for e in p.childNodes if e.nodeType == e.ELEMENT_NODE and e.tagName not in VALID]
    return m
