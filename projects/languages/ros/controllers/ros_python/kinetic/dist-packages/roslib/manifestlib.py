#! /usr/bin/env python
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
#
# Revision $Id$
# $Author$

"""
Internal library for processing 'manifest' files, i.e. manifest.xml and stack.xml.
For external code apis, see L{roslib.manifest} and L{roslib.stack_manifest}.
"""

import sys
import os
import xml.dom
import xml.dom.minidom as dom

import roslib.exceptions

# stack.xml and manifest.xml have the same internal tags right now
REQUIRED = ['author', 'license']
ALLOWXHTML = ['description']
OPTIONAL = ['logo', 'url', 'brief', 'description', 'status',
            'notes', 'depend', 'rosdep', 'export', 'review',
            'versioncontrol', 'platform', 'version', 'rosbuild2',
            'catkin']
VALID = REQUIRED + OPTIONAL

class ManifestException(roslib.exceptions.ROSLibException): pass

def get_nodes_by_name(n, name):
    return [t for t in n.childNodes if t.nodeType == t.ELEMENT_NODE and t.tagName == name]
    
def check_optional(name, allowXHTML=False, merge_multiple=False):
    """
    Validator for optional elements.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        n = get_nodes_by_name(n, name)
        if len(n) > 1 and not merge_multiple:
            raise ManifestException("Invalid manifest file: must have a single '%s' element"%name)
        if n:
            values = []
            for child in n:
                if allowXHTML:
                    values.append(''.join([x.toxml() for x in child.childNodes]))
                else:
                    values.append(_get_text(child.childNodes).strip())
            return ', '.join(values)
    return check

def check_required(name, allowXHTML=False, merge_multiple=False):
    """
    Validator for required elements.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        n = get_nodes_by_name(n, name)
        if not n:
            #print >> sys.stderr, "Invalid manifest file[%s]: missing required '%s' element"%(filename, name)
            return ''
        if len(n) != 1 and not merge_multiple:
            raise ManifestException("Invalid manifest file: must have only one '%s' element"%name)
        values = []
        for child in n:
            if allowXHTML:
                values.append(''.join([x.toxml() for x in child.childNodes]))
            else:
                values.append(_get_text(child.childNodes).strip())
        return ', '.join(values)
    return check

def check_platform(name):
    """
    Validator for manifest platform.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        platforms = get_nodes_by_name(n, name)
        try:
            vals = [(p.attributes['os'].value, p.attributes['version'].value, p.getAttribute('notes')) for p in platforms]
        except KeyError as e:
            raise ManifestException("<platform> tag is missing required '%s' attribute"%str(e))
        return [Platform(*v) for v in vals]
    return check

def check_depends(name):
    """
    Validator for manifest depends.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        nodes = get_nodes_by_name(n, name)
        # TDS 20110419:  this is a hack.
        # rosbuild2 has a <depend thirdparty="depname"/> tag,
        # which is confusing this subroutine with 
        # KeyError: 'package'
        # for now, explicitly don't consider thirdparty depends
        depends = [e.attributes for e in nodes if 'thirdparty' not in e.attributes.keys()]
        try:
            packages = [d['package'].value for d in depends]
        except KeyError:
            raise ManifestException("Invalid manifest file: depends is missing 'package' attribute")

        return [Depend(p) for p in packages]
    return check

def check_stack_depends(name):
    """
    Validator for stack depends.
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        nodes = get_nodes_by_name(n, name)
        depends = [e.attributes for e in nodes]
        packages = [d['stack'].value for d in depends]
        return [StackDepend(p) for p in packages]
    return check

def check_rosdeps(name):
    """
    Validator for stack rosdeps.    
    @raise ManifestException: if validation fails
    """
    def check(n, filename):
        nodes = get_nodes_by_name(n, name)
        rosdeps = [e.attributes for e in nodes]
        names = [d['name'].value for d in rosdeps]
        return [ROSDep(n) for n in names]
    return check

def _attrs(node):
    attrs = {}
    for k in node.attributes.keys(): 
        attrs[k] = node.attributes.get(k).value
    return attrs
    
def check_exports(name):
    def check(n, filename):
        ret_val = []
        for e in get_nodes_by_name(n, name):
            elements = [c for c in e.childNodes if c.nodeType == c.ELEMENT_NODE]
            ret_val.extend([Export(t.tagName, _attrs(t), _get_text(t.childNodes)) for t in elements])
        return ret_val 
    return check

def check_versioncontrol(name):
    def check(n, filename):
        e = get_nodes_by_name(n, name)
        if not e:
            return None
        # note: 'url' isn't actually required, but as we only support type=svn it implicitly is for now
        return VersionControl(e[0].attributes['type'].value, e[0].attributes['url'].value)
    return check

def check(name, merge_multiple=False):
    if name == 'depend':
        return check_depends('depend')
    elif name == 'export':
        return check_exports('export')
    elif name == 'versioncontrol':
        return check_versioncontrol('versioncontrol')
    elif name == 'rosdep':
        return check_rosdeps('rosdep')
    elif name == 'platform':
        return check_platform('platform')
    elif name in REQUIRED:
        if name in ALLOWXHTML:
            return check_required(name, True, merge_multiple)
        return check_required(name, merge_multiple=merge_multiple)
    elif name in OPTIONAL:
        if name in ALLOWXHTML:
            return check_optional(name, True, merge_multiple)
        return check_optional(name, merge_multiple=merge_multiple)
    
class Export(object):
    """
    Manifest 'export' tag
    """
    
    def __init__(self, tag, attrs, str):
        """
        Create new export instance.
        @param tag: name of the XML tag
        @type  tag: str
        @param attrs: dictionary of XML attributes for this export tag
        @type  attrs: dict
        @param str: string value contained by tag, if any
        @type  str: str
        """
        self.tag = tag
        self.attrs = attrs
        self.str = str

    def get(self, attr):
        """
        @return: value of attribute or None if attribute not set
        @rtype:  str
        """
        return self.attrs.get(attr, None)
    def xml(self):
        """
        @return: export instance represented as manifest XML
        @rtype: str
        """        
        attrs = ' '.join([' %s="%s"'%(k,v) for k,v in self.attrs.items()]) #py3k
        if self.str:
            return '<%s%s>%s</%s>'%(self.tag, attrs, self.str, self.tag)
        else:
            return '<%s%s />'%(self.tag, attrs)
        
class Platform(object):
    """
    Manifest 'platform' tag
    """
    __slots__ = ['os', 'version', 'notes']

    def __init__(self, os, version, notes=None):
        """
        Create new depend instance.
        @param os: OS name. must be non-empty
        @type  os: str
        @param version: OS version. must be non-empty
        @type  version: str
        @param notes: (optional) notes about platform support
        @type  notes: str
        """
        if not os:
            raise ValueError("bad 'os' attribute")
        if not version:
            raise ValueError("bad 'version' attribute")
        self.os = os
        self.version = version
        self.notes = notes
        
    def __str__(self):
        return "%s %s"%(self.os, self.version)
    def __repr__(self):
        return "%s %s"%(self.os, self.version)
    def __eq__(self, obj):
        """
        Override equality test. notes *are* considered in the equality test.
        """
        if not isinstance(obj, Platform):
            return False
        return self.os == obj.os and self.version == obj.version and self.notes == obj.notes 
    def xml(self):
        """
        @return: instance represented as manifest XML
        @rtype: str
        """
        if self.notes is not None:
            return '<platform os="%s" version="%s" notes="%s"/>'%(self.os, self.version, self.notes)
        else:
            return '<platform os="%s" version="%s"/>'%(self.os, self.version)

class Depend(object):
    """
    Manifest 'depend' tag
    """
    __slots__ = ['package']

    def __init__(self, package):
        """
        Create new depend instance.
        @param package: package name. must be non-empty
        @type  package: str
        """
        if not package:
            raise ValueError("bad 'package' attribute")
        self.package = package
    def __str__(self):
        return self.package
    def __repr__(self):
        return self.package
    def __eq__(self, obj):
        if not isinstance(obj, Depend):
            return False
        return self.package == obj.package 
    def xml(self):
        """
        @return: depend instance represented as manifest XML
        @rtype: str
        """
        return '<depend package="%s" />'%self.package
        
class StackDepend(object):
    """
    Stack Manifest 'depend' tag
    """
    __slots__ = ['stack', 'annotation']

    def __init__(self, stack):
        """
        @param stack: stack name. must be non-empty
        @type  stack: str
        """
        if not stack:
            raise ValueError("bad 'stack' attribute")
        self.stack = stack
        self.annotation = None
        
    def __str__(self):
        return self.stack
    def __repr__(self):
        return self.stack
    def __eq__(self, obj):
        if not isinstance(obj, StackDepend):
            return False
        return self.stack == obj.stack 
    def xml(self):
        """
        @return: stack depend instance represented as stack manifest XML
        @rtype: str
        """
        if self.annotation:
            return '<depend stack="%s" /> <!-- %s -->'%(self.stack, self.annotation)
        else:
            return '<depend stack="%s" />'%self.stack            

class ROSDep(object):
    """
    Manifest 'rosdep' tag    
    """
    __slots__ = ['name',]

    def __init__(self, name):
        """
        Create new rosdep instance.
        @param name: dependency name. Must be non-empty.
        @type  name: str
        """
        if not name:
            raise ValueError("bad 'name' attribute")
        self.name = name
    def xml(self):
        """
        @return: rosdep instance represented as manifest XML
        @rtype: str
        """        
        return '<rosdep name="%s" />'%self.name

class VersionControl(object):
    """
    Manifest 'versioncontrol' tag
    """
    __slots__ = ['type', 'url']

    def __init__(self, type_, url):
        """
        @param type_: version control type (e.g. 'svn'). must be non empty
        @type  type_: str
        @param url: URL associated with version control. must be non empty
        @type  url: str
        """
        def is_string_type(obj):
            try:
                return isinstance(obj, basestring)
            except NameError:
                return isinstance(obj, str)

        if not type_ or not is_string_type(type_):
            raise ValueError("bad 'type' attribute")
        if not url is None and not is_string_type(url):
            raise ValueError("bad 'url' attribute")
        self.type = type_
        self.url = url
    def xml(self):
        """
        @return: versioncontrol instance represented as manifest XML
        @rtype: str
        """        
        if self.url:
            return '<versioncontrol type="%s" url="%s" />'%(self.type, self.url)
        else:
            return '<versioncontrol type="%s" />'%self.type
    
class _Manifest(object):
    """
    Object representation of a ROS manifest file
    """
    __slots__ = ['description', 'brief', \
                 'author', 'license', 'license_url', 'url', \
                 'depends', 'rosdeps','platforms',\
                 'logo', 'exports', 'version',\
                 'versioncontrol', 'status', 'notes',\
                 'unknown_tags',\
                 '_type']
    def __init__(self, _type='package'):
        self.description = self.brief = self.author = \
                           self.license = self.license_url = \
                           self.url = self.logo = self.status = \
                           self.version = self.notes = ''
        self.depends = []
        self.rosdeps = []
        self.exports = []
        self.platforms = []
        self._type = _type
        
        # store unrecognized tags during parsing
        self.unknown_tags = []
        
    def __str__(self):
        return self.xml()
    def get_export(self, tag, attr):
        """
        @return: exports that match the specified tag and attribute, e.g. 'python', 'path'
        @rtype: [L{Export}]
        """
        return [e.get(attr) for e in self.exports if e.tag == tag if e.get(attr) is not None]
    def xml(self):
        """
        @return: Manifest instance as ROS XML manifest
        @rtype: str
        """
        if not self.brief:
            desc = "  <description>%s</description>"%self.description
        else:
            desc = '  <description brief="%s">%s</description>'%(self.brief, self.description) 
        author  = "  <author>%s</author>"%self.author
        if self.license_url:
            license = '  <license url="%s">%s</license>'%(self.license_url, self.license)
        else:
            license = "  <license>%s</license>"%self.license
        versioncontrol = url = logo = exports = version = ""
        if self.url:
            url     = "  <url>%s</url>"%self.url
        if self.version:
            version = "  <version>%s</version>"%self.version
        if self.logo:
            logo    = "  <logo>%s</logo>"%self.logo
        depends = '\n'.join(["  %s"%d.xml() for d in self.depends])
        rosdeps = '\n'.join(["  %s"%rd.xml() for rd in self.rosdeps])
        platforms = '\n'.join(["  %s"%p.xml() for p in self.platforms])
        if self.exports:
            exports = '  <export>\n' + '\n'.join(["  %s"%e.xml() for e in self.exports]) + '  </export>'
        if self.versioncontrol:
            versioncontrol = "  %s"%self.versioncontrol.xml()
        if self.status or self.notes:
            review = '  <review status="%s" notes="%s" />'%(self.status, self.notes)


        fields = filter(lambda x: x,
                        [desc, author, license, review, url, logo, depends,
                         rosdeps, platforms, exports, versioncontrol, version])
        return "<%s>\n"%self._type + "\n".join(fields) + "\n</%s>"%self._type

def _get_text(nodes):
    """
    DOM utility routine for getting contents of text nodes
    """
    return "".join([n.data for n in nodes if n.nodeType == n.TEXT_NODE])

def parse_file(m, file):
    """
    Parse manifest file (package, stack)
    @param m: field to populate
    @type  m: L{_Manifest}
    @param file: manifest.xml file path
    @type  file: str
    @return: return m, populated with parsed fields
    @rtype: L{_Manifest}
    """
    if not file:
        raise ValueError("Missing manifest file argument")
    if not os.path.isfile(file):
        raise ValueError("Invalid/non-existent manifest file: %s"%file)
    with open(file, 'r') as f:
        text = f.read()
    try:
        return parse(m, text, file)
    except ManifestException as e:
        raise ManifestException("Invalid manifest file [%s]: %s"%(os.path.abspath(file), e))

def parse(m, string, filename='string'):
    """
    Parse manifest.xml string contents
    @param string: manifest.xml contents
    @type  string: str
    @param m: field to populate
    @type  m: L{_Manifest}
    @return: return m, populated with parsed fields
    @rtype: L{_Manifest}
    """
    try:
        d = dom.parseString(string)
    except Exception as e:
        raise ManifestException("invalid XML: %s"%e)
    
    p = get_nodes_by_name(d, m._type)
    if len(p) != 1:
        raise ManifestException("manifest must have a single '%s' element"%m._type)
    p = p[0]
    m.description = check('description')(p, filename)
    m.brief = ''
    try:
        tag = get_nodes_by_name(p, 'description')[0]
        m.brief = tag.getAttribute('brief') or ''
    except:
        # means that 'description' tag is missing
        pass
    #TODO: figure out how to multiplex
    if m._type == 'package':
        m.depends = check_depends('depend')(p, filename)
    elif m._type == 'stack':
        m.depends = check_stack_depends('depend')(p, filename)
    elif m._type == 'app':
        # not implemented yet
        pass
    m.rosdeps = check('rosdep')(p, filename)    
    m.platforms = check('platform')(p, filename)    
    m.exports = check('export')(p, filename)
    m.versioncontrol = check('versioncontrol')(p,filename)
    m.license = check('license')(p, filename)
    m.license_url = ''
    try:
        tag = get_nodes_by_name(p, 'license')[0]
        m.license_url = tag.getAttribute('url') or ''
    except:
        pass #manifest is missing required 'license' tag
  
    m.status='unreviewed'
    try:
        tag = get_nodes_by_name(p, 'review')[0]
        m.status=tag.getAttribute('status') or ''
    except:
        pass #manifest is missing optional 'review status' tag

    m.notes=''
    try:
        tag = get_nodes_by_name(p, 'review')[0]
        m.notes=tag.getAttribute('notes') or ''
    except:
        pass #manifest is missing optional 'review notes' tag

    m.author = check('author', True)(p, filename)
    m.url = check('url')(p, filename)
    m.version = check('version')(p, filename)
    m.logo = check('logo')(p, filename)

    # do some validation on what we just parsed
    if m._type == 'stack':
        if m.exports:
            raise ManifestException("stack manifests are not allowed to have exports")
        if m.rosdeps:
            raise ManifestException("stack manifests are not allowed to have rosdeps") 

    # store unrecognized tags
    m.unknown_tags = [e for e in p.childNodes if e.nodeType == e.ELEMENT_NODE and e.tagName not in VALID]
    return m
