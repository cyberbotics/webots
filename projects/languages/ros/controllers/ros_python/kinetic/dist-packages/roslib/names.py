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

"""
Warning: do not use this library.  It is unstable and most of the routines
here have been superceded by other libraries (e.g. genmsg).  These
routines will likely be *deleted* in future releases.
"""

import os
import sys

#TODO: deprecate PRN_SEPARATOR
PRN_SEPARATOR = '/'
TYPE_SEPARATOR = PRN_SEPARATOR #alias
SEP = '/'
GLOBALNS = '/'
PRIV_NAME = '~'
REMAP = ":="
ANYTYPE = '*'

if sys.hexversion > 0x03000000: #Python3
    def isstring(s):
        return isinstance(s, str) #Python 3.x
else:
    def isstring(s):
        """
        Small helper version to check an object is a string in a way that works
        for both Python 2 and 3
        """
        return isinstance(s, basestring) #Python 2.x

def get_ros_namespace(env=None, argv=None):
    """
    @param env: environment dictionary (defaults to os.environ)
    @type  env: dict
    @param argv: command-line arguments (defaults to sys.argv)
    @type  argv: [str]
    @return: ROS namespace of current program
    @rtype: str
    """    
    #we force command-line-specified namespaces to be globally scoped
    if argv is None:
        argv = sys.argv
    for a in argv:
        if a.startswith('__ns:='):
            return make_global_ns(a[len('__ns:='):])
    if env is None:
        env = os.environ
    return make_global_ns(env.get('ROS_NAMESPACE', GLOBALNS))

def make_caller_id(name):
    """
    Resolve a local name to the caller ID based on ROS environment settings (i.e. ROS_NAMESPACE)

    @param name: local name to calculate caller ID from, e.g. 'camera', 'node'
    @type  name: str
    @return: caller ID based on supplied local name
    @rtype: str
    """    
    return make_global_ns(ns_join(get_ros_namespace(), name))

def make_global_ns(name):
    """
    Convert name to a global name with a trailing namespace separator.
    
    @param name: ROS resource name. Cannot be a ~name.
    @type  name: str
    @return str: name as a global name, e.g. 'foo' -> '/foo/'.
        This does NOT resolve a name.
    @rtype: str
    @raise ValueError: if name is a ~name 
    """    
    if is_private(name):
        raise ValueError("cannot turn [%s] into a global name"%name)
    if not is_global(name):
        name = SEP + name
    if name[-1] != SEP:
        name = name + SEP
    return name

def is_global(name):
    """
    Test if name is a global graph resource name.
    
    @param name: must be a legal name in canonical form
    @type  name: str
    @return: True if name is a globally referenced name (i.e. /ns/name)
    @rtype: bool
    """    
    return name and name[0] == SEP

def is_private(name):
    """
    Test if name is a private graph resource name.
    
    @param name: must be a legal name in canonical form
    @type  name: str
    @return bool: True if name is a privately referenced name (i.e. ~name)
    """    
    return name and name[0] == PRIV_NAME

def namespace(name):
    """
    Get the namespace of name. The namespace is returned with a
    trailing slash in order to favor easy concatenation and easier use
    within the global context.
        
    @param name: name to return the namespace of. Must be a legal
        name. NOTE: an empty name will return the global namespace.
    @type  name: str
    @return str: Namespace of name. For example, '/wg/node1' returns '/wg/'. The
        global namespace is '/'. 
    @rtype: str
    @raise ValueError: if name is invalid
    """    
    "map name to its namespace"
    if name is None: 
        raise ValueError('name')
    if not isstring(name):
        raise TypeError('name')
    if not name:
        return SEP
    elif name[-1] == SEP:
        name = name[:-1]
    return name[:name.rfind(SEP)+1] or SEP

def ns_join(ns, name):
    """
    Join a namespace and name. If name is unjoinable (i.e. ~private or
    /global) it will be returned without joining

    @param ns: namespace ('/' and '~' are both legal). If ns is the empty string, name will be returned.
    @type  ns: str
    @param name str: a legal name
    @return str: name concatenated to ns, or name if it is
        unjoinable.
    @rtype: str
    """    
    if is_private(name) or is_global(name):
        return name
    if ns == PRIV_NAME:
        return PRIV_NAME + name
    if not ns: 
        return name
    if ns[-1] == SEP:
        return ns + name
    return ns + SEP + name

def load_mappings(argv):
    """
    Load name mappings encoded in command-line arguments. This will filter
    out any parameter assignment mappings (see roslib.param.load_param_mappings()).

    @param argv: command-line arguments
    @type  argv: [str]
    @return: name->name remappings. 
    @rtype: dict {str: str}
    """    
    mappings = {}
    for arg in argv:
        if REMAP in arg:
            try:
                src, dst = [x.strip() for x in arg.split(REMAP)]
                if src and dst:
                    if len(src) > 1 and src[0] == '_' and src[1] != '_':
                        #ignore parameter assignment mappings
                        pass
                    else:
                        mappings[src] = dst
            except:
                sys.stderr.write("ERROR: Invalid remapping argument '%s'\n"%arg)
    return mappings

#######################################################################
# RESOURCE NAMES
# resource names refer to entities in a file system

def resource_name(res_pkg_name, name, my_pkg=None):
    """
    Convert package name + resource into a fully qualified resource name

    @param res_pkg_name: name of package resource is located in
    @type  res_pkg_name: str
    @param name: resource base name
    @type  name: str
    @param my_pkg: name of package resource is being referred to
        in. If specified, name will be returned in local form if 
        res_pkg_name is my_pkg
    @type  my_pkg: str
    @return: name for resource 
    @rtype: str
    """    
    if res_pkg_name != my_pkg:
        return res_pkg_name+PRN_SEPARATOR+name
    return name

def resource_name_base(name):
    """
    pkg/typeName -> typeName, typeName -> typeName
    
    Convert fully qualified resource name into the package-less resource name
    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: resource name sans package-name scope
    @rtype: str
    """    

    return name[name.rfind(PRN_SEPARATOR)+1:]

def resource_name_package(name):
    """
    pkg/typeName -> pkg, typeName -> None
    
    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: package name of resource
    @rtype: str
    """    

    if not PRN_SEPARATOR in name:
        return None
    return name[:name.find(PRN_SEPARATOR)]

def package_resource_name(name):
    """
    Split a name into its package and resource name parts, e.g. 'std_msgs/String -> std_msgs, String'

    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: package name, resource name
    @rtype: str
    @raise ValueError: if name is invalid
    """    
    if PRN_SEPARATOR in name:
        val = tuple(name.split(PRN_SEPARATOR))
        if len(val) != 2:
            raise ValueError("invalid name [%s]"%name)
        else:
            return val
    else:
        return '', name

def _is_safe_name(name, type_name):
    #windows long-file name length is 255
    if not isstring(name) or not name or len(name) > 255:
        return False
    return is_legal_resource_name(name)

################################################################################
# NAME VALIDATORS

import re
#ascii char followed by (alphanumeric, _, /)
RESOURCE_NAME_LEGAL_CHARS_P = re.compile('^[A-Za-z][\w_\/]*$') 
def is_legal_resource_name(name):
    """
    Check if name is a legal ROS name for filesystem resources
    (alphabetical character followed by alphanumeric, underscore, or
    forward slashes). This constraint is currently not being enforced,
    but may start getting enforced in later versions of ROS.

    @param name: Name
    @type  name: str
    """
    # resource names can be unicode due to filesystem
    if name is None:
        return False
    m = RESOURCE_NAME_LEGAL_CHARS_P.match(name)
    # '//' check makes sure there isn't double-slashes
    return m is not None and m.group(0) == name and not '//' in name

#~,/, or ascii char followed by (alphanumeric, _, /)
NAME_LEGAL_CHARS_P = re.compile('^[\~\/A-Za-z][\w_\/]*$') 
def is_legal_name(name):
    """
    Check if name is a legal ROS name for graph resources
    (alphabetical character followed by alphanumeric, underscore, or
    forward slashes). This constraint is currently not being enforced,
    but may start getting enforced in later versions of ROS.

    @param name: Name
    @type  name: str
    """    
    # should we enforce unicode checks?
    if name is None:
        return False
    # empty string is a legal name as it resolves to namespace
    if name == '':
        return True
    m = NAME_LEGAL_CHARS_P.match(name)
    return m is not None and m.group(0) == name and not '//' in name
    
BASE_NAME_LEGAL_CHARS_P = re.compile('^[A-Za-z][\w_]*$') #ascii char followed by (alphanumeric, _)
def is_legal_base_name(name):
    """
    Validates that name is a legal base name for a graph resource. A base name has
    no namespace context, e.g. "node_name".
    """
    if name is None:
        return False
    m = BASE_NAME_LEGAL_CHARS_P.match(name)
    return m is not None and m.group(0) == name

BASE_RESOURCE_NAME_LEGAL_CHARS_P = re.compile('^[A-Za-z][\w_]*$') #ascii char followed by (alphanumeric, _)
def is_legal_resource_base_name(name):
    """
    Validates that name is a legal resource base name. A base name has
    no package context, e.g. "String".
    """
    # resource names can be unicode due to filesystem
    if name is None:
        return False
    m = BASE_NAME_LEGAL_CHARS_P.match(name)
    return m is not None and m.group(0) == name

def canonicalize_name(name):
    """
    Put name in canonical form. Extra slashes '//' are removed and
    name is returned without any trailing slash, e.g. /foo/bar
    @param name: ROS name
    @type  name: str
    """
    if not name or name == SEP:
        return name
    elif name[0] == SEP:
        return '/' + '/'.join([x for x in name.split(SEP) if x])
    else:
        return '/'.join([x for x in name.split(SEP) if x])        

def resolve_name(name, namespace_, remappings=None):
    """
    Resolve a ROS name to its global, canonical form. Private ~names
    are resolved relative to the node name. 

    @param name: name to resolve.
    @type  name: str
    @param namespace_: node name to resolve relative to.
    @type  namespace_: str
    @param remappings: Map of resolved remappings. Use None to indicate no remapping.
    @return: Resolved name. If name is empty/None, resolve_name
    returns parent namespace_. If namespace_ is empty/None,
    @rtype: str
    """
    if not name: #empty string resolves to parent of the namespace_
        return namespace(namespace_)

    name = canonicalize_name(name)
    if name[0] == SEP: #global name
        resolved_name = name
    elif is_private(name): #~name
        # #3044: be careful not to accidentally make rest of name global
        resolved_name = canonicalize_name(namespace_ + SEP + name[1:])
    else: #relative
        resolved_name = namespace(namespace_) + name

    #Mappings override general namespace-based resolution
    # - do this before canonicalization as remappings are meant to
    #   match the name as specified in the code
    if remappings and resolved_name in remappings:
        return remappings[resolved_name]
    else:
        return resolved_name

def anonymous_name(id):
    """
    Generate a ROS-legal 'anonymous' name

    @param id: prefix for anonymous name
    @type  id: str
    """
    import socket, random
    name = "%s_%s_%s_%s"%(id, socket.gethostname(), os.getpid(), random.randint(0, sys.maxsize))
    # RFC 952 allows hyphens, IP addrs can have '.'s, both
    # of which are illegal for ROS names. For good
    # measure, screen ipv6 ':'. 
    name = name.replace('.', '_')
    name = name.replace('-', '_')                
    return name.replace(':', '_')

