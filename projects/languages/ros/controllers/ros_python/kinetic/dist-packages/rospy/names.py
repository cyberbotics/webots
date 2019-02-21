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
Support for ROS Names

See: U{http://www.ros.org/wiki/Names}
"""

import sys
import os

from rosgraph.names import namespace, get_ros_namespace, ns_join, make_global_ns, load_mappings, \
     SEP, GLOBALNS, REMAP, ANYTYPE, \
     is_global, is_private
import rosgraph.names

from rospy.exceptions import ROSException
from rospy.impl.validators import ParameterInvalid

TOPIC_ANYTYPE = ANYTYPE #indicates that a subscriber will connect any datatype given to it
SERVICE_ANYTYPE = ANYTYPE #indicates that a service client does not have a fixed type

import struct

if sys.hexversion > 0x03000000: #Python3
    def isstring(s):
        return isinstance(s, str) #Python 3.x
else:
    def isstring(s):
        return isinstance(s, basestring) #Python 2.x

def canonicalize_name(name):
    """
    Put name in canonical form. Double slashes '//' are removed and
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
    ##if len(name) > 1 and name[-1] == SEP:
    ##    return name[:-1]
    ##return name

# Mappings override name resolution by substituting fully-qualified
# names in for local name references. They override any name
# reference, with exception of '.local' names. We load remapping args
# as soon as client API is referenced so that they are initialized
# before Topic constructors are invoked.
_mappings = load_mappings(sys.argv)
_resolved_mappings = {}

def reload_mappings(argv):
    """
    Re-initialize the name remapping table.

    @param argv: Command line arguments to this program. ROS reads
        these arguments to find renaming params. 
    @type  argv: [str]
    """
    global _mappings
    _mappings = load_mappings(argv)

# #1810
def initialize_mappings(node_name):
    """
    Initialize the remapping table based on provide node name.

    @param node_name: name of node (caller ID)
    @type  node_name: str
    """
    global _resolved_mappings
    _resolved_mappings = {}
    for m,v in _mappings.items():
        # resolve both parts of the mappings. use the rosgraph.names
        # version of resolve_name to avoid circular mapping.
        if m.startswith('__'): # __name, __log, etc...
            _resolved_mappings[m] = v
        else:
            _resolved_mappings[rosgraph.names.resolve_name(m, node_name)] = rosgraph.names.resolve_name(v, node_name)

def resolve_name_without_node_name(name):
    """
    The need for this function is complicated -- Topics and Services can be created before init_node is called.
    In general, this is okay, unless the name is a ~name, in which
    case we have to raise an ValueError

    @param name: ROS name to resolve
    @type  name: str
    @raise ValueError: if name is a ~name
    @raise ROSInitException: if name is remapped to a ~name
    """
    if is_private(name):
        raise ValueError("~name topics cannot be created before init_node() has been called")

    # we use the underlying rosgraph.names.resolve_name to avoid dependencies on nodename/remappings
    fake_caller_id = ns_join(get_namespace(), 'node')
    fake_resolved = rosgraph.names.resolve_name(name, fake_caller_id)

    for m, v in _mappings.items():
        if rosgraph.names.resolve_name(m, fake_caller_id) == fake_resolved:
            if is_private(name):
                raise ROSInitException("due to the way this node is written, %s cannot be remapped to a ~name. \nThe declaration of topics/services must be moved after the call to init_node()"%name)
            else:
                return rosgraph.names.resolve_name(v, fake_caller_id)
    return fake_resolved

def get_mappings():
    """
    Get mapping table with unresolved names
    
    @return: command-line remappings {name: name}
    @rtype: {str: str}
    """
    return _mappings

def get_resolved_mappings():
    """
    Get mapping table with resolved names
    
    @return: command-line remappings {name: name}
    @rtype: {str: str}
    """
    return _resolved_mappings

#TODO: port to a wrapped call to rosgraph.names.resolve_name
def resolve_name(name, caller_id=None):
    """
    Resolve a ROS name to its global, canonical form. Private ~names
    are resolved relative to the node name. 

    @param name: name to resolve.
    @type  name: str
    @param caller_id: node name to resolve relative to. To
    resolve to local namespace, omit this parameter (or use None)
    @type  caller_id: str
    @return: Resolved name. If name is empty/None, resolve_name
    returns parent namespace. If namespace is empty/None,
    @rtype: str
    """
    if not caller_id:
        caller_id = get_name()
    if not name: #empty string resolves to namespace
        return namespace(caller_id)

    name = str(name)  # enforce string conversion else struct.pack might raise UnicodeDecodeError (see #3998)
    name = canonicalize_name(name)
    if name[0] == SEP: #global name
        resolved_name = name
    elif is_private(name): #~name
        resolved_name = ns_join(caller_id, name[1:])
    else: #relative
        resolved_name = namespace(caller_id) + name

    #Mappings override general namespace-based resolution
    # - do this before canonicalization as remappings are meant to
    #   match the name as specified in the code
    if resolved_name in _resolved_mappings:
        return _resolved_mappings[resolved_name]
    else:
        return resolved_name


def remap_name(name, caller_id=None, resolved=True):
    """
    Remap a ROS name. This API should be used to instead of
    resolve_name for APIs in which you don't wish to resolve the name
    unless it is remapped.
    @param name: name to remap
    @type  name: str
    
    @param resolved: if True (default), use resolved names in remappings, which is the standard for ROS. 
    @type  resolved: bool
    
    @return: Remapped name
    @rtype: str
    """
    if not caller_id:
        caller_id = get_caller_id()
    if name in _mappings:
        return rosgraph.names.resolve_name(_mappings[name], caller_id)
    return name

def scoped_name(caller_id, name):
    """
    Convert the global caller_id to a relative name within the namespace. For example, for
    namespace '/foo' and name '/foo/bar/name', the return value will
    be 'bar/name'

    WARNING: scoped_name does not validate that name is actually within
    the supplied namespace.
    @param caller_id: caller ID, in canonical form
    @type  caller_id: str
    @param name: name to scope
    @type  name: str
    @return: name scoped to the caller_id's namespace. 
    @rtype: str
    """
    if not is_global(caller_id):
        raise ROSException("caller_id must be global")
    return canonicalize_name(name)[len(namespace(caller_id)):]


###################################################
# Name validators      ############################

#Technically XMLRPC will never send a None, but I don't want to code masterslave.py to be
#XML-RPC specific in this way.

def valid_name_validator_resolved(param_name, param_value, caller_id):
    if not param_value or not isstring(param_value):
        raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
    #TODO: actual validation of chars
    # I added the colon check as the common error will be to send an URI instead of name
    if ':' in param_value or ' ' in param_value:
        raise ParameterInvalid("ERROR: parameter [%s] contains illegal chars"%param_name) 
    #don't use our own resolve_name because we do not want to remap
    return rosgraph.names.resolve_name(param_value, caller_id, remappings=None)

def valid_name_validator_unresolved(param_name, param_value, caller_id):
    if not param_value or not isstring(param_value):
        raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)            
    #TODO: actual validation of chars        
    # I added the colon check as the common error will be to send an URI instead of name
    if ':' in param_value or ' ' in param_value:
        raise ParameterInvalid("ERROR: parameter [%s] contains illegal chars"%param_name) 
    return param_value
    
def valid_name(param_name, resolve=True):
    """
    Validator that resolves names and also ensures that they are not empty
    @param param_name: name
    @type  param_name: str
    @param resolve: if True/omitted, the name will be resolved to
       a global form. Otherwise, no resolution occurs.
    @type  resolve: bool
    @return: resolved parameter value
    @rtype: str
    """
    def validator(param_value, caller_id):
        if resolve:
            return valid_name_validator_resolved(param_name, param_value, caller_id)
        return valid_name_validator_unresolved(param_name, param_value, caller_id)        
    return validator

def global_name(param_name):
    """
    Validator that checks for valid, global graph resource name.
    @return: parameter value
    @rtype: str
    """
    def validator(param_value, caller_id):
        if not param_value or not isstring(param_value):
            raise ParameterInvalid("ERROR: parameter [%s] must be a non-empty string"%param_name)
        #TODO: actual validation of chars
        if not is_global(param_value):
            raise ParameterInvalid("ERROR: parameter [%s] must be a globally referenced name"%param_name)            
        return param_value
    return validator

#########################################################
#Global Namespace Routines
# - Global state, e.g. singletons and namespace

_caller_namespace = get_ros_namespace()
_caller_id = _caller_namespace+'unnamed' #default for non-node. 

def get_namespace():
    """
    Get namespace of local node. 
    @return: fully-qualified name of local node or '' if not applicable
    @rtype: str
    """
    return _caller_namespace

def get_name():
    """
    Get fully resolved name of local node. If this is not a node,
    use empty string
    @return: fully-qualified name of local node or '' if not applicable
    @rtype: str
    """    
    return _caller_id

# backwards compatibility
get_caller_id = get_name

def _set_caller_id(caller_id):
    """
    Internal API.
    Set the global name (i.e. caller_id) and namespace. Methods can
    check what the name of the current node is by calling get_caller_id.

    The caller_id is important as it is the first parameter to any API
    call on a remote node.  Invoked by ROSNode constructor
    @param caller_id: new caller ID
    @type  caller_id: str
    """    
    global _caller_id, _caller_namespace
    _caller_id = caller_id
    _caller_namespace = namespace(caller_id)

