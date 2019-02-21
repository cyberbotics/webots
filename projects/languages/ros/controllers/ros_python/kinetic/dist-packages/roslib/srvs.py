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
ROS Service Description Language Spec
Implements U{http://ros.org/wiki/srv}
"""

import os
import sys
import re

try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x

import roslib.msgs
import roslib.names
import roslib.packages
import roslib.resources

# don't directly use code from this, though we do depend on the
# manifest.Depend data type
import roslib.manifest

## file extension
EXT = '.srv' #alias
SEP = '/' #e.g. std_msgs/String
## input/output deliminator
IODELIM   = '---'
COMMENTCHAR = roslib.msgs.COMMENTCHAR

VERBOSE = False
## @return: True if msg-related scripts should print verbose output
def is_verbose():
    return VERBOSE

## set whether msg-related scripts should print verbose output
def set_verbose(v):
    global VERBOSE
    VERBOSE = v

class SrvSpecException(Exception): pass

# msg spec representation ##########################################

class SrvSpec(object):
    
    def __init__(self, request, response, text, full_name = '', short_name = '', package = ''):
        self.request = request
        self.response = response
        self.text = text
        self.full_name = full_name
        self.short_name = short_name
        self.package = package
        
    def __eq__(self, other):
        if not other or not isinstance(other, SrvSpec):
            return False
        return self.request == other.request and \
               self.response == other.response and \
               self.text == other.text and \
               self.full_name == other.full_name and \
               self.short_name == other.short_name and \
               self.package == other.package
    
    def __ne__(self, other):
        if not other or not isinstance(other, SrvSpec):
            return True
        return not self.__eq__(other)

    def __repr__(self):
        return "SrvSpec[%s, %s]"%(repr(self.request), repr(self.response))
    
# srv spec loading utilities ##########################################

## @internal
## predicate for filtering directory list. matches message files
def _srv_filter(f):
    return os.path.isfile(f) and f.endswith(EXT)

# also used by doxymaker
def list_srv_types(package, include_depends):
    """
    list all services in the specified package
    @param package: name of package to search
    @type  package: str
    @param include_depends: if True, will also list services in package dependencies
    @type  include_depends: bool
    @return: service type names
    @rtype: [str]
    """
    types = roslib.resources.list_package_resources(package, include_depends, 'srv', _srv_filter)
    return [x[:-len(EXT)] for x in types]

def srv_file(package, type_):
    """
    @param package: name of package .srv file is in
    @type  package: str
    @param type_: type name of service
    @type  type_: str
    @return: file path of .srv file in specified package
    @rtype: str
    """
    return roslib.packages.resource_file(package, 'srv', type_+EXT)

def get_pkg_srv_specs(package):
    """
    List all messages that a package contains
    @param depend: roslib.manifest.Depend object representing package
    to load messages from
    @type  depend: Depend
    @return: list of message type names and specs for package, as well as a list
    of message names that could not be processed. 
    @rtype: [(str,roslib.MsgSpec), [str]]
    """
    #almost identical to roslib.msgs.get_pkg_msg_specs
    types = list_srv_types(package, False)
    specs = [] #no fancy list comprehension as we want to show errors
    failures = []
    for t in types:
        try: 
            spec = load_from_file(srv_file(package, t), package)
            specs.append(spec)
        except Exception as e:
            failures.append(t)
            sys.stderr.write("ERROR: unable to load %s\n"%(t))
    return specs, failures

def load_from_string(text, package_context='', full_name='', short_name=''):
    """
    @param text: .msg text 
    @type  text: str
    @param package_context: context to use for msgTypeName, i.e. the package name,
    or '' to use local naming convention.
    @type  package_context: str
    @return: Message type name and message specification
    @rtype: roslib.MsgSpec
    @raise roslib.MsgSpecException: if syntax errors or other problems are detected in file
    """
    text_in  = StringIO()
    text_out = StringIO()
    accum = text_in
    for l in text.split('\n'):
        l = l.split(COMMENTCHAR)[0].strip() #strip comments        
        if l.startswith(IODELIM): #lenient, by request
            accum = text_out
        else:
            accum.write(l+'\n')
    # create separate roslib.msgs objects for each half of file
    
    msg_in = roslib.msgs.load_from_string(text_in.getvalue(), package_context, '%sRequest'%(full_name), '%sRequest'%(short_name))
    msg_out = roslib.msgs.load_from_string(text_out.getvalue(), package_context, '%sResponse'%(full_name), '%sResponse'%(short_name))
    return SrvSpec(msg_in, msg_out, text, full_name, short_name, package_context)

def load_from_file(file_name, package_context=''):
    """
    Convert the .srv representation in the file to a SrvSpec instance.
    @param file_name: name of file to load from
    @type  file_name: str
    @param package_context: context to use for type name, i.e. the package name,
    or '' to use local naming convention.
    @type package_context: str
    @return: Message type name and message specification
    @rtype: (str, L{SrvSpec})
    @raise SrvSpecException: if syntax errors or other problems are detected in file
    """
    if VERBOSE:
        if package_context:
            sys.stdout.write("Load spec from %s into namespace [%s]\n"%(file_name, package_context))
        else:
            sys.stdout.write("Load spec from %s\n"%(file_name))
    base_file_name = os.path.basename(file_name)
    type_ = base_file_name[:-len(EXT)]
    base_type_ = type_
    # determine the type name
    if package_context:
        while package_context.endswith(SEP):
            package_context = package_context[:-1] #strip message separators
        type_ = "%s%s%s"%(package_context, SEP, type_)
    if not roslib.names.is_legal_resource_name(type_):
        raise SrvSpecException("%s: %s is not a legal service type name"%(file_name, type_))
    
    f = open(file_name, 'r')
    try:
        text = f.read()
        return (type_, load_from_string(text, package_context, type_, base_type_))
    finally:
        f.close()




