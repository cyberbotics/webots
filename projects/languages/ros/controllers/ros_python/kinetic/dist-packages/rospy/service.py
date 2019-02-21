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

"""Base-classes and management of ROS services.
See L{rospy.tcpros_service} for actual implementation."""



import logging
import traceback

from rospy.core import *

from rospy.impl.registration import set_service_manager, Registration, get_registration_listeners
from rospy.impl.transport import *

logger = logging.getLogger('rospy.service')

class ServiceException(Exception):
    """Exception class for service-related errors"""
    pass

class _Service(object):
    """Internal-use superclass for storing service information"""
    def __init__(self, name, service_class):
        self.resolved_name = resolve_name(name) #services remap as well
        self.service_class = service_class
        self.request_class = service_class._request_class
        self.response_class = service_class._response_class
        self.uri = None #initialize attr

class ServiceManager(object):
    """Keeps track of currently registered services in the ROS system"""
    
    def __init__(self, registration_listeners=None):
        """
        ctor
        @param registration_listeners: override default registration listener.
        @type  registration_listeners: RegistrationListeners
        """
        self.map = {} # {name : Service}
        self.lock = threading.RLock()
        if registration_listeners is None:
            self.registration_listeners = get_registration_listeners()
        else:
            self.registration_listeners = registration_listeners       

    def get_services(self):
        """
        @return: List of (service_name, service_uri)  for all registered services.
        @rtype: [(str, str)]
        """
        with self.lock:
            ret_val = []
            for name, service in self.map.items():
                ret_val.append((name, service.uri))
            services = list(self.map.values())
        return ret_val
    
    def unregister_all(self):
        """
        Unregister all registered services
        """
        self.map.clear()
    
    def register(self, resolved_service_name, service):
        """
        Register service with ServiceManager and ROS master
        @param resolved_service_name: name of service (resolved)
        @type  resolved_service_name: str
        @param service: Service to register
        @type  service: L{_Service}
        """
        err = None
        with self.lock:
            if resolved_service_name in self.map:
                err = "service [%s] already registered"%resolved_service_name
            else:
                self.map[resolved_service_name] = service
                
            # NOTE: this call can potentially take a long time under lock and thus needs to be reimplmented
            self.registration_listeners.notify_added(resolved_service_name, service.uri, Registration.SRV)

        if err:
            raise ServiceException(err)
        
    def unregister(self, resolved_service_name, service):
        """
        Unregister service with L{ServiceManager} and ROS Master
        @param resolved_service_name: name of service
        @type  resolved_service_name: str
        @param service: service implementation
        @type  service: L{_Service}
        """        
        with self.lock:
            curr = self.map.get(resolved_service_name, None)
            if curr == service:
                del self.map[resolved_service_name]
                
            # NOTE: this call can potentially take a long time under lock
            self.registration_listeners.notify_removed(resolved_service_name, service.uri, Registration.SRV)                

    def get_service(self, resolved_service_name):
        """
        @param resolved_service_name: name of service
        @type  resolved_service_name: str
        @return: service implementation
        @rtype: _Service
        """
        return self.map.get(resolved_service_name, None)

set_service_manager(ServiceManager())
