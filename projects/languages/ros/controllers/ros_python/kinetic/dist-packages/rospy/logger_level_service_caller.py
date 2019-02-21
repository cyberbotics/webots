# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Chris Mansley, Open Source Robotics Foundation, Inc.
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

import rosgraph
import rosnode
import rospy
import rosservice


class ROSConsoleException(Exception):

    """Base exception class of rosconsole-related errors."""

    pass


class LoggerLevelServiceCaller(object):

    """
    Handles service calls for getting lists of nodes and loggers.

    Also handles sending requests to change logger levels.
    """

    def __init__(self):
        pass

    def get_levels(self):
        # Declare level names lower-case, because that's how they are returned
        # from the service call.
        return ['debug', 'info', 'warn', 'error', 'fatal']

    def get_loggers(self, node):
        self._refresh_loggers(node)
        return self._current_loggers

    def get_node_names(self):
        """
        Get a list of available services via a ros service call.

        :returns: a list of all nodes that provide the set_logger_level service, ''list(str)''
        """
        set_logger_level_nodes = []
        nodes = rosnode.get_node_names()
        for name in sorted(nodes):
            for service in rosservice.get_service_list(name):
                if service == name + '/set_logger_level':
                    set_logger_level_nodes.append(name)
        return set_logger_level_nodes

    def _refresh_loggers(self, node):
        """
        Store a list of loggers available for passed in node.

        :param node: name of the node to query, ''str''
        :raises: :exc:`ROSTopicException` If topic type cannot be determined or loaded
        """
        self._current_loggers = []
        self._current_levels = {}
        # Construct the service name, taking into account our namespace
        servicename = rosgraph.names.ns_join(
            rosgraph.names.ns_join(rosgraph.names.get_ros_namespace(), node),
            'get_loggers')
        # Construct the service name, taking into account our namespace
        servicename = rosgraph.names.resolve_name(
            servicename, rosgraph.names.get_ros_namespace())
        try:
            service = rosservice.get_service_class_by_name(servicename)
        except rosservice.ROSServiceException as e:
            raise ROSConsoleException(
                "node '%s' doesn't exist or doesn't support query: %s" % (node, e))

        request = service._request_class()
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
        except rospy.ServiceException as e:
            raise ROSConsoleException("node '%s' logger request failed: %s" % (node, e))

        if response._slot_types[0] == 'roscpp/Logger[]':
            for logger in getattr(response, response.__slots__[0]):
                self._current_loggers.append(logger.name)
                self._current_levels[logger.name] = logger.level
        else:
            raise ROSConsoleException(repr(response))

    def send_logger_change_message(self, node, logger, level):
        """
        Send a logger level change request to 'node'.

        :param node: name of the node to chaange, ''str''
        :param logger: name of the logger to change, ''str''
        :param level: name of the level to change, ''str''
        :returns: True if the response is valid, ''bool''
        :returns: False if the request raises an exception or would not change the state, ''bool''
        """
        # Construct the service name, taking into account our namespace
        servicename = rosgraph.names.ns_join(
            rosgraph.names.ns_join(rosgraph.names.get_ros_namespace(), node),
            'set_logger_level')
        # Construct the service name, taking into account our namespace
        servicename = rosgraph.names.resolve_name(
            servicename, rosgraph.names.get_ros_namespace())
        if self._current_levels[logger] == level:
            return False

        service = rosservice.get_service_class_by_name(servicename)
        request = service._request_class()
        request.logger = logger
        request.level = level
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            proxy(request)
            self._current_levels[logger] = level.upper()
        except rospy.ServiceException as e:
            raise ROSConsoleException("node '%s' logger request failed: %s" % (node, e))

        return True
