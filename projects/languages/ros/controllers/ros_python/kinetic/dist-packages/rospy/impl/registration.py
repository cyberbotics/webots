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

"""Internal use: handles maintaining registrations with master via internal listener APIs"""



import socket
import sys
import logging
import threading
import time
import traceback
try:
    import xmlrpc.client as xmlrpcclient
except ImportError:
    import xmlrpclib as xmlrpcclient

from rospy.core import is_shutdown, is_shutdown_requested, xmlrpcapi, \
    logfatal, logwarn, loginfo, logerr, logdebug, \
    signal_shutdown, add_preshutdown_hook
from rospy.names import get_caller_id, get_namespace

# topic manager and service manager singletons

_topic_manager = None
def set_topic_manager(tm):
    global _topic_manager
    _topic_manager = tm
def get_topic_manager():
    return _topic_manager

_service_manager = None
def set_service_manager(sm):
    global _service_manager
    _service_manager = sm
def get_service_manager():
    return _service_manager

    
class Registration(object):
    """Registration types"""
    PUB = 'pub'
    SUB = 'sub'
    SRV = 'srv'
    
class RegistrationListener(object):
    """Listener API for subscribing to changes in Publisher/Subscriber/Service declarations"""

    def reg_added(self, resolved_name, data_type_or_uri, reg_type): 
        """
        New pub/sub/service declared.
        @param resolved_name: resolved topic/service name
        @param data_type_or_uri: topic type or service uri
        @type  data_type_or_uri: str
        @param reg_type: Valid values are L{Registration.PUB}, L{Registration.SUB}, L{Registration.SRV}
        @type  reg_type: str
        """
        pass
    
    def reg_removed(self, resolved_name, data_type_or_uri, reg_type): 
        """
        New pub/sub/service removed.
        @param resolved_name: topic/service name
        @type  resolved_name: str
        @param data_type_or_uri: topic type or service uri
        @type  data_type_or_uri: str
        @param reg_type: Valid values are L{Registration.PUB}, L{Registration.SUB}, L{Registration.SRV}
        @type  reg_type: str
        """
        pass

class RegistrationListeners(object):
    
    def __init__(self):
        """
        ctor.
        """
        self.listeners = []
        self.lock = threading.Lock()

    def add_listener(self, l):
        """
        Subscribe to notifications of pub/sub/service registration
        changes. This is an internal API used to notify higher level
        routines when to communicate with the master.
        @param l: listener to subscribe
        @type  l: TopicListener
        """
        assert isinstance(l, RegistrationListener)
        with self.lock:
            self.listeners.append(l)

    def notify_removed(self, resolved_name, data_type_or_uri, reg_type):
        """
        @param resolved_name: resolved_topic/service name
        @type  resolved_name: str
        @param data_type_or_uri: topic type or service uri
        @type  data_type_or_uri: str
        @param reg_type: Valid values are L{Registration.PUB}, L{Registration.SUB}, L{Registration.SRV}
        @type  reg_type: str
        """
        with self.lock:
            for l in self.listeners:
                try:
                    l.reg_removed(resolved_name, data_type_or_uri, reg_type)
                except Exception as e:
                    logerr("error notifying listener of removal: %s"%traceback.format_exc())
            
    def notify_added(self, resolved_name, data_type, reg_type):
        """
        @param resolved_name: topic/service name
        @type  resolved_name: str
        @param data_type: topic/service type
        @type  data_type: str
        @param reg_type: Valid values are L{Registration.PUB}, L{Registration.SUB}, L{Registration.SRV}
        @type  reg_type: str
        """
        with self.lock:
            for l in self.listeners:
                try:
                    l.reg_added(resolved_name, data_type, reg_type)
                except Exception as e:
                    logerr(traceback.format_exc())
                    
    def clear(self):
        """
        Remove all registration listeners
        """
        if not is_shutdown_requested():
            with self.lock:
                del self.listeners[:]
        else:
            # when being in shutdown phase the lock might not be lockable
            # if a notify_added/removed is currently ongoing
            locked = self.lock.acquire(False)
            # remove all listeners anyway
            del self.listeners[:]
            if locked:
                self.lock.release()
            
_registration_listeners = RegistrationListeners()
def get_registration_listeners():
    return _registration_listeners

# RegManager's main purpose is to collect all client->master communication in one place

class RegManager(RegistrationListener):
    """
    Registration manager used by Node implemenation.
    Communicates with ROS Master to maintain topic registration
    information. Also responds to publisher updates to create topic
    connections
    """

    def __init__(self, handler):
        """
        ctor.
        @param handler: node API handler
        """
        self.logger = logging.getLogger("rospy.registration")
        self.handler = handler
        self.uri = self.master_uri = None
        self.updates = []
        self.cond = threading.Condition() #for locking/notifying updates
        self.registered = False
        # cleanup has to occur before official shutdown
        add_preshutdown_hook(self.cleanup)
        
    def start(self, uri, master_uri):
        """
        Start the RegManager. This should be passed in as an argument to a thread
        starter as the RegManager is designed to spin in its own thread
        @param uri: URI of local node
        @type  uri: str
        @param master_uri: Master URI
        @type  master_uri: str
        """
        self.registered = False 
        self.master_uri = master_uri
        self.uri = uri
        first = True
        tm = get_topic_manager()
        sm = get_service_manager()
        ns = get_namespace()
        caller_id = get_caller_id()
        if not master_uri or master_uri == uri:
            registered = True
            master = None
        else:
            registered = False
            master = xmlrpcapi(master_uri)
            self.logger.info("Registering with master node %s", master_uri)

        while not registered and not is_shutdown():
            try:
                try:
                    # prevent TopicManager and ServiceManager from accepting registrations until we are done
                    tm.lock.acquire()
                    sm.lock.acquire()                    

                    pub, sub, srv = tm.get_publications(), tm.get_subscriptions(), sm.get_services()
                    for resolved_name, data_type in pub:
                        self.logger.info("Registering publisher topic [%s] type [%s] with master", resolved_name, data_type)
                        code, msg, val = master.registerPublisher(caller_id, resolved_name, data_type, uri)
                        if code != 1:
                            logfatal("cannot register publication topic [%s] with master: %s"%(resolved_name, msg))
                            signal_shutdown("master/node incompatibility with register publisher")
                    for resolved_name, data_type in sub:
                        self.logger.info("registering subscriber topic [%s] type [%s] with master", resolved_name, data_type)
                        code, msg, val = master.registerSubscriber(caller_id, resolved_name, data_type, uri)
                        if code != 1:
                            logfatal("cannot register subscription topic [%s] with master: %s"%(resolved_name, msg))
                            signal_shutdown("master/node incompatibility with register subscriber")                        
                        else:
                            self.publisher_update(resolved_name, val)
                    for resolved_name, service_uri in srv:
                        self.logger.info("registering service [%s] uri [%s] with master", resolved_name, service_uri)
                        code, msg, val = master.registerService(caller_id, resolved_name, service_uri, uri)
                        if code != 1:
                            logfatal("cannot register service [%s] with master: %s"%(resolved_name, msg))
                            signal_shutdown("master/node incompatibility with register service")                        
 
                    registered = True
                    
                    # Subscribe to updates to our state
                    get_registration_listeners().add_listener(self)
                finally:
                    sm.lock.release()                    
                    tm.lock.release()
                
                if pub or sub:
                    logdebug("Registered [%s] with master node %s", caller_id, master_uri)
                else:
                    logdebug("No topics to register with master node %s", master_uri)
                    
            except Exception as e:
                if first:
                    # this use to print to console always, arguable whether or not this should be subjected to same configuration options as logging
                    logerr("Unable to immediately register with master node [%s]: master may not be running yet. Will keep trying."%master_uri)
                    first = False
                time.sleep(0.2)
        self.registered = True
        self.run()
        
    def is_registered(self):
        """
        Check if Node has been registered yet.
        @return: True if registration has occurred with master
        @rtype: bool
        """
        return self.registered 

    def run(self):
        """
        Main RegManager thread loop.
        Periodically checks the update
        queue and generates topic connections
        """
        #Connect the topics
        while not self.handler.done and not is_shutdown():
            cond = self.cond
            try:
                cond.acquire()
                if not self.updates:
                    cond.wait(0.5)
                if self.updates:
                    #work from the end as these are the most up-to-date
                    topic, uris = self.updates.pop()
                    #filter out older updates for same topic
                    self.updates = [x for x in self.updates if x[0] != topic]
                else:
                    topic = uris = None
            finally:
                if cond is not None:
                    cond.release()

            get_topic_manager().check_all()

            #call _connect_topic on all URIs as it can check to see whether
            #or not a connection exists.
            if uris and not self.handler.done:
                for uri in uris:
                    # #1141: have to multithread this to prevent a bad publisher from hanging us
                    t = threading.Thread(target=self._connect_topic_thread, args=(topic, uri))
                    t.setDaemon(True)
                    t.start()

    def _connect_topic_thread(self, topic, uri):
        try:
            code, msg, _ = self.handler._connect_topic(topic, uri)
            if code != 1:
                logdebug("Unable to connect subscriber to publisher [%s] for topic [%s]: %s", uri, topic, msg)
        except Exception as e:
            if not is_shutdown():
                logdebug("Unable to connect to publisher [%s] for topic [%s]: %s"%(uri, topic, traceback.format_exc()))
        
    def cleanup(self, reason):
        """
        Cleans up registrations with master and releases topic and service resources
        @param reason: human-reasonable debug string
        @type  reason: str
        """
        self.logger.debug("registration cleanup starting")
        try:
            self.cond.acquire()
            self.cond.notifyAll()
        finally:
            self.cond.release()        

        # we never successfully initialized master_uri
        if not self.master_uri:
            return
        
        master = xmlrpcapi(self.master_uri)
        # we never successfully initialized master
        if master is None:
            return
        
        caller_id = get_caller_id()

        # clear the registration listeners as we are going to do a quick unregister here
        rl = get_registration_listeners()
        if rl is not None:
            rl.clear()
            
        tm = get_topic_manager()
        sm = get_service_manager()
        try:
            multi = xmlrpcclient.MultiCall(master)
            if tm is not None:
                for resolved_name, _ in tm.get_subscriptions():
                    self.logger.debug("unregisterSubscriber [%s]"%resolved_name)
                    multi.unregisterSubscriber(caller_id, resolved_name, self.uri)
                for resolved_name, _ in tm.get_publications():
                    self.logger.debug("unregisterPublisher [%s]"%resolved_name)                    
                    multi.unregisterPublisher(caller_id, resolved_name, self.uri)

            if sm is not None:
                for resolved_name, service_uri in sm.get_services():
                    self.logger.debug("unregisterService [%s]"%resolved_name) 
                    multi.unregisterService(caller_id, resolved_name, service_uri)
            multi()
        except socket.error as se:
            (errno, msg) = se.args
            if errno == 111 or errno == 61: #can't talk to master, nothing we can do about it
                self.logger.warn("cannot unregister with master due to network issues")
            else:
                self.logger.warn("unclean shutdown\n%s"%traceback.format_exc())
        except:
            self.logger.warn("unclean shutdown\n%s"%traceback.format_exc())

        self.logger.debug("registration cleanup: master calls complete")            

        #TODO: cleanup() should actually be orchestrated by a separate
        #cleanup routine that calls the reg manager/sm/tm
        if tm is not None:
            tm.close_all()
        if sm is not None:
            sm.unregister_all()

    def reg_removed(self, resolved_name, data_type_or_uri, reg_type):
        """
        RegistrationListener callback
        @param resolved_name: resolved name of topic or service
        @type  resolved_name: str
        @param data_type_or_uri: either the data type (for topic regs) or the service URI (for service regs).
        @type  data_type_or_uri: str
        @param reg_type: Valid values are L{Registration.PUB}, L{Registration.SUB}, L{Registration.SRV}
        @type  reg_type: str
        """
        master_uri = self.master_uri
        if not master_uri:
            self.logger.error("Registrar: master_uri is not set yet, cannot inform master of deregistration")
        else:
            try:
                master = xmlrpcapi(master_uri)
                if reg_type == Registration.PUB:
                    self.logger.debug("unregisterPublisher(%s, %s)", resolved_name, self.uri)
                    master.unregisterPublisher(get_caller_id(), resolved_name, self.uri)
                elif reg_type == Registration.SUB:            
                    self.logger.debug("unregisterSubscriber(%s, %s)", resolved_name, data_type_or_uri)
                    master.unregisterSubscriber(get_caller_id(), resolved_name, self.uri)
                elif reg_type == Registration.SRV:
                    self.logger.debug("unregisterService(%s, %s)", resolved_name, data_type_or_uri)
                    master.unregisterService(get_caller_id(), resolved_name, data_type_or_uri)
            except:
                logwarn("unable to communicate with ROS Master, registrations are now out of sync")
                self.logger.error(traceback.format_exc())
    
    def reg_added(self, resolved_name, data_type_or_uri, reg_type):
        """
        RegistrationListener callback
        @param resolved_name: resolved name of topic or service
        @type  resolved_name: str
        @param data_type_or_uri: either the data type (for topic regs) or the service URI (for service regs).
        @type  data_type_or_uri: str
        @param reg_type: Valid values are L{Registration.PUB}, L{Registration.SUB}, L{Registration.SRV}
        @type  reg_type: str
        """
        #TODO: this needs to be made robust to master outages
        master_uri = self.master_uri
        if not master_uri:
            self.logger.error("Registrar: master_uri is not set yet, cannot inform master of registration")
        else:
            master = xmlrpcapi(master_uri)
            args = (get_caller_id(), resolved_name, data_type_or_uri, self.uri)
            registered = False
            first = True
            while not registered and not is_shutdown():
                try:
                    if reg_type == Registration.PUB:
                        self.logger.debug("master.registerPublisher(%s, %s, %s, %s)"%args)
                        code, msg, val = master.registerPublisher(*args)
                        if code != 1:
                            logfatal("unable to register publication [%s] with master: %s"%(resolved_name, msg))
                    elif reg_type == Registration.SUB:
                        self.logger.debug("master.registerSubscriber(%s, %s, %s, %s)"%args)
                        code, msg, val = master.registerSubscriber(*args)
                        if code == 1:
                            self.publisher_update(resolved_name, val)
                        else:
                            # this is potentially worth exiting over. in the future may want to add a retry
                            # timer
                            logfatal("unable to register subscription [%s] with master: %s"%(resolved_name, msg))
                    elif reg_type == Registration.SRV:
                        self.logger.debug("master.registerService(%s, %s, %s, %s)"%args)
                        code, msg, val = master.registerService(*args)
                        if code != 1:
                            logfatal("unable to register service [%s] with master: %s"%(resolved_name, msg))
                        
                    registered = True
                except Exception as e:
                    if first:
                        msg = "Unable to register with master node [%s]: master may not be running yet. Will keep trying."%master_uri
                        self.logger.error(str(e)+"\n"+msg)
                        print(msg)
                        first = False
                    time.sleep(0.2)

    def publisher_update(self, resolved_name, uris):
        """
        Inform psmanager of latest publisher list for a topic.  This
        will cause L{RegManager} to create a topic connection for all new
        publishers (in a separate thread).
        @param resolved_name: resolved topic name
        @type  resolved_name: str
        @param uris: list of all publishers uris for topic
        @type  uris: [str]
        """
        try:
            self.cond.acquire()
            self.updates.append((resolved_name, uris))
            self.cond.notifyAll()              
        finally:
            self.cond.release()
