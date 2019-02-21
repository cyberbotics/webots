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

from __future__ import print_function

"""
Data structures and library for representing ROS Computation Graph state.
"""

import sys
import time
import itertools
import random
import logging
import traceback
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy
import socket

import rosgraph.masterapi

logger = logging.getLogger('rosgraph.graph')

_ROS_NAME = '/rosviz'

def topic_node(topic):
    """
    In order to prevent topic/node name aliasing, we have to remap
    topic node names. Currently we just prepend a space, which is
    an illegal ROS name and thus not aliased.
    @return str: topic mapped to a graph node name.
    """
    return ' ' + topic
def node_topic(node):
    """
    Inverse of topic_node
    @return str: undo topic_node() operation
    """
    return node[1:]

class BadNode(object):
    """
    Data structure for storing info about a 'bad' node
    """
    
    ## no connectivity
    DEAD = 0
    ## intermittent connectivity
    WONKY = 1
    
    def __init__(self, name, type, reason):
        """
        @param type: DEAD | WONKY
        @type  type: int
        """
        self.name =name
        self.reason = reason
        self.type = type

class EdgeList(object):
    """
    Data structure for storing Edge instances
    """
    __slots__ = ['edges_by_start', 'edges_by_end']
    def __init__(self):
        # in order to make it easy to purge edges, we double-index them
        self.edges_by_start = {}
        self.edges_by_end = {}        

    def __iter__(self):
        return itertools.chain(*[v for v in self.edges_by_start.values()])
    
    def has(self, edge):
        return edge in self

    def __contains__(self, edge):
        """
        @return: True if edge is in edge list
        @rtype: bool
        """
        key = edge.key
        return key in self.edges_by_start and \
               edge in self.edges_by_start[key]
        
    def add(self, edge):
        """
        Add an edge to our internal representation. not multi-thread safe
        @param edge: edge to add
        @type  edge: Edge
        """
        # see note in __init__
        def update_map(map, key, edge):
            if key in map:
                l = map[key]
                if not edge in l:
                    l.append(edge)
                    return True
                else:
                    return False
            else:
                map[key] = [edge]
                return True
            
        updated = update_map(self.edges_by_start, edge.key, edge)
        updated = update_map(self.edges_by_end, edge.rkey, edge) or updated
        return updated
        
    def add_edges(self, start, dest, direction, label=''):
        """
        Create Edge instances for args and add resulting edges to edge
        list. Convenience method to avoid repetitve logging, etc...
        @param edge_list: data structure to add edge to
        @type  edge_list: EdgeList
        @param start: name of start node. If None, warning will be logged and add fails
        @type  start: str
        @param dest: name of start node. If None, warning will be logged and add fails
        @type  dest: str
        @param direction: direction string (i/o/b)
        @type  direction: str
        @return: True if update occured
        @rtype: bool
        """
        
        # the warnings should generally be temporary, occuring of the
        # master/node information becomes stale while we are still
        # doing an update
        updated = False        
        if not start:
            logger.warn("cannot add edge: cannot map start [%s] to a node name", start)
        elif not dest:
            logger.warn("cannot add edge: cannot map dest [%s] to a node name", dest)
        else:
            for args in edge_args(start, dest, direction, label):
                updated = self.add(Edge(*args)) or updated
        return updated

    def delete_all(self, node):
        """
        Delete all edges that start or end at node
        @param node: name of node
        @type  node: str
        """
        def matching(map, pref):
            return [map[k] for k in map.keys() if k.startswith(pref)]
        
        pref = node+"|"
        edge_lists = matching(self.edges_by_start, pref) + matching(self.edges_by_end, pref)
        for el in edge_lists:
            for e in el:
                self.delete(e)
        
    def delete(self, edge):
        # see note in __init__
        def update_map(map, key, edge):
            if key in map:
                edges = map[key]
                if edge in edges:
                    edges.remove(edge)
                    return True
        update_map(self.edges_by_start, edge.key, edge)
        update_map(self.edges_by_end, edge.rkey, edge)
    
class Edge(object):
    """
    Data structure for representing ROS node graph edge
    """
    
    __slots__ = ['start', 'end', 'label', 'key', 'rkey']
    def __init__(self, start, end, label=''):
        self.start = start
        self.end = end
        self.label = label
        self.key = "%s|%s"%(self.start, self.label)
        # reverse key, indexed from end
        self.rkey = "%s|%s"%(self.end, self.label)        

    def __ne__(self, other):
        return self.start != other.start or self.end != other.end
    def __str__(self):
        return "%s->%s"%(self.start, self.end)
    def __eq__(self, other):
        return self.start == other.start and self.end == other.end

def edge_args(start, dest, direction, label):
    """
    compute argument ordering for Edge constructor based on direction flag
    @param direction str: 'i', 'o', or 'b' (in/out/bidir) relative to \a start
    @param start str: name of starting node
    @param start dest: name of destination node 
    """
    edge_args = []
    if direction in ['o', 'b']:
        edge_args.append((start, dest, label))
    if direction in ['i', 'b']:
        edge_args.append((dest, start, label))
    return edge_args


class Graph(object):
    """
    Utility class for polling ROS statistics from running ROS graph.
    Not multi-thread-safe    
    """
    
    def __init__(self, node_ns='/', topic_ns='/'):
        self.master = rosgraph.masterapi.Master(_ROS_NAME)

        self.node_ns = node_ns or '/'
        self.topic_ns = topic_ns or '/'

        # ROS nodes
        self.nn_nodes = set([])
        # ROS topic nodes
        self.nt_nodes = set([])

        # ROS nodes that aren't responding quickly
        self.bad_nodes = {}
        import threading
        self.bad_nodes_lock = threading.Lock()
        
        # ROS services
        self.srvs = set([])
        # ROS node->node transport connections
        self.nn_edges = EdgeList()
        # ROS node->topic connections        
        self.nt_edges = EdgeList()        
        # ROS all node->topic connections, including empty
        self.nt_all_edges = EdgeList()        

        # node names to URI map
        self.node_uri_map = {} # { node_name_str : uri_str }
        # reverse map URIs to node names
        self.uri_node_map = {} # { uri_str : node_name_str }

        # time we last contacted master
        self.last_master_refresh = 0
        self.last_node_refresh = {}
        
        # time we last communicated with master
        # seconds until master data is considered stale
        self.master_stale = 5.0 
        # time we last communicated with node
        # seconds until node data is considered stale
        self.node_stale = 5.0 #seconds


    def set_master_stale(self, stale_secs):
        """
        @param stale_secs: seconds that data is considered fresh
        @type  stale_secs: double
        """
        self.master_stale = stale_secs
        
    def set_node_stale(self, stale_secs):
        """
        @param stale_secs: seconds that data is considered fresh
        @type  stale_secs: double
        """
        self.node_stale = stale_secs

    def _master_refresh(self):
        """
        @return: True if nodes information was updated
        @rtype: bool
        """
        logger.debug("master refresh: starting")
        updated = False
        try:
            val = self.master.getSystemState()
        except rosgraph.masterapi.MasterException as e:
            print("Unable to contact master", str(e), file=sys.stderr)
            logger.error("unable to contact master: %s", str(e))
            return False

        pubs, subs, srvs = val

        nodes = []
        nt_all_edges = self.nt_all_edges
        nt_nodes = self.nt_nodes
        for state, direction in ((pubs, 'o'), (subs, 'i')):
            for topic, l in state:
                if topic.startswith(self.topic_ns):
                    nodes.extend([n for n in l if n.startswith(self.node_ns)])
                    nt_nodes.add(topic_node(topic))
                    for node in l:
                        updated = nt_all_edges.add_edges(
                            node, topic_node(topic), direction) or updated

        nodes = set(nodes)

        srvs = set([s for s, _ in srvs])
        purge = None
        if nodes ^ self.nn_nodes:
            purge = self.nn_nodes - nodes
            self.nn_nodes = nodes
            updated = True
        if srvs ^ self.srvs:
            self.srvs = srvs
            updated = True

        if purge:
            logger.debug("following nodes and related edges will be purged: %s", ','.join(purge))
            for p in purge:
                logger.debug('purging edges for node %s', p)
                self.nn_edges.delete_all(p)
                self.nt_edges.delete_all(p)
                self.nt_all_edges.delete_all(p)                    
            
        logger.debug("master refresh: done, updated[%s]", updated)
        return updated
        
    def _mark_bad_node(self, node, reason):
        try:
            # bad nodes are updated in a separate thread, so lock
            self.bad_nodes_lock.acquire()
            if node in self.bad_nodes:
                self.bad_nodes[node].type = BadNode.DEAD
            else:
                self.bad_nodes[node] = (BadNode(node, BadNode.DEAD, reason))                
        finally:
            self.bad_nodes_lock.release()

    def _unmark_bad_node(self, node, reason):
        """
        Promotes bad node to 'wonky' status.
        """
        try:
            # bad nodes are updated in a separate thread, so lock
            self.bad_nodes_lock.acquire()
            bad = self.bad_nodes[node]
            bad.type = BadNode.WONKY
        finally:
            self.bad_nodes_lock.release()        
    
    def _node_refresh_businfo(self, node, api, bad_node=False):
        """
        Retrieve bus info from the node and update nodes and edges as appropriate
        @param node: node name
        @type  node: str
        @param api: XML-RPC proxy
        @type  api: ServerProxy
        @param bad_node: If True, node has connectivity issues and
        should be treated differently
        @type  bad_node: bool
        """
        try:
            logger.debug("businfo: contacting node [%s] for bus info", node)
            
            # unmark bad node, though it stays on the bad list
            if bad_node:
                self._unmark_bad_node(node)            
            # Lower the socket timeout as we cannot abide by slow HTTP timeouts.
            # If a node cannot meet this timeout, it goes on the bad list
            # TODO: override transport instead.
            old_timeout = socket.getdefaulttimeout()
            if bad_node:
                #even stricter timeout for bad_nodes right now
                socket.setdefaulttimeout(0.2)
            else:
                socket.setdefaulttimeout(1.0)
                
            code, msg, bus_info = api.getBusInfo(_ROS_NAME)
            
            socket.setdefaulttimeout(old_timeout)
        except Exception as e:
            # node is (still) bad
            self._mark_bad_node(node, str(e))
            code = -1
            msg = traceback.format_exc()

        updated = False
        if code != 1:
            logger.error("cannot get stats info from node [%s]: %s", node, msg)
        else:
            # [[connectionId1, destinationId1, direction1, transport1, ...]... ]
            for info in bus_info:
                # #3579 bad node, ignore
                if len(info) < 5:
                    continue
                
                connection_id = info[0]
                dest_id       = info[1]
                direction     = info[2]
                transport     = info[3]
                topic         = info[4]
                if len(info) > 5:
                    connected = info[5]
                else:
                    connected = True #backwards compatibility

                if connected and topic.startswith(self.topic_ns):
                    # blindly add as we will be able to catch state change via edges.
                    # this currently means we don't cleanup topics
                    self.nt_nodes.add(topic_node(topic))

                    # update node->topic->node graph edges
                    updated = self.nt_edges.add_edges(node, topic_node(topic), direction) or updated

                    # update node->node graph edges
                    if dest_id.startswith('http://'):
                        #print("FOUND URI", dest_id)
                        dest_name = self.uri_node_map.get(dest_id, None)
                        updated = self.nn_edges.add_edges(node, dest_name, direction, topic) or updated
                else:
                    #TODO: anyting to do here?
                    pass
        return updated
                    
    def _node_refresh(self, node, bad_node=False):
        """
        Contact node for stats/connectivity information
        @param node: name of node to contact
        @type  node: str
        @param bad_node: if True, node has connectivity issues
        @type  bad_node: bool
        @return: True if node was successfully contacted
        @rtype  bool
        """
        # TODO: I'd like for master to provide this information in
        # getSystemState() instead to prevent the extra connection per node
        updated = False
        uri = self._node_uri_refresh(node)
        try:
            if uri:
                api = ServerProxy(uri)
                updated = self._node_refresh_businfo(node, api, bad_node)
        except KeyError as e:
            logger.warn('cannot contact node [%s] as it is not in the lookup table'%node)
        return updated

    def _node_uri_refresh(self, node):
        try:
            uri = self.master.lookupNode(node)
        except:
            msg = traceback.format_exc()
            logger.warn("master reported error in node lookup: %s"%msg)
            return None
        # update maps
        self.node_uri_map[node] = uri
        self.uri_node_map[uri] = node
        return uri
    
    def _node_uri_refresh_all(self):
        """
        Build self.node_uri_map and self.uri_node_map using master as a
        lookup service. This will make N requests to the master for N
        nodes, so this should only be used sparingly
        """
        for node in self.nn_nodes:
            self._node_uri_refresh(node)

    def bad_update(self):
        """
        Update loop for nodes with bad connectivity. We box them separately
        so that we can maintain the good performance of the normal update loop.
        Once a node is on the bad list it stays there.
        """
        last_node_refresh = self.last_node_refresh

        # nodes left to check
        try:
            self.bad_nodes_lock.acquire()
            # make copy due to multithreading
            update_queue = self.bad_nodes.values()[:]
        finally:
            self.bad_nodes_lock.release()

        # return value. True if new data differs from old
        updated = False
        # number of nodes we checked
        num_nodes = 0

        start_time = time.time()
        while update_queue:
            # figure out the next node to contact
            next = update_queue.pop()
            # rate limit talking to any particular node
            if time.time() > (last_node_refresh.get(next, 0.0) + self.node_stale):
                updated = self._node_refresh(next.name, True) or updated
                # include in random offset (max 1/5th normal update interval)
                # to help spread out updates
                last_node_refresh[next] = time.time() + (random.random() * self.node_stale / 5.0)
                num_nodes += 1

            # small yield to keep from torquing the processor
            time.sleep(0.01)
        end_time = time.time()
        #print("Update (bad nodes) took %ss for %s nodes"%((end_time-start_time), num_nodes))
        logger.debug("ROS stats (bad nodes) update took %ss"%(end_time-start_time))
        return updated
            
    def update(self):
        """
        Update all the stats. This method may take awhile to complete as it will
        communicate with all nodes + master.
        """
        
        last_node_refresh = self.last_node_refresh
        
        # nodes left to check
        update_queue = None
        # True if there are still more stats to fetch this cycle
        work_to_do = True
        # return value. True if new data differs from old
        updated = False
        # number of nodes we checked
        num_nodes = 0

        start_time = time.time()
        while work_to_do:

            # each time through the loop try to talk to either the master
            # or a node. stop when we have talked to everybody.

            # get a new node list from the master
            if time.time() > (self.last_master_refresh + self.master_stale):
                updated = self._master_refresh()
                if self.last_master_refresh == 0:
                    # first time we contact the master, also refresh our full lookup tables
                    self._node_uri_refresh_all()

                self.last_master_refresh = time.time()
            # contact the nodes for stats
            else:
                # initialize update_queue based on most current nodes list
                if update_queue is None:
                    update_queue = list(self.nn_nodes)
                # no nodes left to contact
                elif not update_queue:
                    work_to_do = False
                # contact next node
                else:
                    # figure out the next node to contact
                    next = update_queue.pop()
                    # rate limit talking to any particular node
                    if time.time() > (last_node_refresh.get(next, 0.0) + self.node_stale):
                        updated = self._node_refresh(next) or updated
                        # include in random offset (max 1/5th normal update interval)
                        # to help spread out updates
                        last_node_refresh[next] = time.time() + (random.random() * self.node_stale / 5.0)
                        num_nodes += 1

            # small yield to keep from torquing the processor
            time.sleep(0.01)
        end_time = time.time()
        #print("Update took %ss for %s nodes"%((end_time-start_time), num_nodes))
        logger.debug("ROS stats update took %ss"%(end_time-start_time))
        return updated
    
