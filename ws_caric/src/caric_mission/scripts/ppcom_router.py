#!/usr/bin/env python

import sys
import numpy as np
from threading import Lock

import rospy
from rotors_comm.msg import PPComTopology
from importlib import import_module
from caric_mission.srv import CreatePPComTopic

import threading

import random
import string

# The topology to determine whether topic can be relayed
ppcomTopo = None
class PPComAccess:
    
    def __init__(self, topo):
        self.lock       = Lock()
        self.topo       = topo
        self.node_id    = topo.node_id
        self.node_alive = topo.node_alive
        self.Nnodes     = len(topo.node_id)
        self.adjacency  = self.rangeToLinkMat(self.topo.range, self.Nnodes)

    def rangeToLinkMat(self, distances, N):    
        linkMat = np.zeros([N, N])
        range_idx = 0
        for i in range(0, N):
            for j in range(i+1, N):
                linkMat[i, j] = distances[range_idx]
                linkMat[j, i] = linkMat[i, j]
                range_idx += 1
        return linkMat

    def update(self, topo):
        self.lock.acquire()
        self.node_alive = topo.node_alive
        self.adjacency  = self.rangeToLinkMat(topo.range, self.Nnodes)
        self.lock.release()

    def getAdj(self):
        self.lock.acquire()
        adj = self.adjacency
        self.lock.release()
        return adj
    
    def getSimpleAdj(self):
        self.lock.acquire()
        adj = self.adjacency
        self.lock.release()
        
        for i in range(0, adj.shape[0]):
            for j in range(0, adj.shape[1]):
                if adj[i][j] > 0.0:
                    adj[i][j] = 1
                else:
                    adj[i][j] = 0

        return adj


# Dictionary to relay data from one topic to others
topic_to_dialogue = {}
class Dialogue:
    
    def __init__(self, topic, sub, source, callerid):
        self.topic = topic
        self.sub = sub
        self.callerids_to_source = {callerid : source}
        self.target_to_pub = {}
        self.permitted_edges = set()

    def addTarget(self, target, pub):
        self.target_to_pub[target] = pub

    def addSource(self, source, callerid):
        self.callerids_to_source[callerid] = source

    def addPermittedEdge(self, edge):
        self.permitted_edges.add(edge)


# Mutex for the dialogue dictionary
dialogue_mutex = threading.Lock()


# Update the topology
def TopologyCallback(msg):

    # print(type(msg.node_alive), len(msg.node_alive))

    global ppcomTopo
    ppcomTopo.update(msg)
    
    # print("\nLOS:")
    # adj = ppcomTopo.getSimpleAdj()
    # for arr in adj:
    #     print(arr)


# Relay the message
def DataCallback(msg):
    
    if ppcomTopo == None:
        print("ppcom topo not set yet")

    conn_header = msg._connection_header
    
    topic = conn_header['topic']
    callerid = conn_header['callerid']

    try:
        source_node = topic_to_dialogue[topic].callerids_to_source[callerid]
    except:
        print("Missing key: ", callerid)
        print(topic_to_dialogue.keys())
        print(topic_to_dialogue[topic].callerids_to_source.keys())
        return

    # print(f"Msg from callerid {conn_header['callerid']}. Topic: {conn_header['topic']}. Size: {len(msg._buff)}")

    # if callerid not in topic_to_dialogue[topic].callerids_to_source.keys():
    #     return

    adjacency = ppcomTopo.getAdj()
    
    for target_node in ppcomTopo.node_id:

        # Skip if target node is the same as source node
        if target_node == source_node:
            continue

        i = ppcomTopo.node_id.index(source_node)
        j = ppcomTopo.node_id.index(target_node)

        if i >= len(ppcomTopo.node_alive) or j >= len(ppcomTopo.node_alive):
            continue

        # print(f"Checking index {i}, {j}. Nodes {len(ppcomTopo.node_alive)}. Buf: {len(ppcomTopo.topo.node_alive)}")
        # print(f"Checking node_alive[i] {ppcomTopo.node_alive[i]},\nnode_alive[j] {ppcomTopo.node_alive[j]}.")

        # Skip if either node is dead
        if not ppcomTopo.node_alive[i] or not ppcomTopo.node_alive[j]:
            continue

        # Skip if there is no line of sight between node
        if adjacency[i, j] <= 0.0:
            continue
        
        # If edege is not permitted, skip
        if (callerid, target_node) not in topic_to_dialogue[topic].permitted_edges:
            # print((callerid, target_node), "is not in pe: ", topic_to_dialogue[topic].permitted_edges)
            continue

        # if (i == 1 and j == 3) or (i == 3 and j == 1):
        #     print("adj_13: ", adjacency[i, j])
        
        # Publish the message on derived topics
        topic_to_dialogue[topic].target_to_pub[target_node].publish(msg)

    # print(f"Receive msg under {route[0]}. From {route[1]} to {route[2]}. CallerID: {conn_header['callerid']}")


# Create a topic over the ppcom network
def CreatePPComTopicCallback(req):
    
    global topic_to_dialogue

    try:
        
        callerid = req._connection_header['callerid']
        source   = req.source
        targets  = req.targets
        topic    = req.topic_name
        package  = req.package_name
        message  = req.message_type

        print(f"Receiving request from {callerid}. Source {source}. Target {targets}")

        msg_class = getattr(import_module(package + '.msg'), message)

        # Request access to the dialogue dict
        dialogue_mutex.acquire()

        # If topic has not been created, create it
        if topic not in topic_to_dialogue.keys():
            topic_to_dialogue[topic] = Dialogue(topic, rospy.Subscriber(topic, rospy.msg.AnyMsg, DataCallback), source, callerid)
        else:
            topic_to_dialogue[topic].addSource(source, callerid)

        # # If source under this topic has not been created, create it
        # if source not in topic_to_dialogue[topic].keys():
        #     topic_to_dialogue[topic][source] = {}

        # If 'all' is set in targets just set targets to all existing node
        if 'all' in targets[0]:
            targets = ppcomTopo.topo.node_id
            
        # For each target, create one publisher to the target
        for target in targets:

            if target == source:
                # print(f"Target {target} is in source {source}, skipping.")
                continue
            
            # Permit communication from this callerid to target
            topic_to_dialogue[topic].addPermittedEdge((callerid, target))

            # Skip if this publisher has been declared
            if target in topic_to_dialogue[topic].target_to_pub.keys():
                continue

            # Create a subscriber and publisher pair
            topic_to_dialogue[topic].target_to_pub[target] = rospy.Publisher(topic + '/' + target, msg_class, queue_size=1)

        dialogue_mutex.release()

        # Report success
        return 'success lah!'
    
    except Exception as e:

        # Report failure
        return 'fail liao! Error: ' + str(e)
        

if __name__ == '__main__':

    rospy.init_node('ppcom_router', anonymous=True)

    print("ppcom_router started!")

    # Wait for first ppcom message to arrive to initialize the network
    ppcomTopo = PPComAccess(rospy.wait_for_message("/ppcom_topology", PPComTopology))

    # Subscribe to the ppcom topo
    rospy.Subscriber("/ppcom_topology_doa", PPComTopology, TopologyCallback)
    
    # Advertise the ppcom topic creator
    rospy.Service('create_ppcom_topic', CreatePPComTopic, CreatePPComTopicCallback)

    # Go to sleep and let the callback works
    rospy.spin()
