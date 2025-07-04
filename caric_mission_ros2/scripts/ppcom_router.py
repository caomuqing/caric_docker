#!/usr/bin/env python3
# Router for accept original UAV messages and relay them according to the topology.

import sys
import numpy as np
from threading import Lock
import threading
import importlib
import random
import string

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rotors_comm_msgs.msg import PPComTopology
from caric_mission.srv import CreatePPComTopic

# Target to domain mapping - define your mapping here
TARGET_TO_DOMAIN = {
    'gcs': 99,
    'jurong': 1,
    'raffles': 2,
    'sentosa': 3,
    'changi': 4,
    'nanyang': 5,
    # Add more mappings as needed
}


class PPComAccess:
    """Class to handle PPCom topology access and line-of-sight calculations"""
    
    def __init__(self, topo):
        self.lock = Lock()
        self.topo = topo
        self.node_id = topo.node_id
        self.node_alive = topo.node_alive
        self.Nnodes = len(topo.node_id)
        self.adjacency = self.range_to_link_mat(topo.range, self.Nnodes)

    def range_to_link_mat(self, distances, N):
        """Convert range array to adjacency matrix"""
        link_mat = np.zeros([N, N])
        range_idx = 0
        for i in range(0, N):
            for j in range(i+1, N):
                link_mat[i, j] = distances[range_idx]
                link_mat[j, i] = link_mat[i, j]
                range_idx += 1
        return link_mat

    def update(self, topo):
        """Update topology with new data"""
        self.lock.acquire()
        self.node_alive = topo.node_alive
        self.adjacency = self.range_to_link_mat(topo.range, self.Nnodes)
        self.lock.release()

    def get_adj(self):
        """Get current adjacency matrix"""
        self.lock.acquire()
        adj = self.adjacency.copy()
        self.lock.release()
        return adj
    
    def get_simple_adj(self):
        """Get binary adjacency matrix (0 or 1)"""
        self.lock.acquire()
        adj = self.adjacency.copy()
        self.lock.release()
        
        for i in range(0, adj.shape[0]):
            for j in range(0, adj.shape[1]):
                if adj[i][j] > 0.0:
                    adj[i][j] = 1
                else:
                    adj[i][j] = 0
        return adj


class Dialogue:
    """Class to manage communication dialogue for a specific topic"""
    
    def __init__(self, topic):
        self.topic = topic
        self.target_to_pub = {}
        self.permitted_edges = set()

    def add_target(self, target, pub):
        """Add a target node with its publisher"""
        self.target_to_pub[target] = pub

    def add_permitted_edge(self, edge):
        """Add a permitted communication edge"""
        self.permitted_edges.add(edge)


class PPComRouter(Node):
    """Main PPCom Router Node for ROS2"""
    
    def __init__(self):
        super().__init__('ppcom_router')
        
        # Initialize class variables
        self.ppcom_topo = None
        self.topic_to_dialogue = {}
        self.dialogue_mutex = threading.Lock()
        
        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscription to topology updates
        self.topology_sub = self.create_subscription(
            PPComTopology,
            '/ppcom_topology_doa',
            self.topology_callback,
            qos_profile
        )
        
        # Create service for creating PPCom topics
        self.create_topic_service = self.create_service(
            CreatePPComTopic,
            'create_ppcom_topic',
            self.create_ppcom_topic_callback
        )
        
        # Wait for initial topology message
        self.get_logger().info("PPCom Router started! Waiting for initial topology...")
        self.wait_for_initial_topology()
        
        self.get_logger().info("PPCom Router initialized successfully!")

    def wait_for_initial_topology(self):
        """Wait for the first topology message to initialize the network"""
        # Create a temporary subscription to get the first message
        temp_sub = self.create_subscription(
            PPComTopology,
            '/ppcom_topology',
            self.initial_topology_callback,
            10
        )
        
        # Spin until we get the first topology message
        while self.ppcom_topo is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Destroy the temporary subscription
        self.destroy_subscription(temp_sub)

    def initial_topology_callback(self, msg):
        """Handle the first topology message"""
        if self.ppcom_topo is None:
            self.ppcom_topo = PPComAccess(msg)
            self.get_logger().info(f"Initial topology received with {len(msg.node_id)} nodes")

    def topology_callback(self, msg):
        """Handle topology updates"""
        if self.ppcom_topo is not None:
            self.ppcom_topo.update(msg)

    def parse_message_sender(self, msg, topic_name):
        try:
            # Assumes the structure is always /<sender>/...
            source_node = topic_name.split('/')[1]
            return source_node
        except IndexError:
            # Handle cases where the topic_name might not have the expected structure
            print(f"Warning: Could not parse sender from topic '{topic_name}'.")
            return None

    def data_callback(self, msg, topic_name):
        """Handle data messages and relay them according to topology"""
        if self.ppcom_topo is None:
            self.get_logger().warn("PPCom topology not set yet")
            return

        # Parse the message to get the actual sender
        source_node = self.parse_message_sender(msg, topic_name)
        if source_node is None:
            self.get_logger().warn("Could not determine message sender")
            return

        adjacency = self.ppcom_topo.get_adj()
        
        # Relay message to all permitted targets with line of sight
        for target_node in self.ppcom_topo.node_id:
            # Skip if target node is the same as source node
            if target_node == source_node:
                continue

            try:
                i = self.ppcom_topo.node_id.index(source_node)
                j = self.ppcom_topo.node_id.index(target_node)
            except ValueError:
                continue

            if i >= len(self.ppcom_topo.node_alive) or j >= len(self.ppcom_topo.node_alive):
                continue

            # Skip if either node is dead
            if not self.ppcom_topo.node_alive[i] or not self.ppcom_topo.node_alive[j]:
                continue

            # Skip if there is no line of sight between nodes
            if adjacency[i, j] <= 0.0:
                continue
            
            # Check if edge is permitted
            if (source_node, target_node) not in self.topic_to_dialogue[topic_name].permitted_edges:
                continue
            
            # Publish the message to the target
            if target_node in self.topic_to_dialogue[topic_name].target_to_pub:
                self.topic_to_dialogue[topic_name].target_to_pub[target_node].publish(msg)

    def create_ppcom_topic_callback(self, request, response):
        """Handle requests to create new PPCom topics"""
        try:
            source = request.source
            targets = request.targets
            topic = request.topic_name
            package = request.package_name
            message = request.message_type

            self.get_logger().info(f"Creating PPCom topic: {topic} from {source} to {targets}")

            # Import the message type (Like ROS1 AnyMsg)
            try:
                msg_module = importlib.import_module(f'{package}.msg')
                msg_class = getattr(msg_module, message)
            except (ImportError, AttributeError) as e:
                self.get_logger().error(f"Failed to import message type {package}.msg.{message}: {e}")
                response.result = f"Failed to import message type: {e}"
                return response

            # Request access to the dialogue dict
            self.dialogue_mutex.acquire()

            try:
                # If topic has not been created, create it
                if topic not in self.topic_to_dialogue:
                    self.topic_to_dialogue[topic] = Dialogue(topic)

                # If 'all' is set in targets, set targets to all existing nodes
                if targets and 'all' in targets[0]:
                    targets = self.ppcom_topo.node_id[:]

                # For each target, create one publisher to the target
                for target in targets:
                    if target == source:
                        continue
                    
                    # Permit communication from source to target
                    self.topic_to_dialogue[topic].add_permitted_edge((source, target))

                    # Skip if this publisher has already been declared
                    if target in self.topic_to_dialogue[topic].target_to_pub:
                        continue

                    # Get domain number for target
                    domain_num = TARGET_TO_DOMAIN.get(target, 0)  # Default to 0 if target not found
                    
                    # Create publisher for this target with domain format
                    pub = self.create_publisher(
                        msg_class,
                        f"/domain{domain_num}{topic}",
                        10
                    )
                    self.topic_to_dialogue[topic].add_target(target, pub)

                # Create subscription for the main topic if not already created
                if not hasattr(self.topic_to_dialogue[topic], 'subscription'):
                    sub = self.create_subscription(
                        msg_class,
                        topic,
                        lambda msg, tpc=topic: self.data_callback(msg, tpc),
                        10
                    )
                    self.topic_to_dialogue[topic].subscription = sub

                response.result = 'success'
                self.get_logger().info(f"Successfully created PPCom topic: {topic}")

            finally:
                self.dialogue_mutex.release()

        except Exception as e:
            self.get_logger().error(f"Error creating PPCom topic: {e}")
            response.result = f'failed: {str(e)}'

        return response


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    ppcom_router = PPComRouter()
    
    try:
        rclpy.spin(ppcom_router)
    except KeyboardInterrupt:
        pass
    finally:
        ppcom_router.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()