#!/usr/bin/env python3
import sys
import random
import string

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rotors_comm_msgs.msg import PPComTopology
from caric_mission.srv import CreatePPComTopic

class SentosaTalker(Node):
    """Sentosa node talker for PPCom communication"""

    def __init__(self):
        super().__init__('sentosa_talker')
        
        # Initialize flags for tracking received messages
        self.received_flags = {
            'jurong': 0,
            'raffles': 0,
            'changi': 0,
            'nanyang': 0
        }
        
        # Wait for service to be available
        self.create_topic_client = self.create_client(CreatePPComTopic, 'create_ppcom_topic')
        
        self.get_logger().info("Waiting for create_ppcom_topic service...")
        while not self.create_topic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        
        # Register the topic with ppcom router
        self.register_topic()
        
        # Create publisher
        self.msg_pub = self.create_publisher(String, '/sentosa/ping_message', 10)
        
        # Create subscribers for other nodes
        self.jurong_sub = self.create_subscription(
            String,
            '/domain3/jurong/ping_message',
            lambda msg: self.ping_message_callback(msg, 'jurong'),
            10
        )
        
        self.raffles_sub = self.create_subscription(
            String,
            '/domain3/raffles/ping_message',
            lambda msg: self.ping_message_callback(msg, 'raffles'),
            10
        )
        
        self.changi_sub = self.create_subscription(
            String,
            '/domain3/changi/ping_message',
            lambda msg: self.ping_message_callback(msg, 'changi'),
            10
        )
        
        self.nanyang_sub = self.create_subscription(
            String,
            '/domain3/nanyang/ping_message',
            lambda msg: self.ping_message_callback(msg, 'nanyang'),
            10
        )
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Sentosa talker node started!")

    def register_topic(self):
        """Register the ping_message topic with PPCom router"""
        request = CreatePPComTopic.Request()
        request.source = 'sentosa'
        request.targets = ['raffles']
        request.topic_name = '/sentosa/ping_message'
        request.package_name = 'std_msgs'
        request.message_type = 'String'
        
        future = self.create_topic_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Registration response: {future.result().result}")
        else:
            self.get_logger().error("Failed to register topic")

    def ping_message_callback(self, msg, source):
        """Handle received ping messages and set corresponding flag"""
        if source in self.received_flags:
            self.received_flags[source] = 1

    def timer_callback(self):
        """Timer callback for periodic message publishing"""
        # Generate random text
        length = random.randint(0, 20)
        letters = string.ascii_lowercase
        result_str = ''.join(random.choice(letters) for i in range(length))
        
        # Create and publish message
        msg = String()
        msg.data = f"test;/sentosa;{result_str}"
        
        # Color codes
        BLUE = '\033[94m'
        GREEN = '\033[92m'
        RED = '\033[91m'
        RESET = '\033[0m'
        
        # Display sending status
        self.get_logger().info(f"{BLUE}*  SENDING  *{RESET}")
        self.msg_pub.publish(msg)
        
        # Check flags and build received status display
        received_nodes = []
        not_received_nodes = []
        
        for node_name, flag in self.received_flags.items():
            if flag == 1:
                received_nodes.append(f"{GREEN}'{node_name}'{RESET}")
            else:
                not_received_nodes.append(f"{RED}'{node_name}'{RESET}")
        
        # Combine all nodes for display
        all_nodes = received_nodes + not_received_nodes
        if all_nodes:
            nodes_str = ', '.join(all_nodes)
            self.get_logger().info(f"Received: {nodes_str}")
        
        # Reset all flags to 0
        for node_name in self.received_flags:
            self.received_flags[node_name] = 0

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    sentosa_talker = SentosaTalker()
    
    try:
        rclpy.spin(sentosa_talker)
    except KeyboardInterrupt:
        pass
    finally:
        sentosa_talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
