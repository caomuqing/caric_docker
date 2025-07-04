#!/usr/bin/env python3
# Call to create all original topics for drone swarm communication

import rclpy
from rclpy.node import Node
from rclpy.context import Context
import time
import threading
from std_msgs.msg import String
from caric_mission.srv import CreatePPComTopic

# =============================================================================
# Configuration
# =============================================================================

# Drone nodes (excluding GCS)
DRONE_NODES = ["jurong", "raffles", "changi", "sentosa", "nanyang"]

# GCS node
GCS_NODE = "gcs"

# All nodes for reference
ALL_NODES = [GCS_NODE] + DRONE_NODES

# Thread startup delay (seconds) - prevents service call conflicts
STARTUP_DELAY = 0.3

# PPCom service configuration
SERVICE_NAME = '/create_ppcom_topic'
SUCCESS_RESPONSE = "success"
SERVICE_TIMEOUT = 15.0

# GCS topic configuration - only one topic needed
GCS_TOPIC_CONFIG = {
    "topic_name": "/gcs/bounding_box_vertices",
    "package_name": "sensor_msgs", 
    "message_type": "PointCloud"
}

# Drone message configurations - comprehensive sensor data topics
DRONE_TOPICS_CONFIG = [
    {
        "topic_suffix": "/ground_truth/odometry",
        "package_name": "nav_msgs",
        "message_type": "Odometry",
        "description": "Ground truth odometry data"
    },
    {
        "topic_suffix": "/nbr_odom_cloud",
        "package_name": "sensor_msgs",
        "message_type": "PointCloud2",
        "description": "Neighbor odometry point cloud"
    },
    {
        "topic_suffix": "/cloud_inW", 
        "package_name": "sensor_msgs",
        "message_type": "PointCloud2",
        "description": "Point cloud in world frame"
    },
    {
        "topic_suffix": "/slf_kf_cloud",
        "package_name": "sensor_msgs", 
        "message_type": "PointCloud2",
        "description": "Self keyframe point cloud"
    },
    {
        "topic_suffix": "/nbr_kf_cloud",
        "package_name": "sensor_msgs",
        "message_type": "PointCloud2",
        "description": "Neighbor keyframe point cloud"
    },
    {
        "topic_suffix": "/detected_interest_points",
        "package_name": "sensor_msgs",
        "message_type": "PointCloud2",
        "description": "Detected points of interest"
    },
    {
        "topic_suffix": "/gimbal",
        "package_name": "geometry_msgs",
        "message_type": "TwistStamped",
        "description": "Gimbal control commands"
    }
]

# =============================================================================
# Enhanced Service Client Base Class
# =============================================================================

class PPComServiceClient(Node):
    """Enhanced PPCom service client with robust error handling and logging"""
    
    def __init__(self, node_name: str, context=None):
        super().__init__(f"{node_name}_ppcom_client", context=context)
        self.node_name = node_name
        self.client = None
        self.service_call_count = 0
        self.success_count = 0
        self._executor = None
        self.setup_service_client()
        
    def setup_service_client(self):
        """Initialize the PPCom service client"""
        self.client = self.create_client(CreatePPComTopic, SERVICE_NAME)
        self.get_logger().info(f"Service client initialized for {self.node_name}")
        
    def call_ppcom_service(self, source_name: str, targets: list, topic_name: str, 
                          package_name: str, message_type: str, description: str = ""):
        """
        Call PPCom service with comprehensive error handling and logging
        
        Args:
            source_name: Name of the source node
            targets: List of target nodes (or ["all"])
            topic_name: Full topic name including namespace
            package_name: ROS package containing the message type
            message_type: ROS message type name
            description: Optional description for logging
        
        Returns:
            bool: True if service call succeeded, False otherwise
        """
        self.service_call_count += 1
        
        # Wait for service availability with timeout
        self.get_logger().info(f"[WAITING] PPCom service for {source_name}...")
        if not self.client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error(f"[FAILED] PPCom service unavailable for {source_name} after {SERVICE_TIMEOUT}s timeout")
            return False
        
        # Prepare service request
        request = CreatePPComTopic.Request()
        request.source = source_name
        request.targets = targets
        request.topic_name = topic_name
        request.package_name = package_name
        request.message_type = message_type
        
        # Log the service call details
        targets_str = ", ".join(targets) if len(targets) <= 3 else f"{', '.join(targets[:3])}... ({len(targets)} total)"
        self.get_logger().info(f"[CALLING] PPCom service: {source_name} -> [{targets_str}] | {topic_name}")
        if description:
            self.get_logger().info(f"[INFO] {description}")
        
        # Execute service call with proper executor management
        try:
            # Create executor if not exists
            if self._executor is None:
                from rclpy.executors import SingleThreadedExecutor
                self._executor = SingleThreadedExecutor(context=self.context)
                self._executor.add_node(self)
            
            future = self.client.call_async(request)
            
            # Use executor to spin until future completes
            import time
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < SERVICE_TIMEOUT:
                self._executor.spin_once(timeout_sec=0.1)
            
            if future.done() and future.result() and future.result().result == SUCCESS_RESPONSE:
                self.success_count += 1
                self.get_logger().info(f"[SUCCESS] PPCom service: {topic_name} ({package_name}/{message_type})")
                return True
            else:
                response_result = future.result().result if future.done() and future.result() else "No response/timeout"
                self.get_logger().error(f"[FAILED] PPCom service: {topic_name}")
                self.get_logger().error(f"[ERROR] Response: '{response_result}'")
                return False
                
        except Exception as e:
            self.get_logger().error(f"[EXCEPTION] Service call error for {source_name}: {str(e)}")
            return False

    def cleanup_executor(self):
        """Clean up the executor properly"""
        if self._executor is not None:
            try:
                self._executor.remove_node(self)
                self._executor.shutdown()
            except Exception as e:
                self.get_logger().warning(f"Error during executor cleanup: {e}")
            finally:
                self._executor = None

    def get_stats(self):
        """Get service call statistics"""
        return self.success_count, self.service_call_count


# =============================================================================
# Specialized Client Classes
# =============================================================================

class GCSPPComClient(PPComServiceClient):
    """GCS PPCom client - handles ground control station communication services"""
    
    def __init__(self, context=None):
        super().__init__("gcs", context=context)
        
    def create_gcs_services(self):
        """Create GCS-specific PPCom services"""
        self.get_logger().info("[START] GCS PPCom service creation...")
        
        # Create the single GCS bounding box vertices service
        success = self.call_ppcom_service(
            source_name="gcs",
            targets=["all"],  # Broadcast to all nodes
            topic_name=GCS_TOPIC_CONFIG["topic_name"],
            package_name=GCS_TOPIC_CONFIG["package_name"],
            message_type=GCS_TOPIC_CONFIG["message_type"],
            description="GCS bounding box vertices for mission area definition"
        )
        
        success_count, total_count = self.get_stats()
        self.get_logger().info(f"[COMPLETE] GCS service creation: {success_count}/{total_count} successful")
        
        return success_count, total_count


class DronePPComClient(PPComServiceClient):
    """Drone PPCom client - handles individual drone sensor and control communication"""
    
    def __init__(self, drone_name: str, context=None):
        super().__init__(drone_name, context=context)
        self.drone_name = drone_name
        
    def create_drone_services(self):
        """Create all configured PPCom services for this drone"""
        self.get_logger().info(f"[START] {self.drone_name} PPCom service creation...")
        
        # Create services for all configured drone topics
        for i, topic_config in enumerate(DRONE_TOPICS_CONFIG, 1):
            topic_name = f"/{self.drone_name}{topic_config['topic_suffix']}"
            
            self.get_logger().info(f"[PROGRESS] [{i}/{len(DRONE_TOPICS_CONFIG)}] Creating service for {topic_config['topic_suffix']}")
            
            self.call_ppcom_service(
                source_name=self.drone_name,
                targets=["all"],  # Broadcast to all nodes - modify as needed
                topic_name=topic_name,
                package_name=topic_config['package_name'],
                message_type=topic_config['message_type'],
                description=topic_config['description']
            )
        
        success_count, total_count = self.get_stats()
        self.get_logger().info(f"[COMPLETE] {self.drone_name} service creation: {success_count}/{total_count} successful")
        
        return success_count, total_count


# =============================================================================
# Thread Execution Functions
# =============================================================================

def run_gcs_thread():
    """Execute GCS PPCom service creation in dedicated thread with separate ROS context"""
    context = None
    client = None
    try:
        print(f"[THREAD] GCS thread started - initializing ROS context")
        
        # Create separate ROS context for this thread
        context = Context()
        rclpy.init(context=context)
        
        # Create client with the thread-specific context
        client = GCSPPComClient(context=context)
        
        success_count, total_count = client.create_gcs_services()
        
        status = "SUCCESS" if success_count == total_count else f"PARTIAL ({success_count}/{total_count})"
        print(f"[THREAD] GCS thread completed: {status}")
        return success_count == total_count
        
    except Exception as e:
        print(f"[ERROR] GCS thread error: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Clean up thread-specific resources
        if client:
            try:
                client.cleanup_executor()
                client.destroy_node()
            except Exception as e:
                print(f"[WARNING] Error during GCS client cleanup: {e}")
        if context:
            try:
                rclpy.shutdown(context=context)
                print(f"[THREAD] GCS thread - ROS context shutdown complete")
            except Exception as e:
                print(f"[WARNING] Error during GCS context shutdown: {e}")


def run_drone_thread(drone_name: str):
    """Execute drone PPCom service creation in dedicated thread with separate ROS context"""
    context = None
    client = None
    try:
        print(f"[THREAD] {drone_name} thread started - initializing ROS context")
        
        # Create separate ROS context for this thread
        context = Context()
        rclpy.init(context=context)
        
        # Create client with the thread-specific context
        client = DronePPComClient(drone_name, context=context)
        
        success_count, total_count = client.create_drone_services()
        
        status = "SUCCESS" if success_count == total_count else f"PARTIAL ({success_count}/{total_count})"
        print(f"[THREAD] {drone_name} thread completed: {status}")
        return success_count == total_count
        
    except Exception as e:
        print(f"[ERROR] {drone_name} thread error: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Clean up thread-specific resources
        if client:
            try:
                client.cleanup_executor()
                client.destroy_node()
            except Exception as e:
                print(f"[WARNING] Error during {drone_name} client cleanup: {e}")
        if context:
            try:
                rclpy.shutdown(context=context)
                print(f"[THREAD] {drone_name} thread - ROS context shutdown complete")
            except Exception as e:
                print(f"[WARNING] Error during {drone_name} context shutdown: {e}")


# =============================================================================
# Main Execution Controller
# =============================================================================

def create_all_ppcom_services():
    """
    Orchestrate the creation of all PPCom services for the entire drone swarm
    
    This function manages the parallel execution of service creation using
    separate ROS 2 contexts to prevent resource conflicts:
    - 1 GCS node with bounding box vertices topic
    - N drone nodes each with 7 sensor/control topics
    """
    
    print("=" * 70)
    print("DRONE SWARM PPCOM SERVICE INITIALIZATION")
    print("=" * 70)
    print(f"Configuration Summary:")
    print(f"   GCS node: {GCS_NODE} (1 topic)")
    print(f"   Drone nodes: {DRONE_NODES} ({len(DRONE_TOPICS_CONFIG)} topics each)")
    print(f"   Total services to create: {1 + len(DRONE_NODES) * len(DRONE_TOPICS_CONFIG)}")
    print(f"   Thread startup delay: {STARTUP_DELAY}s")
    print(f"   Using separate ROS contexts for optimal parallelization")
    print("")
    
    # Note: No global ROS initialization - each thread manages its own context
    
    threads = []
    results = []
    
    try:
        # 1. Launch GCS service creation thread
        print("[1] Launching GCS PPCom service creation thread...")
        gcs_thread = threading.Thread(
            target=lambda: results.append(('gcs', run_gcs_thread())),
            name="ppcom_gcs_thread",
            daemon=True
        )
        gcs_thread.start()
        threads.append(gcs_thread)
        
        time.sleep(STARTUP_DELAY)
        
        # 2. Launch drone service creation threads
        for i, drone_name in enumerate(DRONE_NODES):
            print(f"[{i+2}] Launching {drone_name} PPCom service creation thread...")
            
            drone_thread = threading.Thread(
                target=lambda dn=drone_name: results.append((dn, run_drone_thread(dn))),
                name=f"ppcom_{drone_name}_thread",
                daemon=True
            )
            drone_thread.start()
            threads.append(drone_thread)
            
            # Stagger thread launches to prevent service overload
            if i < len(DRONE_NODES) - 1:
                time.sleep(STARTUP_DELAY)
        
        print(f"\n[WAITING] All {len(threads)} PPCom service creation threads launched. Waiting for completion...")
        
        # Wait for all threads to complete their work
        for i, thread in enumerate(threads, 1):
            print(f"[WAITING] [{i}/{len(threads)}] Waiting for {thread.name}...")
            thread.join()
            
        # Generate comprehensive results summary
        print("\n" + "=" * 70)
        print("PPCOM SERVICE CREATION RESULTS SUMMARY")
        print("=" * 70)
        
        total_success = 0
        total_nodes = len(results)
        
        for node_name, success in results:
            if node_name == 'gcs':
                services_count = 1
                status = "SUCCESS" if success else "FAILED"
                print(f"[{status}] {node_name.upper():>8}: {status} (1 service)")
            else:
                services_count = len(DRONE_TOPICS_CONFIG)
                status = "SUCCESS" if success else "FAILED"
                print(f"[{status}] {node_name:>8}: {status} ({services_count} services)")
            
            if success:
                total_success += 1
                
        print("-" * 70)
        success_rate = (total_success / total_nodes * 100) if total_nodes > 0 else 0
        print(f"OVERALL RESULT: {total_success}/{total_nodes} nodes successful ({success_rate:.1f}%)")
        
        if total_success == total_nodes:
            print("ALL PPCOM SERVICES CREATED SUCCESSFULLY!")
        elif total_success > 0:
            print("PARTIAL SUCCESS - Some services may need manual intervention")
        else:
            print("CRITICAL FAILURE - No services were created successfully")
        
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print("USER INTERRUPTION DETECTED - Graceful shutdown initiated")
        print("All thread-specific ROS contexts will be cleaned up automatically")
        print("=" * 70)
    except Exception as e:
        print(f"\n[CRITICAL] System error during PPCom service creation: {e}")
    finally:
        # Note: No global ROS cleanup needed - each thread handles its own context
        print("PPCom service creation task completed. All ROS contexts cleaned up.")
        print("Program terminated.")


# =============================================================================
# Application Entry Point
# =============================================================================

def main():
    """
    Main application entry point
    
    Handles top-level exception catching and provides clean program termination.
    No global ROS initialization - contexts are managed per-thread.
    """
    try:
        create_all_ppcom_services()
    except Exception as e:
        print(f"[FATAL] Error during program startup: {e}")
        print("Please check your ROS2 environment and PPCom service availability")


if __name__ == '__main__':
    main()