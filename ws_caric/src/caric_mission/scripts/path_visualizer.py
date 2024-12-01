#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import os
import time

class DronePathTracker:
    def __init__(self):
        self.drones = ["raffles", "jurong", "sentosa", "changi", "nanyang"]
        self.paths = {drone: [] for drone in self.drones}
        self.publishers = {}
        self.last_positions = {drone: None for drone in self.drones}
        self.min_distance = rospy.get_param('~min_distance', 0.5)  # Minimum distance to add a point
        
        self.init_publishers()
        self.init_subscribers()
        self.set_rviz_above()

    def init_publishers(self):
        for drone in self.drones:
            topic_name = f"{drone}/drone_path"
            self.publishers[drone] = rospy.Publisher(topic_name, Marker, queue_size=10)

    def init_subscribers(self):
        for drone in self.drones:
            topic_name = f"/{drone}/ground_truth/odometry"
            rospy.Subscriber(topic_name, Odometry, self.odometry_callback, drone)

    def odometry_callback(self, msg, drone_name):
        current_position = msg.pose.pose.position
        last_position = self.last_positions[drone_name]

        # Add the first point or ensure the drone has moved enough
        if last_position is None or self.calculate_distance(current_position, last_position) > self.min_distance:
            self.paths[drone_name].append(Point(current_position.x, current_position.y, current_position.z))
            self.last_positions[drone_name] = current_position
            self.publish_path(drone_name)


    def calculate_distance(self, point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return (dx**2 + dy**2 + dz**2)**0.5

    def publish_path(self, drone_name):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"{drone_name}_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.5  # Thickness of the path line

        # Set the color of the path line (default is red, can be changed in the launch file)
        marker.color.r = rospy.get_param(f'~{drone_name}_color_r', 1.0)
        marker.color.g = rospy.get_param(f'~{drone_name}_color_g', 0.0)
        marker.color.b = rospy.get_param(f'~{drone_name}_color_b', 0.0)
        marker.color.a = rospy.get_param(f'~{drone_name}_color_a', 1.0)

        marker.points = self.paths[drone_name]
        self.publishers[drone_name].publish(marker)

    def set_rviz_above(self):
        time.sleep(5)
        command = "wmctrl -r RViz -b add,above"
        result = os.system(command)
        if result == 0:
            rospy.logwarn("RViz window set to always on top.")
        else:
            rospy.logerr("Failed to execute wmctrl command. Please ensure wmctrl is installed and RViz is running.")

if __name__ == "__main__":
    rospy.init_node("drone_path_tracker", anonymous=True)
    tracker = DronePathTracker()
    rospy.spin()
