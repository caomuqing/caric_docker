#! /usr/bin/env python

import os
import yaml
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry as OdomMsg
from visualization_msgs.msg import MarkerArray, Marker

fleet = {'gcs'     : 'gcs',
         'jurong'  : 'explorer',
         'raffles' : 'explorer',
         'sentosa' : 'photographer',
         'changi'  : 'photographer',
         'nanyang' : 'photographer'}

def circular_rotate(lst, n):
    n = n % len(lst)  # To handle cases where n is greater than the list length
    rotated_list = lst[-n:] + lst[:-n]
    return rotated_list

class MeshVisualizer:

    def __init__(self):

        print('Initializing mesh visualizer')
        
        # Path to the building mesh models
        self.model_path = rospy.get_param("/model_path")
        self.model_dir = self.model_path.split('vision_create_new')[-1]
        print("model_path: ", self.model_path)

        # Path to the bounding boxes
        self.bbox_path = rospy.get_param("/bounding_box_path")
        print("bounding_box_path: ", self.bbox_path)

        self.modelMarkerPub = rospy.Publisher('model_viz', MarkerArray, queue_size=1)
        self.bboxMarkerPub  = rospy.Publisher('bbox_viz', MarkerArray, queue_size=1)       

        self.current_structure_count = None
        self.current_bbox_count  = None
        
        # Create a timer to update the mesh at 1s
        rospy.Timer(rospy.Duration(5.0), self.update_structure_mesh)
        rospy.Timer(rospy.Duration(5.0), self.update_bbox_mesh)

        # Create the pointcloud 
        self.bb_cloud = PointCloud()
        self.bb_cloud.header.frame_id = "world"
        
        boxes = yaml.safe_load(open(self.bbox_path + "/box_description.yaml", "r"))
        for box in boxes:
            
            center = np.array(boxes[box]['center'])
            size   = np.array(boxes[box]['size'])
            pose   = np.array(boxes[box]['orientation']).reshape(4, 4)

            print("box name: ",   center)
            print("box size: ",   size)
            print("box orie: \n", pose)

            v = self.get_bounding_box_vertices(center, size, pose)

            for i in range(0, v.shape[0]):
                vertex = Point32()
                vertex.x = v[i, 0]
                vertex.y = v[i, 1]
                vertex.z = v[i, 2]
                self.bb_cloud.points.append(vertex)    

        # Create the publisher for the bounding boxes
        self.bbPub = rospy.Publisher('/gcs/bounding_box_vertices', PointCloud, queue_size=1)
        rospy.Timer(rospy.Duration(5.0), self.publish_bbox_vertices)

        # # Subscribe to the drone ground truth to show their mesh
        # self.odomSub        = {}
        # self.lastOdomTime   = {}
        # self.droneMarker    = {}
        # self.droneSTL       = {}
        # self.droneMarkerPub = {}

        # # Create marker and publisher for the drone meshes
        # id_ = 0
        # for drone in fleet:
            
        #     id_ += 1
        #     marker = Marker()
        #     marker.id = id_

        #     stl = []
            
        #     if fleet[drone] == 'gcs':
        #         stl = ['package://rotors_description/meshes/gcs.stl']
        #     elif fleet[drone] == 'explorer':
        #         stl = ['package://rotors_description/meshes/firefly_explorer_0.stl',
        #                'package://rotors_description/meshes/firefly_explorer_1.stl',
        #                'package://rotors_description/meshes/firefly_explorer_2.stl']
        #     else:
        #         stl = ['package://rotors_description/meshes/firefly_photographer_0.stl',
        #                'package://rotors_description/meshes/firefly_photographer_1.stl',
        #                'package://rotors_description/meshes/firefly_photographer_2.stl']

        #     marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        #     marker.type = marker.MESH_RESOURCE
        #     marker.header.frame_id = "world"
        #     marker.scale.x = 7.0
        #     marker.scale.y = 7.0
        #     marker.scale.z = 7.0
        #     marker.pose.orientation.w = 1.0
        #     marker.color.a = 1.0
        #     marker.color.r = 1.0
        #     marker.color.g = 1.0
        #     marker.color.b = 1.0

        #     self.odomSub[drone]        = rospy.Subscriber('/' + drone + '/ground_truth/odometry', OdomMsg, self.update_drone_mesh, drone, queue_size=1)
        #     self.lastOdomTime[drone]   = rospy.Time.now()
        #     self.droneMarker[drone]    = marker
        #     self.droneSTL[drone]       = stl
        #     self.droneMarkerPub[drone] = rospy.Publisher('/' + drone + '/drone_mesh', Marker, queue_size=1)        

    def update_structure_mesh(self, event=None):

        # find all model in the model path
        files = [ file for file in os.listdir(self.model_path) if file.endswith(('.dae', '.stl', '.mesh'))]

        # if len(files) == self.current_structure_count:
        #     return
        # self.current_structure_count = len(files)

        # Marker to visualize
        modelMarkerArray = MarkerArray()

        for marker_id, file in enumerate(files):
            marker = Marker()
            marker.id = marker_id
            marker.mesh_resource = 'package://vision_create_new' + self.model_dir + '/' + file
            marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
            marker.type = marker.MESH_RESOURCE
            marker.header.frame_id = "world"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0            
            modelMarkerArray.markers.append(marker)

        self.modelMarkerPub.publish(modelMarkerArray)

    def update_bbox_mesh(self, event=None):

        files = [ file for file in os.listdir(self.bbox_path) if file.endswith(('.dae', '.stl', '.mesh'))]

        # if len(files) == self.current_bbox_count:
        #     return
        # self.current_bbox_count = len(files)

        # Marker to visualize
        bboxMarkerArray = MarkerArray()

        for marker_id, file in enumerate(files):
            # print('Loading file: %s', file)
            marker = Marker()
            marker.id = marker_id
            marker.mesh_resource = 'package://vision_create_new' + self.model_dir + '/bounding_boxes/' + file
            marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
            marker.type = marker.MESH_RESOURCE
            marker.header.frame_id = "world"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.color.a = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.9
            marker.color.b = 0.0            
            bboxMarkerArray.markers.append(marker)

        self.bboxMarkerPub.publish(bboxMarkerArray)

    # def update_drone_mesh(self, msg, drone):
        
    #     if (msg.header.stamp - self.lastOdomTime[drone]).to_sec() < 0.05:
    #         return
        
    #     # Update the time
    #     self.lastOdomTime[drone] = msg.header.stamp

    #     # Update the pose
    #     self.droneMarker[drone].pose = msg.pose.pose

    #     # Rotate the marker
    #     self.droneSTL[drone] = circular_rotate(self.droneSTL[drone], 1)
    #     self.droneMarker[drone].mesh_resource = self.droneSTL[drone][0]

    #     # Publish the new marker
    #     self.droneMarkerPub[drone].publish(self.droneMarker[drone])

    def get_bounding_box_vertices(self, c, s, T):
        
        hs = s / 2.0
        
        # Define the eight vertices of the rectangle
        vertices = np.array([ [-hs[0], -hs[1], -hs[2]],
                              [ hs[0], -hs[1], -hs[2]],
                              [ hs[0],  hs[1], -hs[2]],
                              [-hs[0],  hs[1], -hs[2]],
                              [-hs[0], -hs[1],  hs[2]],
                              [ hs[0], -hs[1],  hs[2]],
                              [ hs[0],  hs[1],  hs[2]],
                              [-hs[0],  hs[1],  hs[2]]])
        
        R = T[0:3, 0:3]
        rotated_vertices = np.dot(vertices, R.T)
        translated_vertices = rotated_vertices + c
        return translated_vertices

    def publish_bbox_vertices(self, event=None):
        self.bb_cloud.header.stamp = rospy.Time.now()
        self.bbPub.publish(self.bb_cloud)

if __name__ == '__main__':

    rospy.init_node("mesh_visualizer")

    # Create the visualizer    
    mv = MeshVisualizer()

    # Spin
    rospy.spin()