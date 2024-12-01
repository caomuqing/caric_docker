############### Photographer Path Planning Code ##############
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 Kios Center of Excellence"
__version__ = "7.0"
##############################################################

import sys
import rospy
from std_msgs.msg import Bool, Float32, Int16MultiArray, Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointCloud
from kios_solution.msg import norms, area
import sensor_msgs.point_cloud2
from scipy.spatial import Delaunay
import numpy as np
import math
import traceback

repeat = True
debug = False
TAG = ""
odom = Odometry()
position = Point()
target = 0
command_thread = None
cmdPub = None
coordinates = None
grid_resolution = 6
namespace = "jurong"
arrived = False
remaining_time = sys.maxsize
drone_IDs = {'gcs':0, 'jurong':1, 'raffles':2, 'sentosa':3, 'changi':4, 'nanyang':5}

offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

def closest_node_index_1(node, nodes):
    distances = np.linalg.norm(nodes - node, axis=1)
    return np.argmin(distances)

def constuct_adjacency(area_details, coordinates):
    global offsets_cross
    num_of_nodes = len(coordinates)
    adjacency_1 = np.zeros((num_of_nodes,num_of_nodes))
    log_info("Starting Adjacency calculation. Please wait... ")
    for _,coord in enumerate(coordinates):
        for _, offset in enumerate(offsets_cross):
            neighbor_x = coord[0]+(offset[0] * area_details.resolution.data)
            neighbor_y = coord[1]+(offset[1] * area_details.resolution.data)
            neighbor_z = coord[2]+(offset[2] * area_details.resolution.data)
           
            
            gone_too_far_x = (neighbor_x < area_details.minPoint.x) or (neighbor_x > (area_details.minPoint.x + area_details.size.x*area_details.resolution.data))
            gone_too_far_y = (neighbor_y < area_details.minPoint.y) or (neighbor_y > (area_details.minPoint.y + area_details.size.y*area_details.resolution.data))
            gone_too_far_z = (neighbor_z < area_details.minPoint.z) or (neighbor_z > (area_details.minPoint.z + area_details.size.z*area_details.resolution.data))
            if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                continue
            
            
            neighbor_index = closest_node_index_1((neighbor_x, neighbor_y, neighbor_z),coordinates)
            my_index = closest_node_index_1((coord[0], coord[1], coord[2]),coordinates)
            
            
            # cost = euclidean_distance_3d(coord, coordinates[neighbor_index])
            try:
                adjacency_1[my_index,neighbor_index] = 1 #cost
                adjacency_1[neighbor_index,my_index] = 1 #cost
            except:
                pass

    return adjacency_1

def check_point_inside_cuboid(vertices, point):
    DT = Delaunay(vertices)

    if DT.find_simplex(point) >= 0:
        return True
    return False

def calculateCircuits(positions, num_of_nodes, TravellingCost):
    UAVs = len(positions)
    # positions = index where each uav is located

    # Initialization of Set S matrices and CircuitX, CircuitY.
    Set_S_source = [[] for i in range(0, UAVs)]
    Set_S_destination = [[] for i in range(0, UAVs)]
    Set_S_cost = [[] for i in range(0, UAVs)]
    listV1 = [0 for i in range(0, num_of_nodes)]
    for z in range(0, UAVs):
        listV1[positions[z]] = 1
    while (sum(listV1) < num_of_nodes):
        for i in range(0, UAVs):
            node = 0
            flag = False
            futureCost = sys.maxsize
            for j in range(0, num_of_nodes):
                if (listV1[j] == 0):
                    if (futureCost >= TravellingCost[positions[i]][j]):
                        futureCost = TravellingCost[positions[i]][j]
                        node = j
                        flag = True
            if flag:
                listV1[node] = 1
                Set_S_source[i].append(positions[i])
                Set_S_destination[i].append(node)
                Set_S_cost[i].append(futureCost)
                positions[i] = node

    
    return Set_S_destination

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")

def odomCallback(msg):
    global odom, position
    odom = msg
    position = odom.pose.pose.position

def arrivedCallback(msg):
    global arrived
    arrived = msg.data

def missionTimeCallback(msg):
    global remaining_time
    remaining_time = msg.data.secs

def euclidean_distance_points(point1,point2):
    p1 = np.zeros((3,1))
    p1[0] = point1.x
    p1[1] = point1.y
    p1[2] = point1.z
    p2 = np.zeros((3,1))
    p2[0] = point2.x
    p2[1] = point2.y
    p2[2] = point2.z
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def euclidean_distance(p1,point):
    p2 = np.zeros((3,1))
    p2[0] = point.x
    p2[1] = point.y
    p2[2] = point.z
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def main():
    # init
    global grid_resolution, namespace, debug, odom, position, arrived, repeat
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
    except Exception as e:
        print(e)
        namespace = "ERROR"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/arrived_at_target", Bool, arrivedCallback)
    rospy.Subscriber("/"+namespace+"/mission_duration_remained", Duration, missionTimeCallback)

    # target point publisher
    target_pub = rospy.Publisher("/"+namespace+"/command/targetPoint", Point, queue_size=1)
    # velocity publisher
    velo_pub = rospy.Publisher("/"+namespace+"/command/velocity", Float32, queue_size=1)
    # norm pub
    norm_pub = rospy.Publisher("/"+namespace+"/norms", norms, queue_size=1)

    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)
    xrange = range(int(area_details.minPoint.x + area_details.resolution.data/2), int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    yrange = range(int(area_details.minPoint.y + area_details.resolution.data/2), int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    zrange = range(int(area_details.minPoint.z + area_details.resolution.data/2), int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    # Constructing the graph
    coordinates = np.asarray([(x,y,z) for x in xrange for y in yrange for z in zrange]).astype(float)
    adjacency_org = constuct_adjacency(area_details, coordinates)

    # Get Bounding Box Verticies
    bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
    bbox_points = np.zeros((int(len(bboxes.points)/8),8,3))
    counter = 0
    for i in range(0,int(len(bboxes.points)/8)):
        for j in range(8):
            bbox_points[i,j,0] = bboxes.points[counter].x
            bbox_points[i,j,1] = bboxes.points[counter].y
            bbox_points[i,j,2] = bboxes.points[counter].z
            counter += 1
    
    log_info("Waiting for traj script")
    rospy.wait_for_message("/"+namespace+"/arrived_at_target", Bool)
    neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
    init_pos = position
    gcs_pos = Point()
    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        if point[3] == 0:
            gcs_pos.x = point[0]
            gcs_pos.y = point[1]
            gcs_pos.z = point[2]
            break
    rate.sleep()

    # Generate and go to TSP points
    count = 0
    while repeat:
        log_info("Waiting for map from explorers")
        occupied_indicies = Int16MultiArray()
        while len(occupied_indicies.data) == 0:
            try:
                occupied_indicies = rospy.wait_for_message("/jurong/adjacency/"+namespace, Int16MultiArray,0.1)
                log_info("Receivied map from Jurong")
            except rospy.exceptions.ROSException as e:
                try:
                    occupied_indicies = rospy.wait_for_message("/raffles/adjacency/"+namespace, Int16MultiArray,0.1)
                    log_info("Receivied map from Raffles")
                except rospy.exceptions.ROSException as e:
                    try:
                        occupied_indicies = rospy.wait_for_message("/gcs/adjacency/"+namespace, Int16MultiArray, 0.1)
                        log_info("Receivied map from GCS")
                    except rospy.exceptions.ROSException as e:
                        pass
                        # log_info("Waiting for map from explorers")
            rate.sleep() 
        log_info("Loading map")
        adjacency = np.copy(adjacency_org)
        valid_dist_indices =  np.asarray(occupied_indicies.data)
        adjacency[:,valid_dist_indices] = 0
        log_info("Calculating waypoints")
        targeted_points = np.empty((0,1))
        inspect_points = np.empty((0,1))
        for index in valid_dist_indices:
                for box_i in range(0,int(len(bboxes.points)/8)):
                    if check_point_inside_cuboid(bbox_points[box_i], coordinates[index]):
                        targeted_points = np.append(targeted_points, index)
                        break
        
        targeted_points = targeted_points.astype(int)
        all_norms = np.empty((0,3))
        for target_point in targeted_points:
            # target_point = int(target_point)
            points = np.where(adjacency[target_point]>0)[0]
            inspect_points = np.append(inspect_points, points)
            for point in points:
                norm = coordinates[point] - coordinates[target_point]
                norm /= np.linalg.norm(norm)
                all_norms = np.append(all_norms, [norm], axis=0)
            
        log_info("Running unique")
        inspect_points, ind = np.unique(inspect_points,axis=0, return_index=True)
        all_norms = all_norms[ind]
        inspect_points = inspect_points.astype(int)
        
        norm_msg = norms()
        for i, point in enumerate(inspect_points):
            facet_mid = Point()
            facet_mid.x = coordinates[point,0]
            facet_mid.y = coordinates[point,1]
            facet_mid.z = coordinates[point,2]
            norm_msg.facet_mids.append(facet_mid)
            norm_point = Point()
            norm_point.x = all_norms[i,0]
            norm_point.y = all_norms[i,1]
            norm_point.z = all_norms[i,2]
            norm_msg.normals.append(norm_point)

        norm_pub.publish(norm_msg)
    
        neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
        uav_positions = np.empty((0,3))
        uav_indices = np.array([])
        for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
            if point[3] != drone_IDs['gcs']:
                uav_positions = np.append(uav_positions, [[point[0], point[1], point[2]]], axis=0)
                uav_indices = np.append(uav_indices, point[3])

        pos = 0
        while (pos < len(uav_indices)) and (uav_indices[pos] < drone_IDs[namespace]):
            pos += 1
        
        uav_positions = np.insert(uav_positions, pos, [position.x, position.y, position.z], axis=0)
        uav_indices = np.insert(uav_indices, pos, drone_IDs[namespace])

        num_of_agents = uav_positions.shape[0]

        points = np.concatenate((uav_positions, coordinates[inspect_points]))
        num_of_nodes = points.shape[0]

        adjacency = np.zeros((num_of_nodes,num_of_nodes))
        for i in range(num_of_nodes):
            for j in range(num_of_nodes):
                adjacency[i,j] = euclidean_distance_3d(points[i],points[j])

        log_info("Running mTSP")

        waypointsMatrix = calculateCircuits([i for i in range(num_of_agents)], num_of_nodes, adjacency)

        for waypoint in waypointsMatrix[pos]:
            point = Point()
            point.x = points[waypoint,0]
            point.y = points[waypoint,1]
            point.z = points[waypoint,2]
            # log_info("Setting target to point: " + str(points[waypoint]))
            while not arrived:        
                if remaining_time < 10.0:
                    d_init = euclidean_distance_points(position, init_pos)
                    d_gcs = euclidean_distance_points(position, gcs_pos)
                    los = False
                    if d_init <= d_gcs:
                        log_info("Setting target to initial point: " + str(init_pos))
                        while not los:
                            target_pub.publish(init_pos)
                            for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
                                if point[3] == 0:
                                    los = True
                            rate.sleep()
                    elif d_gcs < d_init:
                        log_info("Setting target to gcs point: " + str(gcs_pos))
                        while not los:
                            target_pub.publish(gcs_pos)
                            for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
                                if point[3] == 0:
                                    los = True
                            rate.sleep()
                else:
                    target_pub.publish(point)
                    rate.sleep()
            arrived = False

        if count < 2.0:
            vel_msg = Float32()
            vel_msg.data = 3.0 - count
            velo_pub.publish(vel_msg)
        count += 1.0
    
    log_info("Setting target to initial point: " + str(init_pos))
    while not arrived:
        target_pub.publish(init_pos)
        rate.sleep()
    arrived = False

    
    while not rospy.is_shutdown():
        log_info("Finished")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()