################ Photographer Trajectory Code ################
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 Kios Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Header, Float32, Bool, Int16MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point
from kios_solution.msg import area
from visualization_msgs.msg import MarkerArray, Marker
import math
import numpy as np
import heapq
import threading
import traceback

maxVel = 3.0
debug = False
TAG = ""
odom = Odometry()
target = 0
neighbors = PointCloud2()
command_thread = None
cmdPub = None
coordinates = None
target_yaw = 0.0
grid_resolution = 6
namespace = "sentosa"
adjacency = np.zeros((2,2))

offsets_all = [
    (-1,-1,0), (0,-1,0), (1,-1,0), (1,0,0), (1,1,0), (0,1,0), (-1,1,0), (-1,0,0), 
    (-1,-1,1), (0,-1,1), (1,-1,1), (1,0,1), (1,1,1), (0,1,1), (-1,1,1), (-1,0,1),(0,0,1),
    (-1,-1,-1), (0,-1,-1), (1,-1,-1), (1,0,-1), (1,1,-1), (0,1,-1), (-1,1,-1), (-1,0,-1),(0,0,-1)
]
offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

def dijkstra(g, arrival_pub, s, t):
    
    if (s==t):
        # init arrival message
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        # log_info("ARRIVED")
        return [s,s]
    
    if (len(np.nonzero(g[s,:])[0]) == 0): 
        log_info("Source " + str(s) + " blocked")
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        return [s,s]
    
    if (len(np.nonzero(g[:,t])[0]) == 0):
        log_info("Target " + str(t) + " not reachable")
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        return [s,s]
    
    q = []
    d = {n: float('inf') for n in range(len(g))}
    p = {}

    d[s] = 0
    heapq.heappush(q, (0, s))

    last_w, curr_v = heapq.heappop(q)

    # log_info(g[:,t])
    while  curr_v != t:

        neighbor_indices = np.nonzero(g[curr_v,:])
        #log_info(neighbor_indices)
        #for n, n_w in zip(nodes,g[curr_v]):
        #log_info("kokos")
        for n in neighbor_indices[0]:
            #log_info("kokos")
            n_w = g[curr_v,n]
            #if n_w == 0:
            #    continue

            cand_w = last_w + n_w # equivalent to d[curr_v] + n_w 
            # print d # uncomment to see how deltas are updated
            if cand_w < d[n]:
                d[n] = cand_w
                p[n] = curr_v
                heapq.heappush(q, (cand_w, n))
        last_w, curr_v = heapq.heappop(q)  

    return generate_path(p, s, t)

def generate_path(parents, start, end):
        path = [end]
        #log_info("Recreating path")
        while True:
            key = parents[path[0]]
            path.insert(0, key)
            if key == start:
                break
        #log_info("Returning path")
        return path

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")
    #print(TAG)

def odomCallback(msg):
    global odom
    odom = msg

def targetCallback(msg):
    global target, coordinates
    target_point = msg
    target =  closest_node_index((target_point.x,target_point.y,target_point.z),coordinates)

def yawCallback(msg):
    global target_yaw
    target_yaw = msg.data

def veloCallback(msg):
    global maxVel
    maxVel = msg.data

def neighCallback(msg):
    global neighbors
    neighbors = msg

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

def update_adjacency_with_neighbors(adjacency_og):
    global neighbors, grid_resolution, coordinates, area_details
    adjacency_temp = np.copy(adjacency_og)
    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        if point[3] != 0 and point[2] >= 1: 
            index = closest_node_index_1((point[0], point[1], point[2]), coordinates)
            adjacency_temp[:,index]=0
            for q, offset in enumerate(offsets_cross):
                neighbor_x = coordinates[index,0]+(offset[0] * grid_resolution)
                neighbor_y = coordinates[index,1]+(offset[1] * grid_resolution)
                neighbor_z = coordinates[index,2]+(offset[2] * grid_resolution)
            
                gone_too_far_x = (neighbor_x < area_details.minPoint.x) or (neighbor_x > (area_details.minPoint.x + area_details.size.x*area_details.resolution.data))
                gone_too_far_y = (neighbor_y < area_details.minPoint.y) or (neighbor_y > (area_details.minPoint.y + area_details.size.y*area_details.resolution.data))
                gone_too_far_z = (neighbor_z < area_details.minPoint.z) or (neighbor_z > (area_details.minPoint.z + area_details.size.z*area_details.resolution.data))
                if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                    #log_info(f"{gone_too_far_x}, {gone_too_far_y}, {gone_too_far_z}")
                    continue
               
                neighbor_index = closest_node_index_1((neighbor_x, neighbor_y, neighbor_z),coordinates)
                adjacency_temp[:,neighbor_index]=0

    arr = np.sum(adjacency_temp, axis=1)
    isolated_indicies = np.where(arr <= 2)[0]
    adjacency_temp[:,isolated_indicies] = 0

    return adjacency_temp

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def go_to_point():
    global cmd_pub, odom, waypoint, target_yaw, maxVel
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print(waypoint)
        if waypoint[0] != -3000:
            header_msg = Header()
            header_msg.frame_id = 'world'
            trajset_msg = MultiDOFJointTrajectory()
            trajpt_msg = MultiDOFJointTrajectoryPoint()
            transform_msgs = Transform()
            translation_msg = Vector3()
            rotation_msg = Quaternion()
            zero_vector_msg = Vector3()
            velocities_msg = Twist()
            acceleration_msg = Twist()
            
            
            # if maxVel == 0.0:
            #     translation_msg.x = waypoint[0]
            #     translation_msg.y = waypoint[1]
            #     translation_msg.z = waypoint[2]
            #     velocities_msg.linear.x = 0.0#max(min((waypoint[0]-odom.pose.pose.position.x) * 1.0,maxVel), -maxVel)
            #     velocities_msg.linear.y = 0.0#max(min((waypoint[1]-odom.pose.pose.position.y) * 1.0,maxVel), -maxVel)
            #     velocities_msg.linear.z = 0.0#max(min((waypoint[2]-odom.pose.pose.position.z) * 1.0,2.0), -2.0)
            # else:
            translation_msg.x = 0.0
            translation_msg.y = 0.0
            translation_msg.z = 0.0
            velocities_msg.linear.x = max(min((waypoint[0]-odom.pose.pose.position.x) * 1.0,maxVel), -maxVel)
            velocities_msg.linear.y = max(min((waypoint[1]-odom.pose.pose.position.y) * 1.0,maxVel), -maxVel)
            velocities_msg.linear.z = max(min((waypoint[2]-odom.pose.pose.position.z) * 1.0,maxVel), -maxVel)

            rotation_msg.x = 0.0
            rotation_msg.y = 0.0
            rotation_msg.z = np.sin(target_yaw/2.0)
            rotation_msg.w = np.cos(target_yaw/2.0)          
            
            #velocities_msg.linear = zero_vector_msg
            #q = odom.pose.pose.orientation
            #agent_yaw = np.degrees(np.arctan2(2.0 * (q.y * q.z + q.w *q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z))

            velocities_msg.angular.x = 0.0
            velocities_msg.angular.y = 0.0
            velocities_msg.angular.z = 0.0#max(min((target_yaw - agent_yaw) * 0.5, 5.0), -5.0)
            
            acceleration_msg.linear.x = 0.0
            acceleration_msg.linear.y = 0.0
            acceleration_msg.linear.z = 0.0

            acceleration_msg.angular.x = 0.0
            acceleration_msg.angular.y = 0.0
            acceleration_msg.angular.z = 0.0
            
            transform_msgs.translation = translation_msg
            transform_msgs.rotation = rotation_msg
            
            trajpt_msg.transforms.append(transform_msgs)
            trajpt_msg.velocities.append(velocities_msg)
            trajpt_msg.accelerations.append(acceleration_msg)
            
            trajset_msg.points.append(trajpt_msg)
            
            header_msg.stamp = rospy.Time.now()
            trajset_msg.header = header_msg

            cmdPub.publish(trajset_msg)
        
        rate.sleep()

def closest_node_index(node, nodes):
    global adjacency_neigh
    arr = np.sum(adjacency_neigh, axis=0)
    valid_dist_indices = np.nonzero(arr)[0]
    distances = np.linalg.norm(nodes[valid_dist_indices] - node, axis=1)
    # nodes = np.asarray(nodes)
    # deltas = nodes[valid_dist_indices] - node
    # dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    
    return valid_dist_indices[np.argmin(distances)]#valid_dist_indices[np.argmin(dist_2)]

def publish_graph_viz():
    global waypoint, namespace, viz_pub, coordinates, adjacency
    marker_array = MarkerArray()
    rate = rospy.Rate(0.1)
    # while True:
    for indx, coord in enumerate(coordinates):
        if sum(adjacency[:,indx]) == 0:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = grid_resolution
            marker.scale.y = grid_resolution
            marker.scale.z = grid_resolution
            #marker.color.a = 0.05
            #index = closest_node_index(coord,coords)
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            # else:
            #     marker.color.r = 1.0
            #     marker.color.g = 1.0
            #     marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker_array.markers.append(marker)
        elif sum(adjacency[indx,:]) == 0:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = grid_resolution
            marker.scale.y = grid_resolution
            marker.scale.z = grid_resolution
            #marker.color.a = 0.05
            #index = closest_node_index(coord,coords)
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.9
            # else:
            #     marker.color.r = 1.0
            #     marker.color.g = 1.0
            #     marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker_array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = grid_resolution/5.0
    marker.scale.y = grid_resolution/5.0
    marker.scale.z = grid_resolution/5.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = coordinates[target][0]
    marker.pose.position.y = coordinates[target][1]
    marker.pose.position.z = coordinates[target][2]
    marker_array.markers.append(marker)

    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1

    viz_pub.publish(marker_array)
        # rate.sleep()

def main():
    # init
    global cmdPub, waypoint, command_thread, coordinates, target, grid_resolution, namespace, debug, adjacency, area_details, viz_pub, adjacency_neigh
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
    except Exception as e:
        print(e)
        namespace = "sentosa"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)
    rospy.Subscriber("/"+namespace+"/command/yaw", Float32, yawCallback)
    rospy.Subscriber("/"+namespace+"/command/velocity", Float32, veloCallback)
    # Get Neighbor Positions
    rospy.Subscriber("/"+namespace+"/nbr_odom_cloud", PointCloud2, neighCallback)  

    # create command publisher
    cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    # adjacency vis pub
    viz_pub = rospy.Publisher("/"+namespace+"/adjacency_viz", MarkerArray, queue_size=1)
    # occupied coordinates publisher
    arrival_pub = rospy.Publisher('/'+namespace+'/arrived_at_target', Bool, queue_size=1)

    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)
    log_info("Construct Adjacency")    
    xrange = range(int(area_details.minPoint.x + area_details.resolution.data/2), int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    yrange = range(int(area_details.minPoint.y + area_details.resolution.data/2), int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    zrange = range(int(area_details.minPoint.z + area_details.resolution.data/2), int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    # Constructing the graph
    coordinates = np.asarray([(x,y,z) for x in xrange for y in yrange for z in zrange])
    adjacency_org = constuct_adjacency(area_details, coordinates)

    # create thread
    waypoint = (-3000,-3000,-3000)
    command_thread = threading.Thread(target=go_to_point)
    command_thread.start()

    occupied_msg = Int16MultiArray()
    log_info("Waiting for map from explorers")
    while len(occupied_msg.data) == 0:
        try:
            occupied_msg = rospy.wait_for_message("/jurong/adjacency/"+namespace, Int16MultiArray, 0.1)
            log_info("Receivied map from Jurong")
        except rospy.exceptions.ROSException as e:
            try:
                occupied_msg = rospy.wait_for_message("/raffles/adjacency/"+namespace, Int16MultiArray, 0.1)
                log_info("Receivied map from Raffles")
            except rospy.exceptions.ROSException as e:
                try:
                    occupied_msg = rospy.wait_for_message("/gcs/adjacency/"+namespace, Int16MultiArray, 0.1)
                    log_info("Receivied new map from GCS")
                except rospy.exceptions.ROSException as e:
                    pass
                # log_info("Waiting for map from explorers")
        rate.sleep() 
    
    log_info("Loading map")
    occupied_indicies = np.asarray(occupied_msg.data)
    adjacency = np.copy(adjacency_org)
    adjacency[:,occupied_indicies] = 0
    adjacency_neigh = update_adjacency_with_neighbors(adjacency)
    publish_graph_viz()

    log_info("Waiting for target point")
    arrival_pub.publish(True)
    try:
        rospy.wait_for_message("/"+namespace+"/command/targetPoint", Point)
    except rospy.exceptions.ROSException as e:
        log_info("Waiting for target point TIMEOUT")
        target =  closest_node_index((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),coordinates)

    while not rospy.is_shutdown():
        try:
            agent_index = closest_node_index_1(([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]),coordinates)
            #log_info("Generating path. Starting Point: " + str(agent_index) + " Target Point: " + str(target))
            path = dijkstra(adjacency_neigh, arrival_pub, agent_index, target)
            #log_info("Going to point: " + str(coordinates[path[1]]))
            waypoint = coordinates[path[1]]

            new_map = False
            try:
                occupied_msg = rospy.wait_for_message("/jurong/adjacency/"+namespace, Int16MultiArray, 0.1)
                # log_info("Receivied new map from Jurong")
                new_map = True
            except rospy.exceptions.ROSException as e:
                try:
                    occupied_msg = rospy.wait_for_message("/raffles/adjacency/"+namespace, Int16MultiArray, 0.1)
                    # log_info("Receivied new map from Raffles")
                    new_map = True
                except rospy.exceptions.ROSException as e:
                    try:
                        occupied_msg = rospy.wait_for_message("/gcs/adjacency/"+namespace, Int16MultiArray, 0.1)
                        # log_info("Receivied new map from GCS")
                        new_map = True
                    except rospy.exceptions.ROSException as e:
                        pass

            if new_map:
                # log_info("Updating map")
                adjacency = np.copy(adjacency_org)
                adjacency[:,np.asarray(occupied_msg.data)] = 0
            adjacency_neigh = update_adjacency_with_neighbors(adjacency)
            publish_graph_viz()
        except Exception as e:
            pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
        command_thread.terminate()
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()