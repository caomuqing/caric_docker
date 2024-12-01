################## Explorer Trajectory Code ##################
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 KIOS Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Header, Float32, Bool, Int16MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area
from octomap_msgs.srv import BoundingBoxQuery
from visualization_msgs.msg import MarkerArray, Marker
import math
import numpy as np
import heapq
import threading
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
import traceback

maxVel = 4.0
debug = False
TAG = ""
odom = Odometry()
target = 0
neighbors = PointCloud2()
command_thread = None
update_from_neighbor_thread = None
cmdPub = None
coordinates = None
target_yaw = 0.0
grid_resolution = 6
namespace = "jurong"
adjacency = np.zeros((2,2))
adjacency_final = np.zeros((2,2))
update=True
mutex = threading.Lock()
#neighbors' offsets starting from same z plane counter-clockwise
offsets_all = [
    (-1,-1,0), (0,-1,0), (1,-1,0), (1,0,0), (1,1,0), (0,1,0), (-1,1,0), (-1,0,0), 
    (-1,-1,1), (0,-1,1), (1,-1,1), (1,0,1), (1,1,1), (0,1,1), (-1,1,1), (-1,0,1),(0,0,1),
    (-1,-1,-1), (0,-1,-1), (1,-1,-1), (1,0,-1), (1,1,-1), (0,1,-1), (-1,1,-1), (-1,0,-1),(0,0,-1)
]
offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

def sci_dijkstra(g, arrival_pub, s, t):
    if (s==t):
        # init arrival message
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        return [s,s]
    
    if (len(np.nonzero(g[s,:])[0]) == 0): 
        log_info("Source " + str(t) + " blocked")
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

    graph = csr_matrix(g)
    _, Pr = shortest_path(csgraph=graph, directed=False, method='D', return_predecessors=True)

    path = [t]   
    k = t  
    while Pr[s, k] != -9999:
        path.append(Pr[s, k])
        k = Pr[s, k]
    return path[::-1]

def dijkstra(g, arrival_pub, s, t):  
    if (s==t):
        # init arrival message
        log_info("Arrived at " + str(t))
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        return [s,s]
    
    if (len(np.nonzero(g[s,:])[0]) == 0): 
        log_info("Source " + str(t) + " blocked")
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        return [s,s]
    
    if (len(np.nonzero(g[:,t])[0]) == 0):
        log_info("Target " + str(t) + " not reachable")
        arrived_msg = Bool()
        arrived_msg.data = True
        # arrival_pub.publish(arrived_msg)
        return [s,s]
    
    q = []
    d = {n: float('inf') for n in range(len(g))}
    p = {}

    d[s] = 0
    heapq.heappush(q, (0, s))

    last_w, curr_v = heapq.heappop(q)

    while  curr_v != t:

        neighbor_indices = np.nonzero(g[curr_v,:])
        for n in neighbor_indices[0]:
            n_w = g[curr_v,n]

            cand_w = last_w + n_w # equivalent to d[curr_v] + n_w 
            if cand_w < d[n]:
                d[n] = cand_w
                p[n] = curr_v
                heapq.heappush(q, (cand_w, n))
        last_w, curr_v = heapq.heappop(q)  

    return generate_path(p, s, t)

def generate_path(parents, start, end):
        path = [end]
        while True:
            key = parents[path[0]]
            path.insert(0, key)
            if key == start:
                break
        return path

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")
        publish_text_viz(TAG + f"{info}")

def odomCallback(msg):
    global odom
    odom = msg

def targetCallback(msg):
    global target, coordinates
    target_point = msg
    try:
        target =  closest_node_index((target_point.x,target_point.y,target_point.z),coordinates)
    except:
        pass

def yawCallback(msg):
    global target_yaw
    target_yaw = msg.data

def veloCallback(msg):
    global maxVel
    maxVel = msg.data

def neighCallback(msg):
    global neighbors
    neighbors = msg
    
def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

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
                adjacency_1[my_index,neighbor_index] = 1#cost
                adjacency_1[neighbor_index,my_index] = 1#cost
            except:
                pass

    return adjacency_1

def update_adjacency(adjacency, coordinates, obstacle_coordinates):    
    global grid_resolution
    adjacency_temp = np.copy(adjacency)
    # Add octomap voxel centers as obstacles in graph
    inds = np.empty((0))
    for obstacle in obstacle_coordinates:
        index = closest_node_index_1((obstacle[0], obstacle[1], obstacle[2]), coordinates)
        inds = np.append(inds, [index], axis=0)

    inds = inds.astype(int)
    adjacency_temp[:,inds] = 0
    
    adjacency_neigh = update_adjacency_with_neighbors(adjacency_temp)

    return adjacency_temp, adjacency_neigh

def update_adjacency_with_neighbors(adjacency):
    global neighbors, grid_resolution, coordinates, area_details

    # add LOS neighbors as obstacles in graph
    adjacency_temp = np.copy(adjacency)

    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        if point[3] != 0 and point[2] >= 1:
            index = closest_node_index_1((point[0], point[1], point[2]), coordinates)
            adjacency_temp[:,index]=0
            for _, offset in enumerate(offsets_cross):
                neighbor_x = coordinates[index][0]+(offset[0] * grid_resolution)
                neighbor_y = coordinates[index][1]+(offset[1] * grid_resolution)
                neighbor_z = coordinates[index][2]+(offset[2] * grid_resolution)
            
                gone_too_far_x = (neighbor_x < area_details.minPoint.x) or (neighbor_x > (area_details.minPoint.x + area_details.size.x*area_details.resolution.data))
                gone_too_far_y = (neighbor_y < area_details.minPoint.y) or (neighbor_y > (area_details.minPoint.y + area_details.size.y*area_details.resolution.data))
                gone_too_far_z = (neighbor_z < area_details.minPoint.z) or (neighbor_z > (area_details.minPoint.z + area_details.size.z*area_details.resolution.data))
                if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                    continue
              
                neighbor_index = closest_node_index_1((neighbor_x, neighbor_y, neighbor_z),coordinates)
                adjacency_temp[:,neighbor_index]=0

    # mark isolated nodes as obstacles in graph
    arr = np.sum(adjacency_temp, axis=1)
    isolated_indicies = np.where(arr <= 2)[0]
    for _, index in enumerate(isolated_indicies):
        adjacency_temp[:,index] = 0

    return adjacency_temp

def update_from_neighbor(coordinates):
    global adjacency, update, namespace, mutex, adjacency_final, scenario
    log_info("waiting for update command")
    flag_pub = rospy.Publisher("/"+namespace+"/command/update_done", Bool, queue_size=1, latch=True)
    flag_pub2 = rospy.Publisher("/"+namespace+"/command/update", Bool, queue_size=1, latch=True)
    adj_pub = rospy.Publisher("/"+namespace+"/adjacency", Int16MultiArray, queue_size=1, latch=True)
    occ_pub = rospy.Publisher("/"+namespace+"/occupancy_coords", Int16MultiArray, queue_size=1, latch=True)
    bool_msg = Bool()
    bool_msg.data = True

    rospy.wait_for_message("/"+namespace+"/command/update", Bool)
    rate = rospy.Rate(1)
   
    if scenario != 'hangar':
        occupancy_coords = Int16MultiArray()
        log_info("Waiting for neighbor map")
        while len(occupancy_coords.data) == 0:
            try:
                if namespace == 'jurong' :
                    rospy.wait_for_message("/raffles/command/update/"+namespace, Bool,1)
                    occupancy_coords = rospy.wait_for_message('/raffles/occupancy_coords/'+namespace, Int16MultiArray, 1)
                else:
                    rospy.wait_for_message("/jurong/command/update/"+namespace, Bool,1)
                    occupancy_coords = rospy.wait_for_message('/jurong/occupancy_coords/'+namespace, Int16MultiArray, 1)
            except rospy.exceptions.ROSException as e:
                log_info("Waiting for neighbor map")
            rate.sleep()  
        flag_pub.publish(bool_msg)
        flag_pub2.publish(bool_msg)
        log_info("Acquiring mutex")
        mutex.acquire()
        update = False
        flag_pub2.publish(bool_msg)
        log_info("Merging map")
        flag_pub2.publish(bool_msg)
        # occupancy_coords = enumerate(sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z']))
        # adjacency_final, _ = update_adjacency(adjacency, coordinates, occupancy_coords)
        occupied_indicies = np.asarray(occupancy_coords.data)
        adjacency[:,occupied_indicies] = 0
        flag_pub2.publish(bool_msg)
        clear_agent_box(6, namespace)
        occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)
        flag_pub2.publish(bool_msg)
        occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z'])
        adjacency_final, _ = update_adjacency(adjacency, coordinates, occupancy_coords)
        flag_pub2.publish(bool_msg)
        mutex.release()
        log_info("Merging DONE")
    else:
        flag_pub2.publish(bool_msg)
        log_info("Acquiring mutex")
        mutex.acquire()
        update = False
        log_info("Final map update")
        clear_agent_box(6, namespace)
        occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)
        occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z'])
        adjacency_final, _ = update_adjacency(adjacency, coordinates, occupancy_coords)
        mutex.release()
        log_info("Final map update DONE")

    # filename = "./"+namespace+"_adjacency.csv"
    # np.savetxt(filename, adjacency_final, delimiter=",")

    arr = np.sum(adjacency_final, axis=0)
    occupied_msg = Int16MultiArray()
    occupied_msg.data = np.where(arr == 0)[0].astype(int)

    while not rospy.is_shutdown():
        clear_agent_box(6, namespace)
        flag_pub.publish(bool_msg)
        flag_pub2.publish(bool_msg)
        adj_pub.publish(occupied_msg)
        occ_pub.publish(occupied_msg)
        arr = np.sum(adjacency_final, axis=0)
        occupied_msg = Int16MultiArray()
        occupied_msg.data = np.where(arr == 0)[0].astype(int)
        publish_graph_viz(coordinates, adjacency_neigh)
        rate.sleep

def closest_node_index_1(node, nodes):
    distances = np.linalg.norm(nodes - node, axis=1)
    return np.argmin(distances)

def closest_node_index(node, nodes):
    global adjacency_neigh
    arr = np.sum(adjacency_neigh, axis=0)
    valid_dist_indices = np.nonzero(arr)[0]
    distances = np.linalg.norm(nodes[valid_dist_indices] - node, axis=1)
    # nodes = np.asarray(nodes)
    # deltas = nodes[valid_dist_indices] - node
    # dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    
    return valid_dist_indices[np.argmin(distances)]#valid_dist_indices[np.argmin(dist_2)]

def clear_agent_box(size, namespace):
    global odom, neighbors
    clear_bbox = rospy.ServiceProxy('/'+namespace+'/octomap_server_'+namespace+'/clear_bbx', BoundingBoxQuery)
    min = Point()
    max = Point()
    min.x = odom.pose.pose.position.x - size/4
    min.y = odom.pose.pose.position.y - size/4
    min.z = odom.pose.pose.position.z - size/4
    max.x = odom.pose.pose.position.x + size/4
    max.y = odom.pose.pose.position.y + size/4
    max.z = odom.pose.pose.position.z + size/4
    clear_bbox(max=max, min=min)

    for point in sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True):
        min.x = point[0] - size/4
        min.y = point[1] - size/4
        min.z = point[2] - size/4
        max.x = point[0] + size/4
        max.y = point[1] + size/4
        max.z = point[2] + size/4
        clear_bbox(max=max, min=min)

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
    
            if maxVel == 0.0:
                translation_msg.x = waypoint[0]
                translation_msg.y = waypoint[1]
                translation_msg.z = waypoint[2]
                velocities_msg.linear.x = 0.0#max(min((waypoint[0]-odom.pose.pose.position.x) * 1.0,maxVel), -maxVel)
                velocities_msg.linear.y = 0.0#max(min((waypoint[1]-odom.pose.pose.position.y) * 1.0,maxVel), -maxVel)
                velocities_msg.linear.z = 0.0#max(min((waypoint[2]-odom.pose.pose.position.z) * 1.0,2.0), -2.0)
            else:
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

def publish_text_viz(msg):
    viz_pub = rospy.Publisher("/"+namespace+"/text_viz", Marker, queue_size=1)
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.TEXT_VIEW_FACING
    marker.text = msg
    marker.action = marker.ADD
    marker.scale.x = 3.0
    marker.scale.y = 3.0
    marker.scale.z = 3.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    if namespace == "jurong":
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.y = -50.0
    else:
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.y = -57.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.z = 30.0

    viz_pub.publish(marker)

def publish_graph_viz(coords, adj):
    global waypoint, namespace, viz_pub
    marker_array = MarkerArray()

    for indx, coord in enumerate(coords):
        if sum(adj[:,indx]) == 0:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = grid_resolution
            marker.scale.y = grid_resolution
            marker.scale.z = grid_resolution
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker_array.markers.append(marker)
        elif sum(adj[indx,:]) == 0:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = grid_resolution
            marker.scale.y = grid_resolution
            marker.scale.z = grid_resolution
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.9
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
    if namespace == "jurong":
        marker.color.g = 0.0
        marker.color.b = 1.0
    else:
        marker.color.g = 1.0
        marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = coords[target][0]
    marker.pose.position.y = coords[target][1]
    marker.pose.position.z = coords[target][2]
    marker_array.markers.append(marker)

    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1

    viz_pub.publish(marker_array)

def main():
    # init
    global cmdPub, waypoint, command_thread, update_from_neighbor_thread, coordinates, target, grid_resolution, scenario
    global namespace, debug, adjacency, update, mutex, adjacency_final, area_details, viz_pub, adjacency_neigh
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
    except Exception as e:
        print(e)
        namespace = "jurong"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)
    rospy.Subscriber("/"+namespace+"/command/yaw", Float32, yawCallback)
    rospy.Subscriber("/"+namespace+"/command/velocity", Float32, veloCallback)
    
    # create command publisher
    cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    # adjacency vis pub
    viz_pub = rospy.Publisher("/"+namespace+"/adjacency_viz", MarkerArray, queue_size=1)
    # occupied coordinates publisher
    occ_pub = rospy.Publisher('/'+namespace+'/occupancy_coords', Int16MultiArray, queue_size=1)
    # occupied coordinates publisher
    arrival_pub = rospy.Publisher('/'+namespace+'/arrived_at_target', Bool, queue_size=1)

    # Get Neighbor Positions
    rospy.Subscriber("/"+namespace+"/nbr_odom_cloud", PointCloud2, neighCallback)

    # Create a ppcom publisher
    # Wait for service to appear
    log_info("Waiting for ppcom")
    rospy.wait_for_service('/create_ppcom_topic')
    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # Register the topic with ppcom router
    create_ppcom_topic(namespace, ['all'], '/'+namespace+'/occupancy_coords', 'std_msgs', 'Int16MultiArray')
    create_ppcom_topic(namespace, ['all'], '/'+namespace+'/adjacency', 'std_msgs', 'Int16MultiArray')
    # Register the topic with ppcom router
    if namespace == 'jurong':
        create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update_done', 'std_msgs', 'Bool')
        create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')
    else:
        create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update_done', 'std_msgs', 'Bool')
        create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')

    # Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area)

    xrange = range(int(area_details.minPoint.x + area_details.resolution.data/2), int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    yrange = range(int(area_details.minPoint.y + area_details.resolution.data/2), int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    zrange = range(int(area_details.minPoint.z + area_details.resolution.data/2), int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 

    # Constructing the graph
    log_info("Constructing initial graph")
    coordinates = np.asarray([(x,y,z) for x in xrange for y in yrange for z in zrange])
    adjacency_org = constuct_adjacency(area_details, coordinates)

    # Get obstacles' coordinates
    log_info("Waiting for octomap")
    occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)
    log_info("translating octomap")
    occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z'])
    log_info("calling adjacency update")
    adjacency, adjacency_neigh = update_adjacency(adjacency_org, coordinates, occupancy_coords)
    log_info("publishing adjacency")
    publish_graph_viz(coordinates, adjacency)

    # create ros control thread
    waypoint = (-3000,-3000,-3000)
    command_thread = threading.Thread(target=go_to_point)
    command_thread.start()

    # create map merge thread
    update_from_neighbor_thread = threading.Thread(target=update_from_neighbor, args=(coordinates,))
    update_from_neighbor_thread.start()

    log_info("Waiting for target point")
    rospy.wait_for_message("/"+namespace+"/command/targetPoint", Point)

    octomap_length = 0
    while not rospy.is_shutdown():
        try:
            agent_index = closest_node_index_1((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),coordinates)
            path = dijkstra(adjacency_neigh, arrival_pub, agent_index, target)
            # path = sci_dijkstra(adjacency_neigh, arrival_pub, agent_index, target)
            waypoint = coordinates[path[1]]

            clear_agent_box(6, namespace)
            occupancy_coords_msg = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)
            occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords_msg, skip_nans=True, field_names=['x','y','z'])
            
            if update:
                # log_info("Updating map")
                mutex.acquire()
                adjacency, adjacency_neigh = update_adjacency(adjacency_org,coordinates, occupancy_coords)
                mutex.release()
                arr = np.sum(adjacency, axis=0)
                # obstacle_indicies = np.where(arr == 0)[0].astype(int)
                occupied_msg = Int16MultiArray()
                occupied_msg.data = np.where(arr == 0)[0].astype(int)
                # for ind in obstacle_indicies:
                #     occupied_msg.data.append(ind)
                occ_pub.publish(occupied_msg)
            else:            
                if abs(octomap_length - occupancy_coords_msg.width) > 20:
                    mutex.acquire()
                    log_info("Updating Map")
                    # publish_text_viz("Map Updated")
                    octomap_length = occupancy_coords_msg.width
                    try:
                        adjacency_final, adjacency_neigh = update_adjacency(adjacency_final,coordinates, occupancy_coords)
                        # adjacency_neigh = update_adjacency_with_neighbors(adjacency_final)
                    except:
                        pass
                    finally:
                        mutex.release()
                else:
                    adjacency_neigh = update_adjacency_with_neighbors(adjacency_final)
                # publish_text_viz("")

        except Exception as e:
            pass
        
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
        command_thread.terminate()
        update_from_neighbor_thread.terminte()
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()