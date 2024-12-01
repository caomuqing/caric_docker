####################### GCS Logic Code #######################
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 KIOS Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Int16MultiArray
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2
from nav_msgs.msg import Odometry
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area
import traceback

debug = False
TAG = ""
grid_res = 6
odom = Odometry()
jurongMap = Int16MultiArray()
jurongTime = 0
rafflesMap = Int16MultiArray()
rafflesTime = 0

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + info)

def odomCallback(msg):
    global odom
    odom = msg

def jurongMapCallback(msg):
    global jurongMap, jurongTime
    jurongMap = msg
    jurongTime = rospy.Time.now()

def rafflesMapCallback(msg):
    global rafflesMap, rafflesTime
    rafflesMap = msg
    rafflesTime = rospy.Time.now()

def process_boxes(msg):
    global debug
    points = msg.points
    minx = 99999
    miny = 99999
    minz = 99999
    maxx = -99999
    maxy = -99999
    maxz = -99999
    for point in points:
        if minx > point.x:
            minx = point.x
        if maxx < point.x:
            maxx = point.x

        if miny > point.y:
            miny = point.y
        if maxy < point.y:
            maxy = point.y

        if minz > point.z:
            minz = point.z
        if maxz < point.z:
            maxz = point.z

    return [minx, maxx, miny, maxy, minz, maxz]

def find_world_min_max(msg, min_max):
    minx = min_max[0]
    miny = min_max[2]
    minz = min_max[4]
    maxx = min_max[1]
    maxy = min_max[3]
    maxz = min_max[5]

    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        if minx > point[0]:
            minx = point[0]
        if maxx < point[0]:
            maxx = point[0]

        if miny > point[1]:
            miny = point[1]
        if maxy < point[1]:
            maxy = point[1]

        if minz > point[2]:
            minz = point[2]
        if maxz < point[2]:
            maxz = point[2]

    # Uncomment the following to include GCS in the min max world calculation
    # if minx > odom.pose.pose.position.x:
    #     minx = odom.pose.pose.position.x
    # if maxx < odom.pose.pose.position.x:
    #     maxx = odom.pose.pose.position.x

    # if miny > odom.pose.pose.position.y:
    #     miny = odom.pose.pose.position.y
    # if maxy < odom.pose.pose.position.y:
    #     maxy = odom.pose.pose.position.y

    # if minz > odom.pose.pose.position.z:
    #     minz = odom.pose.pose.position.z
    # if maxz < odom.pose.pose.position.z:
    #     maxz = odom.pose.pose.position.z

    relax = 5
    minx = round(minx-relax)
    maxx = round(maxx+relax)
    if (abs(minx) + abs(maxx))%grid_res!=0:
        maxx += grid_res-(abs(minx) + abs(maxx))%grid_res

    miny = round(miny-relax)
    maxy = round(maxy+relax)
    if (abs(miny) + abs(maxy))%grid_res!=0:
            maxy += grid_res-(abs(miny) + abs(maxy))%grid_res

    minz = round(max(2.0,minz-relax))
    maxz = round(maxz+relax)
    if (abs(minz) + abs(maxz))%grid_res!=0:
            maxz += grid_res-(abs(minz) + abs(maxz))%grid_res
    return [minx, maxx, miny, maxy, minz, maxz]

def main():
    global debug, scenario, jurongMap, jurongTime, rafflesMap, rafflesTime
    # init
    try:
        namespace = rospy.get_param('namespace')
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_res = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " SCRIPT]: ")
    except Exception as e:
        print(e)
        namespace = "gcs"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " SCRIPT]: ")

    rospy.init_node(namespace, anonymous=True)
    log_info(namespace)

    rate = rospy.Rate(1)

    # wait for gazebo
    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rate.sleep()

    state = get_state(model_name=scenario)
    while state.status_message != "GetModelState: got properties":
        if debug:
            rospy.loginfo("[TESTING FLIGHT SCRIPT: GET STATE]: " + namespace)
        state = get_state(model_name=namespace)
        rate.sleep()
    
    jurongTime = rospy.Time.now()
    rafflesTime = rospy.Time.now()
    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/jurong/adjacency/"+namespace, Int16MultiArray, jurongMapCallback)
    rospy.Subscriber("/raffles/adjacency/"+namespace, Int16MultiArray, rafflesMapCallback)

    # Create a ppcom publisher
    # Wait for service to appear
    rospy.wait_for_service('/create_ppcom_topic')

    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # Register the topic with ppcom router
    create_ppcom_topic('gcs', ['all'], '/world_coords', 'kios_solution', 'area')
    create_ppcom_topic('gcs', ['all'], "/"+namespace+"/adjacency", 'std_msgs', 'Int16MultiArray')
    # Create the publisher
    # coords pub
    msg_pub = rospy.Publisher('/world_coords', area, queue_size=1)
    # norm pub
    adj_pub = rospy.Publisher("/"+namespace+"/adjacency", Int16MultiArray, queue_size=1, latch=True)


    # Get Bounding Box Verticies
    bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
    min_max = process_boxes(bboxes)

    # Get Neighbor Positions
    log_info("Waiting for neighbors positions")
    neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)

    # Calculate discretized world
    log_info("Calculating discretized world size")
    min_max = find_world_min_max(neighbors, min_max)
    size_x = (abs(min_max[0]) + abs(min_max[1]))/grid_res
    size_y = (abs(min_max[2]) + abs(min_max[3]))/grid_res
    size_z = (abs(min_max[4]) + abs(min_max[5]))/grid_res

    area_msg = area()
    area_msg.minPoint.x = min_max[0]
    area_msg.minPoint.y = min_max[2]
    area_msg.minPoint.z = min_max[4]
    area_msg.size.x = size_x
    area_msg.size.y = size_y
    area_msg.size.z = size_z
    area_msg.resolution.data = grid_res

    log_info("DONE")
    while not rospy.is_shutdown():
        rate.sleep()
        msg_pub.publish(area_msg)
        if jurongTime > rafflesTime:
            adj_pub.publish(jurongMap)
        else:
            adj_pub.publish(rafflesMap)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()