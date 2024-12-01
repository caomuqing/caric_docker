######################### UAV FoV Code ########################
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 Kios Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from kios_solution.msg import norms
import numpy as np

debug = False
TAG = ""
odom = Odometry()
position = Point()
target = 0
command_thread = None
cmdPub = None
normals_msg = None
grid_resolution = 6
namespace = "jurong"
target = 0


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

def closest_node_index(node, nodes):
    agent_pos = np.asarray((node.x, node.y, node.z))
    verticies = np.zeros((len(nodes),3))
    for i, point in enumerate(nodes):
        verticies[i,0] = point.x
        verticies[i,1] = point.y
        verticies[i,2] = point.z
    deltas = np.subtract(verticies, agent_pos)
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

def vec_to_eurler(vector):
    global odom

    yaw = np.arctan2(-vector.y, -vector.x)
    pitch = np.arcsin(vector.z)

    return pitch, yaw

def targetCallback(msg):
    global target, normals_msg
    target_point = msg
    try:
        target =  closest_node_index(target_point,normals_msg.facet_mids)
    except:
        pass

def normCallback(msg):
    global normals_msg
    normals_msg = msg

def main():
    # init
    global grid_resolution, namespace, debug, odom, position, target, normals_msg
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " FOV SCRIPT]: ")
        #rospy.loginfo(TAG + namespace)
    except Exception as e:
        print(e)
        namespace = "ERROR"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " FOV SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    rate = rospy.Rate(10)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    # subscribe to target point
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)
    rospy.Subscriber("/"+namespace+"/norms/", norms, normCallback)
    # Get inspection norms
    log_info("Waiting for normals details")
    rospy.wait_for_message("/"+namespace+"/norms/", norms)

    # Create the publisher
    gmb_pub = rospy.Publisher('/'+namespace+'/command/gimbal', Twist, queue_size=1)
    yaw_pub = rospy.Publisher('/'+namespace+'/command/yaw', Float32, queue_size=1)
    gmb_cmd = Twist()
    gmb_cmd.linear.x = 1.0

    while not rospy.is_shutdown():
        ind = closest_node_index(position, normals_msg.facet_mids)
        normal_direction_position = normals_msg.normals[ind]
        pitch, yaw  = vec_to_eurler(normal_direction_position)

        gmb_cmd.linear.y = pitch # pitch
        gmb_cmd.linear.z = 0.0
        yaw_msg = Float32()
        yaw_msg.data = yaw # yaw
    
        gmb_pub.publish(gmb_cmd)
        yaw_pub.publish(yaw_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        print(e)
    finally:
        exit()