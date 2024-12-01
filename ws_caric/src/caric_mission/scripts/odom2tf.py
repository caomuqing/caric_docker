import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry as OdomMsg

broadcaster = tf2_ros.TransformBroadcaster()

def odomCallback(msg):

    # Create a TransformStamped message
    transform_stamped = TransformStamped()

    transform_stamped.header.stamp            = msg.header.stamp
    transform_stamped.header.frame_id         = msg.header.frame_id
    transform_stamped.child_frame_id          = msg.child_frame_id
    transform_stamped.transform.translation.x = msg.pose.pose.position.x
    transform_stamped.transform.translation.y = msg.pose.pose.position.y
    transform_stamped.transform.translation.z = msg.pose.pose.position.z
    transform_stamped.transform.rotation.x    = msg.pose.pose.orientation.x
    transform_stamped.transform.rotation.y    = msg.pose.pose.orientation.y
    transform_stamped.transform.rotation.z    = msg.pose.pose.orientation.z
    transform_stamped.transform.rotation.w    = msg.pose.pose.orientation.w

    broadcaster.sendTransform(transform_stamped)

if __name__ == '__main__':

    rospy.init_node('odom2tf', anonymous=True)

    # Subscribe to the ppcom topo
    rospy.Subscriber("/jurong/ground_truth/odometry", OdomMsg, odomCallback)

    # Go to sleep and let the callback works
    rospy.spin()
