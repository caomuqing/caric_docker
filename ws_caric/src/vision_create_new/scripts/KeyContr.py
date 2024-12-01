#!/usr/bin/env python3

import rospy
import curses
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3

def update_trajectory(stdscr, trajectory_msg):
    rate = rospy.Rate(10)  # Publish rate

    while not rospy.is_shutdown():
        key = stdscr.getch()
        if key == ord('c'):
            break
        elif key == ord('q'):
            trajectory_msg.points[0].transforms[0].translation.z += 0.6
        elif key == ord('e'):
            trajectory_msg.points[0].transforms[0].translation.z -= 0.6
        elif key == ord('w'):
            trajectory_msg.points[0].transforms[0].translation.x += 0.6
        elif key == ord('s'):
            trajectory_msg.points[0].transforms[0].translation.x-= 0.6
        elif key == ord('a'):
            trajectory_msg.points[0].transforms[0].translation.y += 0.6
        elif key == ord('d'):
            trajectory_msg.points[0].transforms[0].translation.y-= 0.6
        elif key == ord('u'):

            pass

        trajectory_msg.header.stamp = rospy.Time.now()
        pub.publish(trajectory_msg)
        rate.sleep()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('keyboard_control_trajectory_node')
    pub = rospy.Publisher('/jurong/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

    # Create a MultiDOFJointTrajectory message
    trajectory_msg = MultiDOFJointTrajectory()
    trajectory_msg.header.stamp = rospy.Time.now()
    trajectory_msg.joint_names = ['']  # Joint names can be empty

    # Create a trajectory point
    point = MultiDOFJointTrajectoryPoint()
    transform = Transform()
    transform.translation.x =-11
    transform.translation.y = 11
    transform.translation.z = 0
    transform.rotation.x = 0.0
    transform.rotation.y = 0.0
    transform.rotation.z = 0.0
    transform.rotation.w = 0.0
    point.transforms.append(transform)

    # Set velocities and accelerations (you can adjust as needed)
    point.velocities.append(Twist())
    point.accelerations.append(Twist())

    # Set time from start
    point.time_from_start = rospy.Duration(0)

    # Add the trajectory point to the message
    trajectory_msg.points.append(point)

    try:
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(1)
        stdscr.addstr(0, 0, "Press 'w' to move up, 's' to move down, 'q' to quit")

        update_trajectory(stdscr, trajectory_msg)

    except rospy.ROSInterruptException:
        pass
    finally:
        curses.endwin()