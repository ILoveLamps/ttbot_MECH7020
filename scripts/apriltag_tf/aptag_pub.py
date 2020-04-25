#!/usr/bin/env python
"""
    04.21.20
    Broadcast Node
        1. Camera Feed Frame (april-tag detection)
        2. TurtleBot3 Frame (odometry)
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""

import rospy
from nav_msgs.msg import Odometry
import tf
# from apriltag_ros.msg import AprilTagDetectionArray
# from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Pose


# def ap_callback(msg):
#     # orientation_q = msg.detections[-1].pose.pose.pose.orientation
#     # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#
#     trans = msg.detections[-1].pose.pose.pose.position
#     orient = msg.detections[-1].pose.pose.pose.orientation
#
#     br = tf.TransformBroadcaster()
#     br.sendTransform((trans.x, trans.y, trans.z),
#                      (orient.x, orient.y, orient.z, orient.w),
#                      rospy.Time.now(),
#                      "camera",
#                      "base_footprint")


# def ap_listen():
#     """"""
#     """
#         listens to /tag_detections published by apriltag_ros
#     """
#     """"""
#     rospy.Subscriber('/tag_detections', AprilTagDetectionArray, ap_callback)


def base_footprint_listen():
    """"""
    """ 
        Listens to base_footprint from /odom
    """
    """"""
    rospy.Subscriber('/odom', Odometry, base_footprint_callback)


def base_footprint_callback(msg):
    """"""
    """ 
        Broadcasts base_footprint
    """
    """"""
    trans = msg.pose.pose.position
    orient = msg.pose.pose.orientation

    br = tf.TransformBroadcaster()
    br.sendTransform((trans.x, trans.y, trans.z),
                     (orient.x, orient.y, orient.z, orient.w),
                     rospy.Time.now(),
                     "camera",
                     "base_footprint")


if __name__ == '__main__':
    """"""
    """ 
        Main program loop
    """
    """"""
    rospy.init_node('aptag_pub')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            # ap_listen()
            base_footprint_listen()
        except rospy.ROSInterruptException:
            pass
        rate.sleep()


