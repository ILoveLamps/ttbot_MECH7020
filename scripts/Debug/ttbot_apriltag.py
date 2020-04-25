#!/usr/bin/env python
"""
    April tag detection for turtlebot3
    4/21/20
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""


import rospy
import tf
# from apriltag_ros.msg import AprilTagDetectionArray
# from tf.transformations import euler_from_quaternion


if __name__ == '__main__':
    rospy.init_node('ttbot_apriltag')

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            listener = tf.TransformListener()
            listener.waitForTransform('odom', 'tag_0', rospy.Time(), rospy.Duration(8.0))
            (trans, rot) = listener.lookupTransform('odom', 'camera', rospy.Time())
            print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
