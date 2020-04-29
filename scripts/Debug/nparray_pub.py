#!/usr/bin/env python
"""
    Testing np.array publisher
    - Don't forget to download the msg folder with ttbot_nparray.msg in it. Place it
        inside ttbot_waypoint folder
    - Make sure you update CMakeList.txt and package.xml with the updated files on git
    - Or update those files manually following the link below
    - http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Common_step_for_msg_and_srv
    - you also HAVE TO catkin_make_isolated after updating those files

    - Hirdayesh Shrestha
"""

import rospy
import numpy as np
from ttbot_waypoint.msg import tag_array    # # this is the custom message
from rospy.numpy_msg import numpy_msg


def np_array_pub():
    rospy.init_node('nparray_pub', anonymous=True)
    pub = rospy.Publisher('/nparray_topic', numpy_msg(tag_array), queue_size=10)

    dat = tag_array()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        dat.range = np.array([[1.0], [5.0], [3.0]])     # # test array

        pub.publish(dat)
        r.sleep()


if __name__ == '__main__':
    try:
        np_array_pub()
    except rospy.ROSInterruptException:
        pass
