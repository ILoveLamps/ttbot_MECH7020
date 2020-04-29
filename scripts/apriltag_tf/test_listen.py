#!/usr/bin/env python
"""
    Listen to aptag_pub.py
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""

import rospy
from ttbot_waypoint.msg import tag_array


def aptag_info_callback(taginfo):
    x = taginfo.x
    y = taginfo.y
    ids = taginfo.id
    quantity = taginfo.quantity

    print(x)
    print(y)
    print(ids)
    print(quantity)


if __name__ == '__main__':
    rospy.init_node('test_listen_aptag')
    rospy.Subscriber('/aptag_info', tag_array, aptag_info_callback)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pass
    r.sleep()
