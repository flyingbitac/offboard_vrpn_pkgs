#! /usr/bin/env python

import rospy
from mavros_msgs.msg import HomePosition

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('set_home')
        rate = rospy.Rate(10)
        home_pub = rospy.Publisher('/mavros/home_position/home', HomePosition, queue_size=1)
        home_pos = HomePosition()
        home_pos.header.stamp = rospy.Time.now()
        home_pos.position.x = 1.6
        home_pos.position.y = 2.0
        home_pos.position.z = 0
        home_pub.publish(home_pos)
        rate.sleep()
    rospy.spin()