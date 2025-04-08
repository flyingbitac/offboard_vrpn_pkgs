#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('tf_node')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(400.0)
    while not rospy.is_shutdown():
        odom_in = rospy.wait_for_message('/mavros/odometry/in', Odometry, timeout=None)
        px = odom_in.pose.pose.position.x
        py = odom_in.pose.pose.position.y
        pz = odom_in.pose.pose.position.z
        x = odom_in.pose.pose.orientation.x
        y = odom_in.pose.pose.orientation.y
        z = odom_in.pose.pose.orientation.z
        w = odom_in.pose.pose.orientation.w
        br.sendTransform((px, py, pz),
                        (x, y, z, w), # x, y, z, w
                        rospy.Time.now(),
                        "base_link",
                        "map")
        rate.sleep()