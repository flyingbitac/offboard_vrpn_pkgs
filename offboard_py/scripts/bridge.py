#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

pos = PoseStamped()
odom = Odometry()

def pos_cb(msg):
    # while not rospy.is_shutdown():
    pos.header.stamp = rospy.Time.now()
    pos.header.frame_id = "world"
    pos.pose.position.x = msg.pose.position.z
    pos.pose.position.y = msg.pose.position.x
    pos.pose.position.z = msg.pose.position.y
    pos.pose.orientation = msg.pose.orientation
    # rospy.loginfo("Received pos data: Position: %s, Orientation: %s",
    #           msg.pose.position, msg.pose.orientation)

def odom_cb(msg):
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "map"
    odom.child_frame_id = "baselink"
    odom.pose.pose.position.x = pos.pose.position.x
    odom.pose.pose.position.y = pos.pose.position.z
    odom.pose.pose.position.z = - pos.pose.position.y
    odom.pose.pose.orientation = pos.pose.orientation
    odom.twist = msg.twist
    # rospy.loginfo("Received odom data: twist: %s",
    #           msg.twist)
    publish()

def publish():
    # rospy.loginfo("pub pos data: Position: %s, Orientation: %s",pos.pose.position, pos.pose.orientation)
    odom_pub.publish(odom)
    pos_pub.publish(pos)

while not rospy.is_shutdown():
    rospy.init_node('bridge')
    # rate = rospy.Rate(100)
    pos_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    odom_pub = rospy.Publisher("/odometry", Odometry, tcp_nodelay=True, queue_size=1)
    rospy.Subscriber('/vrpn_client_node/RigidBody/pose', PoseStamped, callback = pos_cb)
    rospy.Subscriber('/mavros/odometry/in', Odometry, callback = odom_cb)
    # timer = rospy.Timer(rospy.Duration(0.01), publish, oneshot=False, reset=False)
rospy.spin()
    # rate.sleep()