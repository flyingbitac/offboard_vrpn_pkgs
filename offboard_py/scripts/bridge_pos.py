#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def subscribe_pos():
    while not rospy.is_shutdown():
        data = rospy.wait_for_message('/some_object_name_vrpn_client/estimated_odometry', Odometry, timeout=None)
        # rospy.loginfo("Received pos data: Position: %s, Orientation: %s",
        #           data.pose.position, data.pose.orientation)
        publish_pos(data)

def publish_pos(data):
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    pos = PoseStamped()
    pos.header.stamp = rospy.Time.now()
    pos.header.frame_id = "optitrack"
    # pos.child_frame_id = "base_link"
    pos.pose = data.pose.pose
    # rospy.loginfo("pub pos data: Position: %s, Orientation: %s",pos.pose.position, pos.pose.orientation)

    pub.publish(pos)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pos_node')
    rate = rospy.Rate(100) 
    subscribe_pos()