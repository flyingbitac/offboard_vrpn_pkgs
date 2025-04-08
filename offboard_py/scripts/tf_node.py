1 #! /usr/bin/env python 

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_node')
    world_br = tf.TransformBroadcaster()
    body_br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        world_br.sendTransform((0.0, 0.0, 0.0),
                        (-0.707, 0.0, 0.0, 0.707), # x, y, z, w
                        rospy.Time.now(),
                        "world_NED",
                        "world")   # world_NED是world的子坐标系
        body_br.sendTransform((0.0, 0.0, 0.0),
                        (-0.707, 0.0, 0.0, 0.707), # x, y, z, w
                        rospy.Time.now(),
                        "RigidBody_NED",
                        "RigidBody")   #RigidBody_NED是RigidBody的子坐标系
        rate.sleep()