#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
from mavros_msgs.srv import CommandHome, CommandHomeRequest

current_state = State()
land = False
current_pos = PoseStamped()
theta = 0
twist_x = []
twist_y = []
twist_z = []

def state_cb(msg):
    global current_state
    current_state = msg

def odom_cb(msg):
    global twist_list
    twist_x.append(msg.twist.twist.linear.x)
    twist_y.append(msg.twist.twist.linear.y)
    twist_z.append(msg.twist.twist.linear.z)

if __name__ == "__main__":
    rospy.init_node("vel")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
    odom_sub = rospy.Subscriber("/mavros/odometry/in", Odometry, callback = odom_cb)
    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/set_home")
    set_home_client = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(200)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 1.6
    pose.pose.position.y = 2.0
    pose.pose.position.z = 2.0
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    set_home_cmd = CommandHomeRequest()
    set_home_cmd.current_gps = False
    set_home_cmd.latitude = 1.6
    set_home_cmd.longitude = 2.0
    set_home_cmd.altitude = 0

    if set_home_client.call(set_home_cmd).success == True:
        rospy.loginfo("Set home position success")

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
                
        if(rospy.Time.now() - last_req) > rospy.Duration(10.0) and (rospy.Time.now() - last_req) < rospy.Duration(20.0):
            twist.linear.x = 2 * math.sin(theta)
            twist.linear.y = - 2 * math.cos(theta)
            twist.linear.z = 0
            local_vel_pub.publish(twist)
            if theta < 2*math.pi:
                theta += math.pi/256
            else:
                theta = 0
            rate.sleep()
            continue

        if(rospy.Time.now() - last_req) > rospy.Duration(20.0):
            land = True
            pose.pose.position.x = 1.6
            pose.pose.position.y = 2.0
            pose.pose.position.z = 0.1
            if(rospy.Time.now() - last_req) > rospy.Duration(25.0):
                rospy.spin()
        
        local_pos_pub.publish(pose)

        rate.sleep()

    print("x: {} m/s".format(max(twist_x)))
    print("y: {} m/s".format(max(twist_y)))
    print("z: {} m/s".format(max(twist_z)))