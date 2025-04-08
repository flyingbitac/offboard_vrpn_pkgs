#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

current_state = State()
land = False

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("atti")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
    local_attitude_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(200)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 1.6
    pose.pose.position.y = 2.0
    pose.pose.position.z = 1
    attitude = AttitudeTarget()
    attitude.type_mask=AttitudeTarget.IGNORE_ATTITUDE
    attitude.body_rate.x = 0
    attitude.body_rate.y = 0
    attitude.body_rate.z = 0
    attitude.thrust = 0

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
            attitude.header.stamp = rospy.Time.now()   
            attitude.type_mask=AttitudeTarget.IGNORE_ATTITUDE     
            attitude.body_rate.x = 1
            attitude.body_rate.y = 0
            attitude.body_rate.z = 0
            attitude.thrust = 0.8
            local_attitude_pub.publish(attitude)
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