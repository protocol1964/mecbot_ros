#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import mecbot
from std_msgs.msg import *
from geometry_msgs.msg import Twist

g_vcx_val = 0.0
g_vcr_val = 0.0

def cmd_vel_callback(msg):
    global g_vcx_val
    global g_vcr_val
    g_vcx_val = msg.linear.x
    g_vcr_val = msg.angular.z

def main():
    rospy.init_node("mecbot")
    cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    rate = rospy.Rate(30)
    mb = mecbot.Mecbot()

    vcx_last = 0.0
    vcr_last = 0.0

    while not rospy.is_shutdown():
        try:
            print mb.measure_speed()
        except mecbot.MecbotMeasureError:
            pass

        vcx_buf = g_vcx_val
        vcr_buf = g_vcr_val

        if vcx_buf != vcx_last:
            mb.control_forward_speed(vcx_buf)
            vcx_last = vcx_buf
        if vcr_buf != vcr_last:
            mb.control_turning_speed(vcr_buf)
            vcr_last = vcr_buf

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()