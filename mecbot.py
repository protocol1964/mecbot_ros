#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    ser.write("VCX" + str(msg.linear.x) + "\r\n")
    ser.write("VCR" + str(msg.angular.z) + "\r\n")

rospy.init_node("mecbot")
cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=0)

rospy.spin()