#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def write_command(cmd):
    ser.write(cmd + "\n")
    return read_command()

def read_command():
    return_str = ""
    while True:
        recv_char = ser.read()
        return_str += recv_char
        if recv_char == ">":
            break
    return return_str

def cmd_vel_callback(msg):
    print(write_command("VCX" + str(msg.linear.x)))
    print(write_command("VCR" + str(msg.angular.z)))

rospy.init_node("mecbot")
cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=0)
ser.reset_input_buffer()
ser.reset_output_buffer()

rospy.spin()