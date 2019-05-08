#!/usr/bin/env python
import rospy
import serial, sys, re
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def write_command(cmd):
    ser.write(cmd + "\n")
    return read_command()

def read_command():
    return_str = ser.readline()
    ser.reset_input_buffer()
    return return_str

def cmd_vel_callback(msg):
    global g_vcx_val
    global g_vcr_val
    g_vcx_val = msg.linear.x
    g_vcr_val = msg.angular.z

def str2int(string):
    return int(string)

rospy.init_node("mecbot")
cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
rate = rospy.Rate(10)

g_vcx_val = 0.0
g_vcr_val = 0.0
vcx_last = 0.0
vcr_last = 0.0

ser = serial.Serial("/dev/ttyUSB0", 57600, timeout=0)
ser.reset_input_buffer()
ser.reset_output_buffer()

while not rospy.is_shutdown():
    encoder_data_raw = write_command("ME")
    encoder_data = map(str2int, re.findall("[-]?\d+", encoder_data_raw))
    print(encoder_data)

    vcx_buf = g_vcx_val
    vcr_buf = g_vcr_val
    if vcx_buf != vcx_last:
        write_command("VCX" + str(g_vcx_val))
        vcx_last = vcx_buf
    if vcr_buf != vcr_last:
        write_command("VCR" + str(g_vcr_val))
        vcr_last = vcr_buf

    rate.sleep()