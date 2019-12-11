#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import mecbot
import math
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

g_vcx_val = 0.0
g_vcr_val = 0.0


def cmd_vel_callback(msg):
    global g_vcx_val
    global g_vcr_val
    g_vcx_val = msg.linear.x
    g_vcr_val = msg.angular.z


def main():
    rospy.init_node("pathfinder")
    cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    rate = rospy.Rate(30)

    device_path = rospy.get_param("/pathfinder/device", "/dev/ttyUSB0")
    mb = mecbot.Mecbot(device_path, 57600)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    br = tf.TransformBroadcaster()

    vcx_last = 0.0
    vcr_last = 0.0

    world_x = 0.0
    world_y = 0.0
    world_th = 0.0

    pulse_sum_l = 0
    pulse_sum_r = 0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        pulse_count = None
        try:
            pulse_count = mb.measure_pulse()
        except mecbot.MecbotMeasureError:
            pass
        else:
            calc_result = mb.calc_pos(world_x, world_y, world_th, pulse_count[2] * -1, pulse_count[3] * -1)

            last_time = current_time
            current_time = rospy.Time.now()

            dt_duration = current_time - last_time
            dt = dt_duration.to_sec()
            dx = calc_result[0] - world_x
            dy = calc_result[1] - world_y
            dth = calc_result[2] - world_th

            vx = dx/dt
            vy = dy/dt
            vth = dth/dt

            world_x = calc_result[0]
            world_y = calc_result[1]
            world_th = calc_result[2]

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, world_th)

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.pose.pose = Pose(Point(world_x, world_y, 0.),
                                  Quaternion(*odom_quat))
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            odom_pub.publish(odom)

            pulse_sum_r += pulse_count[2] * -1
            pulse_sum_l += pulse_count[3] * -1

            br.sendTransform((0, -0.182, 0.13),
                             tf.transformations.quaternion_from_euler(math.radians(90), pulse_sum_r/float(mb.PULSE_OF_ROTATION)*2*math.pi, 0),
                             rospy.Time.now(),
                             "right_wheel",
                             "base_link")
            br.sendTransform((0, 0.182, 0.13),
                             tf.transformations.quaternion_from_euler(math.radians(-90), pulse_sum_l/float(mb.PULSE_OF_ROTATION)*2*math.pi, 0),
                             rospy.Time.now(),
                             "left_wheel",
                             "base_link")

        vcx_buf = g_vcx_val
        vcr_buf = g_vcr_val * -1

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