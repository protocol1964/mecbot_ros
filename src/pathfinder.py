#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import mecbot
import math
from geometry_msgs.msg import Twist

g_vcx_val = 0.0
g_vcr_val = 0.0


def cmd_vel_callback(msg):
    global g_vcx_val
    global g_vcr_val
    g_vcx_val = msg.linear.x
    g_vcr_val = msg.angular.z


def main():
    rospy.init_node("pathfinder")
    rospy.loginfo('Started Pathfinder')
    device = rospy.get_param('~device', '/dev/ttyUSB0')
    cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    rate = rospy.Rate(30)
    mb = mecbot.Mecbot(device, 57600)
    rospy.loginfo('Connected to Mecbot with path: ' + device)
    br = tf.TransformBroadcaster()

    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "odom",
                     "map")

    vcx_last = 0.0
    vcr_last = 0.0

    last_x = 0.0
    last_y = 0.0
    last_theta = 0.0

    pulse_sum_l = 0
    pulse_sum_r = 0

    while not rospy.is_shutdown():
        pulse_count = None
        try:
            pulse_count = mb.measure_pulse()
        except mecbot.MecbotMeasureError:
            pass
        else:
            calc_result = mb.calc_pos(last_x, last_y, last_theta, pulse_count[2] * -1, pulse_count[3] * -1)
            last_x = calc_result[0]
            last_y = calc_result[1]
            last_theta = calc_result[2]

            pulse_sum_r += pulse_count[2] * -1
            pulse_sum_l += pulse_count[3] * -1

            # Relative coordinates from odom to footprint of base link
            """
            br.sendTransform((last_x, last_y, 0),
                             tf.transformations.quaternion_from_euler(0, 0, last_theta),
                             rospy.Time.now(),
                             "base_footprint",
                             "odom")
            """
            # Relative coordinates from odom to middle point of the wheels (base link)
            br.sendTransform((last_x, last_y, mb.DIAMETER/2),
                             tf.transformations.quaternion_from_euler(0, 0, last_theta),
                             rospy.Time.now(),
                             "base_link",
                             "odom")
            # Relative coordinates from base link to right wheel
            br.sendTransform((0, mb.TREAD / 2 * -1, 0),
                             tf.transformations.quaternion_from_euler(math.radians(90), pulse_sum_r/float(mb.PULSE_OF_ROTATION)*2*math.pi, 0),
                             rospy.Time.now(),
                             "right_wheel",
                             "base_link")
            # Relative coordinates from base link to left wheel
            br.sendTransform((0, mb.TREAD / 2, 0),
                             tf.transformations.quaternion_from_euler(math.radians(-90), pulse_sum_l/float(mb.PULSE_OF_ROTATION)*2*math.pi, 0),
                             rospy.Time.now(),
                             "left_wheel",
                             "base_link")
            # Relative coordinates from base link to platform base
            br.sendTransform((-0.156, 0, -1*mb.DIAMETER/2 + 0.235 + 0.043),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "platform_base",
                             "base_link")
            # Relative coordinates from platform base to forward laser
            br.sendTransform((0.25 - 0.062/2, 0, 0.0525),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "forward_laser",
                             "platform_base")
            # Relative coordinates from platform base to sideways laser
            br.sendTransform((0.16 + 0.15, 0, 0.8),
                             tf.transformations.quaternion_from_euler(0, math.radians(90), 0),
                             rospy.Time.now(),
                             "sideways_laser",
                             "platform_base")

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