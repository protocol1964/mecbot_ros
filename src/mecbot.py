#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import re
import math


class Mecbot:
    TREAD = 0.47
    PULSE_OF_ROTATION = 120000
    DIAMETER = 0.254
    LENGTH_OF_ROTATION = math.pi * DIAMETER

    # private
    def __init__(self, dev="/dev/ttyUSB0", baud=57600):
        """Mecbot wrapper

        :param str dev: Path of the USB serial device.
        :param int baud: Baudrate
        """

        self.__ser = serial.Serial(dev, baud, timeout=0)
        self.__ser.reset_input_buffer()
        self.__ser.reset_output_buffer()

    def __del__(self):
        self.__ser.close()

    def __write(self, cmd):
        self.__ser.write(cmd + "\n")
        return self.__read()

    def __read(self):
        responce = self.__ser.readline()
        self.__ser.reset_input_buffer()
        return responce
    
    # public
    def pwm_output(self, motor, duty):
        """PWM output

        :param int motor: Number of the motor (1,2)
        :param int duty: Duty cycle (-100~100)
        :raises MecbotRangeError: An out-of-range value has been entered.
        """

        if motor != 1 and motor != 2:
            raise MecbotRangeError("motor")
        if duty < -100 or 100 < duty:
            raise MecbotRangeError("duty")

        self.__write("P" + str(motor) + "DU" + str(duty))
    
    def control_rotation(self, motor, speed):
        """Control angular velocity of the wheel.

        :param int motor: Number of the motor (1,2)
        :param float speed: Angular velocity of the wheel (-100~100)[rad/s]
        :raises MecbotRangeError: An out-of-range value has been entered.
        """

        if motor != 1 and motor != 2:
            raise MecbotRangeError("motor")
        if speed < -100 or 100 < speed:
            raise MecbotRangeError("speed")

        self.__write("RC" + str(motor) + "RS" + str(round(speed, 3)))
        
    def control_forward_speed(self, speed):
        """Control forward speed.

        :param float speed: Forward speed (-30~30)[m/s]
        :raises MecbotRangeError: An out-of-range value has been entered.
        """

        if speed < -30 or 30 < speed:
            raise MecbotRangeError("speed")
        
        self.__write("VCX" + str(round(speed, 3)))
    
    def control_turning_speed(self, speed):
        """Control turning speed.

        :param float speed: Turning speed (-10~10)[rad/s]
        :raises MecbotRangeError: An out-of-range value has been entered.
        """

        if speed < -10 or 10 < speed:
            raise MecbotRangeError("speed")

        self.__write("VCR" + str(round(speed, 3)))
    
    def measure_speed(self):
        """Measure the speed of both wheels

        :return float: Speed of both wheels [m/s]
        :raises MecbotMeasureError: Measure failed.
        """

        responce = self.__write("MEV")
        if not re.search("\$MEV:", responce):
            raise MecbotMeasureError()
        search_result = map(float, re.findall("([-]?\d+\.\d+|[-]?\d+)", responce))

        if len(search_result) < 4:
            raise MecbotMeasureError()
        else:
            return search_result[0], search_result[1], search_result[2], search_result[3]

    def measure_pulse(self):
        """Measure the encoder pulse of both wheels

        :return int: Pulse of both wheels [m/s]
        :raises MecbotMeasureError: Measure failed.
        """

        responce = self.__write("ME")
        if not re.search("\$ME:", responce):
            raise MecbotMeasureError()

        search_result = map(int, re.findall("([-]?\d+\.\d+|[-]?\d+)", responce))

        if len(search_result) < 4:
            raise MecbotMeasureError()
        else:
            return search_result[0], search_result[1], search_result[2], search_result[3]

    def calc_pos(self, init_x, init_y, init_theta, pulse_r, pulse_l):
        """Calculate Mecbot odometry

        :param float init_x:
        :param float init_y:
        :param float init_theta:
        :param int pulse_r:
        :param int pulse_l:
        :return:
        """
        delta_x_r = float(pulse_r) / self.PULSE_OF_ROTATION * self.LENGTH_OF_ROTATION
        delta_x_l = float(pulse_l) / self.PULSE_OF_ROTATION * self.LENGTH_OF_ROTATION
        delta_x = (delta_x_r + delta_x_l) / 2
        delta_theta = (delta_x_r - delta_x_l) / self.TREAD

        theta = delta_theta + init_theta
        x = delta_x * math.cos(theta) + init_x
        y = delta_x * math.sin(theta) + init_y

        return x, y, theta


class MecbotError(Exception):
    pass


class MecbotRangeError(MecbotError):
    def __init__(self, arg):
        self.__arg = arg

    def __str__(self):
        return "An out-of-range value has been entered. (" + self.__arg + ")"


class MecbotMeasureError(MecbotError):
    def __str__(self):
        return "Measure failed."


def main():
    pass


if __name__ == "__main__":
    main()