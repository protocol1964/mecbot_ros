#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial, re

class Mecbot:
    """Mecbot wrapper
    """
    # private
    def __init__(self, dev="/dev/ttyUSB0", baud=57600):
        """Mecbot wrapper
        
        Args:
            dev (str): Path of the USB serial device.
            baud (int): baudrate
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
        
        Args:
            motor (int): Number of the motor (1,2)
            duty (int): Duty cycle (-100~100)
        
        Raises:
            MecbotRangeError: An out-of-range value has been entered.
        """
        if motor != 1 and motor != 2:
            raise MecbotRangeError("motor")
        if duty < -100 or 100 < duty:
            raise MecbotRangeError("duty")

        self.__write("P" + str(motor) + "DU" + str(duty))
    
    def control_rotation(self, motor, speed):
        """Control angular velocity of the wheel.
        
        Args:
            motor (int): Number of the motor (1,2)
            speed (float): Angular velocity of the wheel (-100~100)[rad/s]
        
        Raises:
            MecbotRangeError: An out-of-range value has been entered.
        """
        if motor != 1 and motor != 2:
            raise MecbotRangeError("motor")
        if speed < -100 or 100 < speed:
            raise MecbotRangeError("speed")

        self.__write("RC" + str(motor) + "RS" + str(round(speed, 3)))
        
    def control_forward_speed(self, speed):
        """Control forward speed.
        
        Args:
            speed (float): Forward speed (-30~30)[m/s]
        
        Raises:
            MecbotRangeError: An out-of-range value has been entered.
        """
        if speed < -30 or 30 < speed:
            raise MecbotRangeError("speed")
        
        self.__write("VCX" + str(round(speed, 3)))
    
    def control_turning_speed(self, speed):
        """Control turning speed.
        
        Args:
            speed (float): Turning speed (-10~10)[rad/s]
        
        Raises:
            MecbotRangeError: An out-of-range value has been entered.
        """
        if speed < -10 or 10 < speed:
            raise MecbotRangeError("speed")
        
        self.__write("VCR" + str(round(speed, 3)))
        return True
    
    def measure_speed(self):
        """Measure the speed of both wheels
        
        Returns:
            float float: Speed of both wheels [m/s]

        Raises:
            MecbotMeasureError: Measure failed.
        """
        responce = self.__write("MEV")
        search_result = map(float, re.findall("([-]?\d+\.\d+|[-]?\d)", responce))

        if len(search_result) < 2:
            raise MecbotMeasureError()
        else:
            return search_result[0], search_result[1]

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