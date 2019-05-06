#!/usr/bin/env python

# serial_tx_hub subscribes to a String rostopic and writes to the serial port.
# It is used to control the hub motor via pwm through the psoc 5lp.

import rospy
import serial
import sys
from std_msgs.msg import String

# https://github.com/BerkeleyExpertSystemTechnologiesLab/2d-spine-control-hardware/blob/master/ros-spine-control/src/spine_controller/src/serial_tx_fromtopic.py
class SerialTxHubFromTopic:
    def __init__(self, device_name):
        self.serial_tx_startup(device_name)

    def serial_tx_startup(self, device_name):
        # anonymous=False because we want unique nodes
        rospy.init_node("serial_tx_hub_from_topic", anonymous=False)

        baud_rate = 115200; # must be consistent with the psoc UART component
        self.ser = serial.Serial(device_name, baud_rate)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        sub = rospy.Subscriber("/jelly_hardware/vesc/command", String, self.serial_tx_callback)

        print("Opened serial port and created subscriber.")
        print("Now receiving from topic.")

    def serial_tx_callback(self, message):
        print("message received: " + message.data)
        try:
            print("message received: " + message.data)
            self.res = self.ser.write(message.data + "\r") # ascii \r is 13
            print(self.res)

            print("message received: " + message.data)
        except:
            print("Error writing to serial.")

if __name__ == "__main__":
    device_name = "/dev/serial/by-id/usb-Cypress_Semiconductor_Cypress_KitProg_16170ADE002E4400-if02"
    s_tx = SerialTxHubFromTopic(device_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
