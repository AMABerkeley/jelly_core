#!/user/bin/python

# serial_tx_hub subscribes to a String rostopic and writes to the serial port.
# It is used to control the hub motor via pwm through the psoc 5lp.

import rospy
import serial
import sys
from std_msgs.msg import String

# https://github.com/BerkeleyExpertSystemTechnologiesLab/2d-spine-control-hardware/blob/master/ros-spine-control/src/spine_controller/src/serial_tx_fromtopic.py
class SerialTxHubFromTopic:
    def __init__(self, device_name, topic_name):
        self.serial, self.sub = self.serial_tx_startup(device_name, topic_name)

    def serial_tx_startup(self, device_name, topic_name):
        # anonymous=False because we want unique nodes
        rospy.init_node("serial_tx_hub_from_topic", anonymous=False)

        baud_rate = 115200; # must be consistent with the psoc UART component
        ser = serial.Serial(device_name, baud_rate)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        sub = rospy.Subscriber(topic_name, String, self.serial_tx_callback)

        print("Opened serial port and created subscriber.")
        print("Now receiving from topic.")
        return ser, sub

    def serial_tx_callback(self, message):
        print("message received: " + message)
        try:
            ser.write(message + "\r") # ascii \r is 13
        except:
            print("Error writing to serial.")

if __name__ == "__main__":
    s_tx = SerialTxHubFromTopic(sys.argv[1], sys.argv[2])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
