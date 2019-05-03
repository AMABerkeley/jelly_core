#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class JellyGUI:
    def __init__(self):
        # TODO check with jelly_web that thses topcis are correct
        rospy.Subscriber("/jelly_gui/command", String, self.update_status)
        self.pub = rospy.Publisher("/jelly_gui/status", String, queue_size=1)
        self.mode = 0
        self.cmd = 0

        # initalize
    def update_status(self, msg):
        # TODO figure out how gui commands are specified

        if msg.data == "rolling":
            self.mode = 0
            self.cmd = 0
        elif msg.data == "rolling":
            self.mode = 0
            self.cmd = 0
        elif msg.data == "rolling":
            self.mode = 0
            self.cmd = 0
        elif msg.data == "rolling":
            self.mode = 0
            self.cmd = 0
        else:
            self.mode = 0
            self.cmd = 0

    def read_command(self):
        return self.mode, self.cmd

    def write(self, msg):
        # write to console
        rospy.loginfo(msg)
        s = String()
        s.data = msg
        self.pub.publish(s)


class JellyRobot:
    def __init__(self):
        # set control rate
        self.rate = rospy.Rate(200)

        # collect parameters of robot
        self.joint_names = rospy.get_param("/jelly_hardware/joint_names")
        self.joint_to_idx = {}
        for idx, joint in enumerate(self.joint_names):
            self.joint_to_idx[joint] = idx

        self.base_link   = rospy.get_param("/jelly_hardware/base_link")
        self.fl_link     = rospy.get_param("/jelly_hardware/fl_link")
        self.fr_link     = rospy.get_param("/jelly_hardware/fr_link")
        self.rl_link     = rospy.get_param("/jelly_hardware/rl_link")
        self.rr_link     = rospy.get_param("/jelly_hardware/rr_link")

        self.gear_ratio  = rospy.get_param("/jelly_hardware/gear_ratio")
        self.joint_directions = rospy.get_param("/jelly_hardware/joint_directions")

        # set up publishers for odrive
        self.odrives = rospy.get_param("/jelly_hardware/odrive_ids")
        self.motor_publishers = []

        for odrive in self.odrives:
            id = odrive["id"]
            a1 = odrive["axis1"]
            a2 = odrive["axis2"]
            # TODO change to correct message type and topic
            pi = rospy.Publisher("/jelly_hardware/odrives" + str(id)  +"/command", Float64MultiArray, queue_size=1)
            self.motor_publishers.append(pi) # set up odrive
            pass

        # TODO change to correct message type and topic
        self.vesc_pub = rospy.Publisher("/jelly_hardware/vesc_cmd/command", Float64, queue_size=1)


        # initalize state
        # self.joint_positions  = [0.0] * len(self.joint_names)
        # self.joint_velocities = [0.0] * len(self.joint_names)
        # self.joint_torques    = [0.0] * len(self.joint_names)

        # collect parameters of controller
        self._home_position = rospy.get_param("/jelly_control/home_position")
        self._rolling_position = rospy.get_param("/jelly_control/rolling_position")


    def set_joints(self, cmds):
        for i, odrive in enumerate(self.odrives):
            a1 = odrive["axis1"]
            a2 = odrive["axis2"]
            idx1 = self.joint_to_idx[a1]
            idx2 = self.joint_to_idx[a2]
            # TODO change to correct message type and topic
            pub_i = self.motor_publishers[i] # set up odrive

            # TODO use correct custon message type
            msg = Message()
            msg.data1 = cmds[idx1]
            msg.data2 = cmds[idx2]
            pub_i.publish(msg)

    def update_joints(self, joint_msg):
        # TODO
        pass

    def calibrate(self):
        # TODO
        # insert calibration code
        pass


    def home(self):
        self.set_joints(self._home_position)

    def command(self, mode, command):
        if mode == self.mode:
            if self.mode == 0: # standing mode
                # TODO fill in control for walking
                pass
            elif self.mode == 1: # walking mode
                # TODO fill in control for walking
                pass
            elif self.mode == 2: #rolling  mode
                # TODO fill in control for walking
                pass

        else:
            self.switch_to(mode)

    def switch_to(mode):
        if mode == 2:
            self.set_joints(self._rolling_position)
        else:
            self.home()

if __name__ == '__main__':
    # initialize the robot and GUi
    rospy.init_node('jelly_controller', anonymous=True)
    jelly = JellyRobot()
    gui = JellyGUI()
    gui.write("Jelly setup complete")

    # calibrate jelly
    gui.write("calibrating")
    jelly.calibrate()
    gui.write("done calibrating")

    # initilize to home position
    # jelly.home()

    gui.write("done calibrating")
    while not rospy.is_shutdown():
        # control loop

        # read from gui
        mode, cmd = gui.read_command()

        # jelly.command(mode, cmd)
        jelly.rate.sleep()
