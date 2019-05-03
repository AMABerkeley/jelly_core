#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class JellyRobot:
    def __init__(self):
        # set control rate
        self.rate = rospy.Rate(200)

        # collect parameters of robot
        self.joint_names = rospy.get_param("/jelly_hardware/joint_names")
        self.base_link   = rospy.get_param("/jelly_hardware/base_link")
        self.fl_link     = rospy.get_param("/jelly_hardware/FL_lower_leg")
        self.fr_link     = rospy.get_param("/jelly_hardware/FR_lower_leg")
        self.rl_link     = rospy.get_param("/jelly_hardware/RL_lower_leg")
        self.rr_link     = rospy.get_param("/jelly_hardware/RR_lower_leg")

        self.gear_ratio  = rospy.get_param("/jelly_hardware/gear_ratio")
        self.joint_directions = rospy.get_param("/jelly_hardware/joint_directions")


        # set up publishers for odrive
        self.leg_odrive_map   = rospy.get_param("/jelly_hardware/leg_odrive_id")
        self.leg_odrive_axis  = rospy.get_param("/jelly_hardware/leg_odrive_axis")
        self.motor_publishers = []

        for odrive in self.leg_odrive_map:
            pi = rospy.Publisher("/jelly_hardware/odrives" + str(odrive)  +"/command", Float64MultiArray, queue_size=1)
            self.motor_publishers.append(pi) # set up odrive
            # some command
            pass

        self.vesc_pub = rospy.Publisher("/jelly_hardware/odrives" + str(odrive)  +"/command", Float64MultiArray, queue_size=1)

        # set subscribers and publishers
        #  rospy.Subscriber("/joint_states", JointState, self.update_joints)
        #  rospy.Subscriber("/motor_states", MotorState, self.update_motors)

        # initalize state
        self.joint_positions  = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_torques    = [0.0] * len(self.joint_names)

        # collect parameters of controller
        self._home_position = rospy.get_param("/jelly_control/home_position")
        self._rolling_position = rospy.get_param("/jelly_control/rolling_position")


    def set_joints(self, cmds):
        # TODO figure out how odrive topics will be layed out
        j_cmd = Float64MultiArray()
        j_cmd.data = cmds

        cmd_idx = 0
        for pub in self.motor_publishers:
            self.cmd.publish(j_cmd)

    def update_joints(self, joint_msg):
        self.joint_positions  = [0.0] * len(self.joint_names)
        for i, n in enumerate(joint_msg.name):
            for j, jn in enumerate(self.joint_names):
                if jn == n:
                    self.joint_positions[j] = joint_msg.position[i]
                    continue
                #  else:
                    #  rospy.logerr('No Valid Joint Found')

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
    # initialize the robot
    rospy.init_node('jelly_controller', anonymous=True)
    jelly = JellyRobot()
    rospy.loginfo('Jelly setup complete')

    # calibrate jelly
    jelly.calibrate()

    # initilize to home position
    jelly.home()

    # gui = GUI()
    # set up gui object that reads from commands from gui

    while not rospy.is_shutdown():
        
        # control loop

        # read from gui
        # mode, cmd = gui.read_command()

        # jelly.command(mode, cmd)
        jelly.rate.sleep()
