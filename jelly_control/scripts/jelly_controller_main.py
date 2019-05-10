#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import jelly_locomotion.jelly_gaits as gaits

class JellyGUI:
    def __init__(self):
        # TODO check with jelly_web that thses topcis are correct
        rospy.Subscriber("/jelly_gui/command", String, self.update_status)
        self.pub = rospy.Publisher("/jelly_gui/status", String, queue_size=1)
        self.msg = "none"

        # initalize
    def update_status(self, msg):
        self.msg = msg.data

    def read_command(self):
        return self.msg

    def write(self, msg):
        # write to console
        rospy.logerr(msg)
        s = String()
        s.data = msg
        self.pub.publish(s)


class JellyRobot:
    def update_joints(self, idx0, idx1):

        def callback(msg):
            j0 = msg.data[0] /  (self.gear_ratio * self.joint_directions[idx0]) - self.motor_zeros[idx0]
            j1 = msg.data[1] /  (self.gear_ratio * self.joint_directions[idx1]) - self.motor_zeros[idx1]
            self.joint_positions[idx0] = j0
            self.joint_positions[idx1] = j1

        return callback

    def calibrate_callback(self, msg):
        self.calibrated = msg.data

    def pd_force_control(self, positions):
        for p_des, p_curr, v_curr in zip(positions, p_curr):




    def mode_callback(self, msg):
        self.set_mode(msg.data)

    def set_mode(self, mode_string):
        clip_thresh = self.clip_threshold / 4.0
        if mode_string == "crab":
            self.leg_mutiplier = np.ones(12)
        if mode_string == "reverse_crab":
            self.leg_mutiplier = np.array([1, -1, -1]*4)
        if mode_string == "normal":
            self.leg_mutiplier = np.hstack(( np.array([1, -1, -1]*4), np.ones(6)))

    def __init__(self):
        # set control rate
        self.rate = rospy.Rate(170)

        # calibration subscriber
        self.calibrated = False
        rospy.Subscriber("/jelly_hardware/calibrate", Bool, self.calibrate_callback)
        self.leg_mutiplier = np.ones(12)
        rospy.Subscriber("/jelly_control/mode", String, self.mode_callback)


        # collect parameters of robot
        self.joint_names = rospy.get_param("/jelly_hardware/joint_names")
        self.starting_joints = rospy.get_param("/jelly_hardware/starting_joints")
        self.joint_to_idx = {}
        for idx, joint in enumerate(self.joint_names):
            self.joint_to_idx[joint] = idx

        self.base_link   = rospy.get_param("/jelly_hardware/base_link")
        self.fl_link     = rospy.get_param("/jelly_hardware/fl_link")
        self.fr_link     = rospy.get_param("/jelly_hardware/fr_link")
        self.rl_link     = rospy.get_param("/jelly_hardware/rl_link")
        self.rr_link     = rospy.get_param("/jelly_hardware/rr_link")

        self.mass = rospy.get_param("/jelly_hardware/mass")

        self.gear_ratio  = rospy.get_param("/jelly_hardware/gear_ratio")
        self.joint_directions = rospy.get_param("/jelly_hardware/joint_directions")

        # set up publishers for odrive
        self.odrives = rospy.get_param("/jelly_hardware/odrive_ids")
        self.motor_publishers = {}


        for odrive in self.odrives:
            id = odrive["id"]
            a0 = odrive["axis0"]
            a1 = odrive["axis1"]
            pi = rospy.Publisher("/jelly_hardware/odrives/" + str(id)  +"/command", Float64MultiArray, queue_size=1)
            self.motor_publishers[id] = pi # set up odrive
            # rospy.logerr(a0)
            # rospy.logerr(a1)

            joint0 = self.joint_to_idx[a0]
            joint1 = self.joint_to_idx[a1]

            # create subscriber
            rospy.Subscriber("/jelly_hardware/odrives/" + str(id)  +"/state", Float64MultiArray, self.update_joints(joint0, joint1))

        # TODO change to correct message type and topic
        self.vesc_pub = rospy.Publisher("/jelly_hardware/vesc/command", Int32, queue_size=1)
        self.js_pub   = rospy.Publisher("/joint_states", JointState, queue_size=1)


        # initalize state
        self.joint_positions  = [0.0] * len(self.joint_names)
        self.motor_zeros      = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_torques    = [0.0] * len(self.joint_names)

        # position set point
        self.joint_positions_cmd  = [0.0] * len(self.joint_names)

        # collect parameters of controller
        self._home_position = rospy.get_param("/jelly_control/home_position")
        self._rolling_position = rospy.get_param("/jelly_control/rolling_position")

        # vesc rpm
        self.speed = rospy.get_param("/jelly_control/speed")
        if self.speed < 0:
            self.speed  = 0
        elif self.speed > 1:
            self.speed = 1

        # force control setings
        self.is_force_control = rospy.get_param("/jelly_control/pd_force_control")
        self.kp = rospy.get_param("/jelly_control/kp")
        self.kd = rospy.get_param("/jelly_control/kd")

        # Initalize Gaits
        self.total_gait_count = rospy.get_param("/jelly_control/total_gait_count")
        self.height = rospy.get_param("/jelly_control/height")
        self.clip_threshold_orig = rospy.get_param("/jelly_control/clip_threshold")
        self.clip_threshold = rospy.get_param("/jelly_control/clip_threshold")
        self.gait_index = 0
        self.mode = 0
        ###################### Walking ###################################
        p1     = np.array(rospy.get_param("/jelly_control/walking_gait/p1"))
        p2     = np.array(rospy.get_param("/jelly_control/walking_gait/p2"))
        offset = np.array(rospy.get_param("/jelly_control/walking_gait/offset"))
        beta   = np.array(rospy.get_param("/jelly_control/walking_gait/beta"))
        p1 = p1 + offset;

        # self.walking_gait = gaits.SimpleWalkingGait(beta, p1, p2, mode="reverse_crab")
        self.walking_gait = gaits.SimpleWalkingGait(beta, p1, p2, mode="crab", height=self.height)
        ######################################################################

        ####################### Turning ###################################
        p1     = np.array(rospy.get_param("/jelly_control/turning_gait/p1"))
        p2     = np.array(rospy.get_param("/jelly_control/turning_gait/p2"))
        flip = np.array([-1.0, -1.0, 1.0])

        p1l = p1
        p2l = p2
        p1r = p1 * flip
        p2r = p2 * flip

        # self.turning_gait = gaits.TurningGait(p1l, p2l, p1r, p2r, mode="reverse_crab")
        self.turning_gait = gaits.TurningGait(p1l, p2l, p1r, p2r, mode="crab", height=self.height)
        ########################################################################

        ####################### Troting ###################################
        # offset = 0.07
        # p1 = np.array([offset + 1.85e-01, 4.5e-02, -3.04e-01])
        # p2 = np.array([-1.85e-01, 4.5e-02, -4.14e-01])
        # gait_controller = gaits.TrotGait(p1, p2, mode="reverse_crab")
        # T_cycle = 400
        ########################################################################

        ######################## SideStep ###################################
        # offset = 0.0
        # p1 = np.array([offset ,-0.1 +  0.5e-01, -4.24e-01])
        # p2 = np.array([offset, -0.1 + -0.5e-01, -4.24e-01])
        # gait_controller = gaits.SimpleSideGait(0.85, p1, p2, mode="reverse_crab")
        # gait_controller.set_height(0.20)
        # T_cycle = 2400
        ###########################################################################

        ######################### Bounding  ###################################
        # offset = 0.07
        # p1 = np.array([offset + 0.95e-01, -0.5e-01, -3.94e-01])
        # p2 = np.array([-1.25e-01        , -0.5e-01, -3.94e-01])
        # gait_controller = gaits.BoundGait(p1, p2, mode=None)
        # gait_controller.set_height(0.09)
        # T_cycle = 300
        #############################################################################

    def calibrate(self):
        # wait until calibrated
        while not rospy.is_shutdown() and not self.calibrated:
            # read from gui
            self.publish_robot_state()
            current_state = self.joint_positions
            jelly.rate.sleep()

        self.motor_zeros = np.array(current_state) - np.array(self.starting_joints)
        self.publish_robot_state()
        self.clip_threshold = self.clip_threshold_orig / 20.0

    def home(self):
        self.joint_positions_cmd = self._home_position

    def set_command(self, mode, command):

        # increment gait
        self.gait_index = (self.gait_index + command*1)%self.total_gait_count
        gait_cmd = float(self.gait_index)/float(self.total_gait_count)

        # command motors appropriately
        if mode == self.mode and not command == 0:
            self.clip_threshold = self.clip_threshold_orig

        if mode == self.mode:
            if self.mode == -1: #rolling  mode
                msg = Int32()
                msg.data = int(8980 + self.speed * (9250 - 8980) * command)
                # write to vesc and joints
                self.vesc_pub.publish(msg)
                self.joint_positions_cmd = self._rolling_position

            elif self.mode == 0: # standing mode
                self.home()

            elif self.mode == 1: # walking mode
                positions = self.walking_gait.step(gait_cmd)
                self.joint_positions_cmd = positions

            elif self.mode == 2: # Turning mode
                positions = self.turning_gait.step(gait_cmd)
                self.joint_positions_cmd = positions

        else:
            self.clip_threshold = self.clip_threshold_orig / 10.0
            self.switch_to(mode)


    def publish_robot_state(self):
        # rospy.logerr("publishing state")
        js = JointState()
        js.name = self.joint_names
        js.position = self.joint_positions
        js.velocity = self.joint_velocities
        js.effort = self.joint_torques
        now = rospy.get_rostime()
        js.header.stamp.secs  = now.secs
        js.header.stamp.nsecs = now.nsecs
        self.js_pub.publish(js)

    def switch_to(self, mode):
        rospy.logerr("switching to mode: " + str(mode))
        switch_time = 1.5
        self.gait_index = 0
        gait_cmd = self.gait_index/self.total_gait_count

        if mode == -1:
            self.joint_positions_cmd = self._rolling_position
            self.set_mode("reverse_crab")
        elif self.mode == 0: # standing mode
            self.home()
            self.set_mode("crab")
        elif self.mode == 1: # walking mode
            positions = self.walking_gait.step(gait_cmd)
            self.joint_positions_cmd = positions
            self.set_mode("crab")
        elif self.mode == 2: # Turning mode
            positions = self.turning_gait.step(gait_cmd)
            self.joint_positions_cmd = positions
            self.set_mode("crab")
        else:
            self.home()
        self.mode = mode

    def actuate(self):
        clip_thresh = self.clip_threshold

        mode_cmd = self.joint_positions_cmd * self.leg_mutiplier

        curr_joints  = np.array(self.joint_positions).copy()
        signed_diff  = np.array(mode_cmd) - curr_joints
        clipped_diff = np.clip(signed_diff, -clip_thresh, clip_thresh)

        joint_set =  curr_joints + clipped_diff
        self._set_joints(joint_set)


    def _set_joints(self, cmds):
        cmds = np.array(cmds)
        # cmds = np.clip(cmds.copy(), -np.pi, np.pi)
        cmds = np.clip(cmds.copy(), -3.0, 3.0)

        for i, odrive in enumerate(self.odrives):
            a0 = odrive["axis0"]
            a1 = odrive["axis1"]
            idx0 = self.joint_to_idx[a0]
            idx1 = self.joint_to_idx[a1]
            pub_i = self.motor_publishers[odrive["id"]] # get motor publisher

            msg = Float64MultiArray()
            pos0 = float(self.gear_ratio) * float(self.joint_directions[idx0]) * (float(cmds[idx0]) + float(self.motor_zeros[idx0]))
            pos1 = float(self.gear_ratio) * float(self.joint_directions[idx1]) * (float(cmds[idx1]) + float(self.motor_zeros[idx1]))

            msg.data = [pos0, pos1]

            # rospy.logerr("m%s - cmd %s | rads %s" %(odrive["id"], str(cmds), str(msg.data)))
            pub_i.publish(msg)

def parse_msg(msg):
    # -1 rolling
    # 0 standing
    # 1 walking front or back
    # 2 walking side to side

    mode = 0
    cmd  = 0

    # Rolling
    if msg == "roll_forward":
        mode = -1
        cmd = 1
    elif msg == "roll_stop":
        mode = -1
        cmd = 0
    elif msg == "roll_back":
        mode = -1
        cmd = -1

    # Standing
    elif msg == "stand_left":
        mode = 0
        cmd = -1
    elif msg == "stand_stop":
        mode = 0
        cmd = 0
    elif msg == "stand_right":
        mode = 0
        cmd = 1

    # Walking
    elif msg == "walk_forward":
        mode = 1
        cmd = 1
    elif msg == "walk_stop":
        mode = 1
        cmd = 0
    elif msg == "walk_back":
        mode = 1
        cmd = -1

    # Turning
    elif msg == "walk_left":
        mode = 2
        cmd = -1
    elif msg == "walk_right":
        mode = 2
        cmd = 1

    return mode, cmd

if __name__ == '__main__':
    # initialize the robot and GUi
    rospy.init_node('jelly_controller', anonymous=True)
    jelly = JellyRobot()
    gui = JellyGUI()
    gui.write("Jelly setup complete")

    # calibrate jelly
    gui.write("calibrating")
    jelly.calibrate()
    jelly.home()
    gui.write("done calibrating")

    # control loop
    while not rospy.is_shutdown():
        # read from gui
        msg = gui.read_command()
        # gui.write("from gui:" + msg)
        mode, cmd = parse_msg(msg)
        # gui.write("mode: " + str(mode))
        #jj gui.write("cmd: " + str(cmd))
        jelly.set_command(mode, cmd)
        jelly.actuate()
        jelly.publish_robot_state()
        jelly.rate.sleep()
