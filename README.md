# jelly_core
Software for controlling jelly robot with ros


### bringup_jelly
 * used to bringup robot

### control_jelly
 * used for implementing control

### sensors_jelly
 * used for communicating between motors, motor controllers, IMU


### Highest Layer
Controller
 * Subscriptions
     * joint state of robot for each leg (joint positions a total of 12, 3 for each leg)
     * center of mass state from IMU (orientation, velocity, anglar velocity)
 * Publications
     * joint forces
     * joint positions
     * Note: a service or a flag in the command should allow the higher level controller switch between position and force control

### Leg Level
 * leg has knowlege of the actuator gear ratios
 * publishes joint state of leg(joint position and joint velocity), by taking position and veloctiy readings from motor and converting to joint readings
 * subscribes to command topic for leg that controls either the force or position as specified by the high level controller
     * converts joint torques to actuator torques

# Motor level(but can be in the same node as the leg node)
 * reads motor position and velocity for Odrive
 * takes command motor torque and converts to motor current and write to Odrive

# Joint naming conventions
* Each leg has a ID as well as a corresponding number ==> ID(#)
    * FR(0) - Front Right
    * FL(1) - Front Left
    * RR(2) - Rear Right
    * RL(3) - Rear Left

Each Leg has a hip, upper_leg, lower_leg joints.


The joints for the front right leg is

* FR_hip
* FR_upper_leg
* FR_lower_leg


# ODrive to Joint Mapping

ODrive serial IDs will be parameters in the form `od_FL_leg_id = 207C37823548`.

* od_FL_leg
    * FL_upper_leg
    * FL_lower_leg
* od_FR_leg
    * FR_upper_leg
    * FR_lower_leg
* od_BL_leg
    * BL_upper_leg
    * BL_lower_leg
* od_BR_leg
    * BR_upper_leg
    * BR_lower_leg
* od_F_hip
    * FL_hip
    * FR_hip
* od_B_hip
    * BL_hip
    * BR_hip

