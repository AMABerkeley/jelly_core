# jelly_core
Software for controlling jelly robot with ros

## Setup

`mkdir -p catkin_ws/src`\
`cd catkin_ws/src`\
`catkin_init_workspace`\
`git clone https://github.com/but-i-love-pbj/jelly_core.git`\
'git submodule update'


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
