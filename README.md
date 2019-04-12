# Jelly
A repo for a ros based walking robot named jelly


## Jelly_Core
This includes the ros packages to control the robot.

### Setup ROS Workspace

`mkdir -p jelly_ws/src`\
`cd jelly_ws/src`\
`catkin_init_workspace`\
`git clone https://github.com/AMABerkeley/jelly.git`\
`cd jelly`\
`git submodule init`\
`git submodule update`

## Jelly_Mechanical
This includes all files to build a jelly robot.

## Jelly_Cypress_Workspace
This includes the PSoC Creator project for interfacing between the ODroid and VESC.
