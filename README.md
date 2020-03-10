# explorer_robot
implementation of a package for exploration and mapping

# Package required

## Make sure the simulation package is installed:

sudo apt-get install ros-<distro>-husky-simulator

## Set an environmental variable HUSKY_GAZEBO_DESCRIPTION:

export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

# Run

## bring up the robot by command
$ roslaunch explorer_turtle turtlebot_start.launch
## We bring up 3D node, gmapping node, move base node
$ roslaunch turtlebot_gazebo gmapping_demo.launch
## run map create
$ roslaunch explorer_turtle map.launch
## teleop
$ roslaunch turtlebot_teleop keyboard_teleop.launch
