# explorer_robot
implementation of a package for exploration and mapping

# Package required

## Make sure the simulation package is installed:

sudo apt-get install ros-<distro>-husky-simulator

## Set an environmental variable HUSKY_GAZEBO_DESCRIPTION:

export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

# Run

$ roslaunch explorer_turtle robot_start.launch
$ roslaunch explorer_turtle map.launch
