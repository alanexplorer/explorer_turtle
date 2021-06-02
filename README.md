# Explorer Turtlebot

  

## Description

  

This project uses the extended kalman filter to provide the robot's location with the fusion of the camera sensor and odometry sensors and builds a grid map with the infra red.

  

## Softwares Version

  

* ROS : Kinetic

* Python: 2.7.12

* OpenCV: 3.3.1-dev

  
  

## How Run

  

### Run Simulation

  

#### Terminal tab 1

```

> roslaunch explorer_turtle turtlebot_start.launch

```

#### Terminal tab 2

```

> roslaunch explorer_turtle aruco_simulator.launch

```

#### Terminal tab 3

```

> rosrun image_view image_view image:=/explorer/marker/image

```

#### Terminal tab 4

```

> roslaunch explorer_turtle localization.launch

```

#### Terminal tab 5

```

> roslaunch explorer_turtle map.launch

```

## TOOLS

