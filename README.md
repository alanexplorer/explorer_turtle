
# Explorer Turtlebot

## Description

This project uses the extended kalman filter to provide the robot's location and builds a grid map with the laser data.


## How Run

### Run Simulation

#### Terminal tab 1
```
> roslaunch explorer_turtle turtlebot_start.launch
```
#### Terminal tab 2
```
> roslaunch explorer_turtle map.launch
```
### Terminal tab 3
```
> roslaunch explorer_turtle aruco_simulator.launch
```
#### Terminal tab 4
```
> rosrun image_view image_view image:=/explorer/marker/image
```
#### Terminal tab 5
```
> roslaunch explorer_turtle localization.launch
```