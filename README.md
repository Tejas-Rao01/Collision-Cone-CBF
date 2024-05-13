# Collision-Cone-CBF

## This repo contains the code for Collision Cone Control Barrier Functions (C3BFs), based on this paper. Code is provided for both Bicycle and Unicycle model robots. 




# Installation

### The code was written to run on ROS Noetic on ubuntu 20.04. 
### Please run the following commands. 

```
cd ~/catkin_ws/src
```

```
git clone https://github.com/Tejas-Rao01/Collision-Cone-CBF.git
```
```
cd ~/catkin_ws 
```
```
catkin_make
```
```
source devel/setup.bash
```



# Running 

## Gazebo Simulations
### Unicycle with single obstacles
```
roslaunch turtlebot3_gazebo turtlebot3_single_obstacle.launch
```

### Unicycle with multiple obstacles
```
roslaunch turtlebot3_gazebo turtlebot3_multi_obstacle.launch
```
### Bicycle with single obstacles
```
roslaunch ackermann_vehicle_gazebo ackermann_single_obstacle.launch
```

### Bicycle with multiple obstacles
```
roslaunch ackermann_vehicle_gazebo ackermann_multi_obstacle.launch
```


