# Collision-Cone-CBF

## This repo contains the code for Collision Cone Control Barrier Functions (C3BFs), based on [this paper](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://arxiv.org/pdf/2209.11524&ved=2ahUKEwjIuoWN74mGAxU3-zgGHfGNDjAQFnoECB8QAQ&usg=AOvVaw1tP28HeSzucpQmdhTHB6b3). Code is provided for both Bicycle and Unicycle model robots. Shown below is a video containing hardware demonstrations of the controller for both unicycle and bicycle model robots.


https://github.com/Tejas-Rao01/Collision-Cone-CBF/assets/60615539/561b3520-dc2d-40ce-affd-dffdf57d1339


https://github.com/Tejas-Rao01/Collision-Cone-CBF/assets/60615539/2e33a566-d3da-431b-8edd-0cbffeaae4a4


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

#### To launch the simulation
```
roslaunch turtlebot3_gazebo turtlebot3_single_obstacle.launch
``` 
#### Open a new terminal and enter the following
```
cd ~/catkin_ws/src/scripts/turtlebot/
python3 pid.py
```

### Unicycle with multiple obstacles
#### To launch the simulation
```
roslaunch turtlebot3_gazebo turtlebot3_multi_obstacle.launch
```
#### Open a new terminal and enter the following
```
cd ~/catkin_ws/src/scripts/turtlebot/
python3 pid.py
```

### Bicycle with single obstacles
#### To launch the simulation
```
roslaunch ackermann_vehicle_gazebo ackermann_single_obstacle.launch
```
#### Open a new terminal and enter the following
```
cd ~/catkin_ws/src/scripts/turtlebot/
python3 pid.py
```

### Bicycle with multiple obstacles
#### To launch the simulation

```
roslaunch ackermann_vehicle_gazebo ackermann_multi_obstacle.launch
```
#### Open a new terminal and enter the following
```
cd ~/catkin_ws/src/scripts/turtlebot/
python3 pid.py
```


