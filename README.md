# Turtlebot3 Project in ROS2

This project utilizes the Turtlebot3 Burger and Gazebo for visualization. The base file used for this project is the provided `turtlesim_controller.py`, which has been modified and renamed to `turtlebot3_controller.py`. The changes include remappings and calculations of angles from orientation w,x,y, and z.

## Launching the Project

The project can be launched using the following command:

```
ros2 launch ros2_course turtlebot_launch.py
```

A launch file has been provided for convenience.

## Viewing the Robot

To view the robot, Gazebo needs to be launched first with the following commands:

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## For First Time Users

If you're using this project for the first time, here are a few additional steps to ensure everything is working correctly:

1. **Test if Gazebo is installed correctly:**

```
export 'GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **Test if the Turtlebot can be moved:**

```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

3. **Test if SLAM is working:**

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

4. **Test Map functionality:**

```
ros2 run nav2_map_server map_saver_cli -f ~/map
```

5. **Test Navigator functionality with the map:**

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

Please ensure to follow these steps in order to fully utilize the functionalities of this project.
