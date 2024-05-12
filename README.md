# Autonomous Multi-Floor Robot

This repository contains all the packages for my final year project at the University of Nottingham. The aim was to develop an AGV capable of navigating in a multi-floor building using elevators to go between floors. The image below shows the robot used for this. It's equipt with a Jetson Nano (running ROS 2 in a Docker container) and an RPLidar A2M4. The package named "multi_floor_navigator" in the src directory enables a custom behaviour necessary for going between floors using an elevator in combination with ROS 2's navigation stack, Nav2 for autonomous navigation. This is not a step but step guide on how to set it up. I might write that later. 
***

#### Some other packages include:
1. tarkbot_robot: Driver for the base. To allow controlling it using ROS Topics.
2. rplidar_ros  : Driver for the lidar. To allow controlling it using ROS Topics.
3. turtlebot4*  : Contains files which I used to speed up testing of simulations using turtlebot4's already set-up files.

***

<p align=center>
<img  width=50% height=50% src="https://github.com/CraftyCranberry/ros2_ws/assets/82392157/25ea1176-7634-4310-911b-06e8e2ddb827" >
</p>



## Important commands

#### Run Docker
```docker run -t -i --privileged -v /dev:/dev --net=host 415cdcab3289 bash```

#### Call Map
```ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/ros2_ws/src/tarkbot_robot/maps/BlkD/BlkD_Floor1_V2.yaml}"```

#### Send goal
```ros2 topic pub --once /user_goal geometry_msgs/Pose "{position: {x: 4.0, y: -3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" ```


### Important Note:
If you go over the source code of the multi-floor node you may notice I have defined some things like the path to the maps folder or the co-ordinates of the elevator inside the C++ file. This is definitely not ideal and not how I would go about on production-level or final software. I only did this to save time as I was running close to the deadline trying to get it to work. I will definitely try to convert those over to values that can be defined in a parameter (.yaml) file when I get some time in future. 
