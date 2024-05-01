# Autonomous Multi-Floor Robot

This repository contains all the packages for my final year project at the University of Nottingham. The aim was to develop an AGV capable of navigating in a multi-floor building using elevators to go between floors. The image below shows the robot used for this. It's equipt with a Jetson Nano (running ROS 2 in a Docker container) and an RPLidar A2M4. The package named "multi_floor_navigator" in the src directory enables a custom behaviour necessary for going between floors using an elevator in combination with ROS 2's navigation stack, Nav2 for autonomous navigation. 





<img src="https://github.com/CraftyCranberry/ros2_ws/assets/82392157/25ea1176-7634-4310-911b-06e8e2ddb827" width=50% height=50%>




## Important command to launch docker with usb and net host
docker run -t -i --privileged -v /dev:/dev --net=host 415cdcab3289 bash


calling map:
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/ros2_ws/src/tarkbot_robot/maps/BlkD/BlkD_Floor1_V2.yaml}"
