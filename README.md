# Autonomous Multi-Floor Robot

This repository contains all the packages for my final year project at the University of Nottingham. The aim was to develop an AGV capable of navigating in a multi-floor building using elevators to go between floors. The image below shows the robot used for this. Its equipt with a Jetson Nano (running ROS 2 in a Docker container) along with an RPLidar A2M4. 




![image](https://github.com/CraftyCranberry/ros2_ws/assets/82392157/25ea1176-7634-4310-911b-06e8e2ddb827)





## Important command to launch docker with usb and net host
docker run -t -i --privileged -v /dev:/dev --net=host 415cdcab3289 bash


calling map:
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/ros2_ws/src/tarkbot_robot/maps/BlkD/BlkD_Floor1_V2.yaml}"
