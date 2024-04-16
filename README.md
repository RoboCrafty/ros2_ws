## Important command to launch docker with usb and net host
docker run -t -i --privileged -v /dev:/dev --net=host 415cdcab3289 bash


calling map:
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/ros2_ws/src/tarkbot_robot/maps/BlkD/BlkD_Floor1_V2.yaml}"
