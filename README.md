## Important command to launch docker with usb and net host
docker run -t -i --privileged -v /dev:/dev --net=host 415cdcab3289 bash
