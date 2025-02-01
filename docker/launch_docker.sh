#!/bin/bash
xhost +local:

if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi

docker run -it --rm ${GPU_OPT} --net host \
        --privileged -v /dev/bus/usb:/dev/bus/usb \
        --env="DISPLAY" \
        -v $HOME/.Xauthority:/root/.Xauthority:rw \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /tmp/argus_socket:/tmp/argus_socket \
	--workdir="/root/colcon_ws" \
        --volume="$(pwd)/../colcon_ws:/root/colcon_ws" \
        corejp-ros2-image:latest
