#!/bin/bash
#docker rmi corejp-ros2-image

docker build --build-arg NUM_THREADS=8 --rm -t corejp-ros2-image .
