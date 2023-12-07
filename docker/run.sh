#!/bin/bash
set -e
set -u

xhost +
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --network=host --name=frankapy_container frankapy
xhost -
