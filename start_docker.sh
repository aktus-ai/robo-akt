#!/usr/bin/bash 

# docker build -t ros2-image .


docker run -it --rm -p 8885:8885 -p 23:22   -v ${PWD}:/home -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix   ros2-image

