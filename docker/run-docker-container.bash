#!/bin/bash

# start sharing xhost
# xhost +local:root

# run docker
docker run --rm \
  --net=host \
  --ipc=host \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:$docker/.Xauthority \
  -v $HOME/workspace/digital_twin_rpi4_ws:$HOME/work \
  -e XAUTHORITY=$home_folder/.Xauthority \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -it --name "digital-twin-rpi4" $(id -un)/digital-twin-rpi4
