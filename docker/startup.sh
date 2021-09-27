#!/bin/bash

# start vnc server if VNC_PORT specified
if [[ ! -z ${VNC_PORT} ]]; then
    echo "Starting x11vnc ..."
    DISPLAY=:10
    export DISPLAY
    x11vnc -rfbport ${VNC_PORT} -display WAIT:cmd=FINDCREATEDISPLAY-Xvfb -geometry 1280x1024x24 -noxrecord -noxfixes -noxdamage -xkb -forever -shared -repeat -capslock -nopw -bg -ncache 10 -ncache_cr &
fi

echo "$CATKIN_WS"
roslaunch -v ${CATKIN_WS}/src/asv_perception.launch