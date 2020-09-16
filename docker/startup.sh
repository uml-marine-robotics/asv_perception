#!/bin/bash

# start vnc server if VNC_PORT specified
if [[ ! -z ${VNC_PORT} ]]; then
    DISPLAY=:10
    export DISPLAY
    x11vnc -rfbport ${VNC_PORT} -display WAIT:cmd=FINDCREATEDISPLAY-Xvfb -geometry 1280x1024x24 -noxrecord -noxfixes -noxdamage -xkb -forever -shared -repeat -capslock -nopw -bg -ncache 10 -ncache_cr &
fi

roslaunch ${CATKIN_WS}/src/asv_perception.launch