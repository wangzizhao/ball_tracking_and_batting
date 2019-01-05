#!/bin/bash
v4l2-ctl -d /dev/video0 --set-ctrl=saturation=80
v4l2-ctl -d /dev/video1 --set-ctrl=saturation=50
gnome-terminal -e "python camera_detect.py 1"
gnome-terminal -e "python camera_detect.py 2"