#!/bin/bash
gnome-terminal -e "mjpg_streamer -i 'input_uvc.so -d /dev/video0' -o 'output_http.so -p 8080'"
gnome-terminal -e "mjpg_streamer -i 'input_uvc.so -d /dev/video1' -o 'output_http.so -p 8090'"