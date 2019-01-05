#!/bin/bash
gnome-terminal -e "./bin/dynamixel_driver -n 6 -d /dev/arm --status-channel DXL_STATUS_ARM --command-channel DXL_COMMAND_ARM --config-channel DXL_CONFIG_ARM"
gnome-terminal -e "./bin/dynamixel_driver -n 2 -d /dev/rex1 --status-channel DXL_STATUS_REX1 --command-channel DXL_COMMAND_REX1 --config-channel DXL_CONFIG_REX1"
gnome-terminal -e "./bin/dynamixel_driver -n 2 -d /dev/rex2 --status-channel DXL_STATUS_REX2 --command-channel DXL_COMMAND_REX2 --config-channel DXL_CONFIG_REX2"