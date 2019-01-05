#!/bin/bash
gnome-terminal -e "./bin/dynamixel_driver -n 2 -d /dev/arm --status-channel DXL_STATUS_ARM1 --command-channel DXL_COMMAND_ARM1 --config-channel DXL_CONFIG_ARM1"
gnome-terminal -e "./bin/dynamixel_driver -n 4 -i 2 -d /dev/arm2 --status-channel DXL_STATUS_ARM2 --command-channel DXL_COMMAND_ARM2 --config-channel DXL_CONFIG_ARM2"
gnome-terminal -e "./bin/dynamixel_driver -n 2 -d /dev/rex1 --status-channel DXL_STATUS_REX1 --command-channel DXL_COMMAND_REX1 --config-channel DXL_CONFIG_REX1"
gnome-terminal -e "./bin/dynamixel_driver -n 2 -d /dev/rex2 --status-channel DXL_STATUS_REX2 --command-channel DXL_COMMAND_REX2 --config-channel DXL_CONFIG_REX2"