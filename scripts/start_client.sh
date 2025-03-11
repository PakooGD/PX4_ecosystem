#!/bin/bash
gnome-terminal -- ./scripts/start_px4.sh
gnome-terminal -- ./scripts/start_ros2.sh
gnome-terminal -- ./scripts/start_micro_xrce_dds.sh
gnome-terminal -- ./scripts/start_foxglove.sh
gnome-terminal -- ./scripts/start_qgc.sh