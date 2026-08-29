#!/bin/bash
echo "Cleaning up any leftover processes..."
pkill -9 -f "gz sim"
pkill -9 -f slam_toolbox
pkill -9 -f nav2
pkill -9 -f ekf_node
pkill -9 -f system_mode_manager
pkill -9 -f safety_monitor
sleep 2
echo "Launching rover stack..."
ros2 launch my_rover_nodes full_stack.launch.py
