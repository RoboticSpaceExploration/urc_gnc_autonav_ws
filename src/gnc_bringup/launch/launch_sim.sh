#!/bin/bash
# This script launches multiple ROS launch files

# Start the ROS master
roscore &
sleep 5

# Launch the first launch file
roslaunch gnc_bringup gnc_simulation.launch &
sleep 2

# Launch the second launch file
roslaunch gnc_bringup gnc_core.launch &
sleep 2

# Add more launch commands as needed

roslaunch gnc_bringup gnc_spawn.launch &
sleep 2
# Wait for all jobs to finish
wait

