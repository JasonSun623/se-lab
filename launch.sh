#!/bin/bash

# Build and launch our prototype

catkin_make
catkin_make run_tests

roslaunch lab_simulator simulator.launch &

rosrun half_circle_detection half_circle_detection_node &
rosrun random_walk_strategy random_walk_strategy_node &
