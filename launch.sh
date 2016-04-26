#!/bin/bash

# Build and launch our prototype

catkin_make
catkin_make run_tests

source ./devel/setup.bash

roslaunch lab_simulator simulator.launch &
roslaunch wall_following_strategy wallFollowing.launch
