#Team 3

## How to run

Primary way of launching the project is running a script from the workspace root:

`./src/team-3/launch.sh`

It is also possible to launch all the commands manually:

`catkin_make`
`roscore`
`roslaunch lab_simulator simulator.launch`
`rosrun half_circle_detection half_circle_detection_node`
`rosrun random_walk_strategy random_walk_strategy_node`

Alternatively to using the lab_simulator you can run the following command:
`roslaunch movement movementSimulator.launch`
`rviz -d ``rospack find movement``/rviz/basic.rviz`

If ROS tells you that no package with the name "lab_simulator" exists, you may have forgotten to source your workspace. Fix it by:

`source /home/robotics/ros_ws/devel/setup.bash`

The robot should then start moving in a random walk and eventually drive into the semi-circle.


##Testing

For testing you have to be at the workspace root and then you can run test cases by doing the following:

`catkin_make run_tests`
