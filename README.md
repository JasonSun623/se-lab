#Team 3

## How to run

Run all nodes using the following command:
`roslaunch movement movementSimulator.launch`
`rviz -d ``rospack find movement``/rviz/basic.rviz`

If ROS tells you that no package with the name "lab_simulator" exists, you may have forgotten to source your workspace. Fix it by:

`source devel/setup.bash`

The robot should then start moving along the first wall it finds, eventually finding the semicircle.


##Testing

For testing you have to be at the workspace root and then you can run test cases by doing the following:

`catkin_make run_tests`
