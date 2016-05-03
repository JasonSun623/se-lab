#Team 3

## How to run

Run all nodes using the following command:
`roslaunch wall_following_strategy wallFollowingSimulator.launch`
`rviz -d ``rospack find movement``/rviz/basic.rviz`

If you want to change the world for the simulator, please choose between lab-competition01.world, lab-competition02.world and lab-competition03.world(line 11 in `wallFollowingSimulator.launch`).
Keep in mind that for successful run you need to change halfCircleRadius parameter in `wallFollowingSimulator.launch`:
* lab-competition02.world has value `0.25`
* lab-competition02.world has value `0.25`
* lab-competition03.world has value `0.18`

If ROS tells you that no package with the name "lab_simulator" exists, you may have forgotten to source your workspace. Fix it by:

`source devel/setup.bash`

The robot should then start moving along the first wall it finds, eventually finding the semicircle.


##Testing

For testing you have to be at the workspace root and then you can run test cases by doing the following:

`catkin_make run_tests`
