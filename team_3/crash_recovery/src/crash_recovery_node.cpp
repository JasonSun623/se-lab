/** @file crash_recovery_node.cpp
 *  @brief Main executable of crash_recovery_noe
 *
 *  @author Leonhard Kuboschek (kuboschek)
 *  @author Felix Schmoll (LiftnLearn)
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include "../include/CrashRecoverer.h"


/** Main executable for crash resolution advice node.
 *
 *  Subscribes to ```base_scan``` for laser data.
 *  Publishes to ```crash_recovery``` for publishing resolution advice. */
int main(int argc, char **argv) {
  ROS_ASSERT(argc > 4);

  std::string crash_topic = argv[1];
  std::string laser_topic = argv[2];

  ros::init(argc, argv, crash_topic);

  ros::NodeHandle node(argv[1]);

  float crash_distance;
  int recovery_steps;

  ROS_ASSERT(node.getParam("CRASH_DISTANCE", crash_distance));
  ROS_ASSERT(node.getParam("RECOVERY_STEPS", recovery_steps));

  CrashRecoverer recoverer(crash_distance, recovery_steps);

  ros::Subscriber sub =
      node.subscribe(laser_topic, 1, &CrashRecoverer::receiveLaserScan, &recoverer);

  ros::Publisher pub = node.advertise< geometry_msgs::Twist >(crash_topic, 1);

  ros::Rate rate(10);

  while (ros::ok()) {
    pub.publish(recoverer.getResolution());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
