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

  std::string laser_topic = argv[1];
  std::string crash_topic = argv[2];

  ros::init(argc, argv, crash_topic);

  ros::NodeHandle n;

  CrashRecoverer recoverer;

  ros::Subscriber sub =
      n.subscribe(laser_topic, 1, &CrashRecoverer::receiveLaserScan, &recoverer);

  ros::Publisher pub = n.advertise< geometry_msgs::Twist >(crash_topic, 1);

  ros::Rate rate(10);

  while (ros::ok()) {
    pub.publish(recoverer.getResolution());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
