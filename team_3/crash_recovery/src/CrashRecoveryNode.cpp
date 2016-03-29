#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include "CrashRecoverer.h"

/** Main executable for crash resolution advice node.
 *
 *  Subscribes to ```base_scan``` for laser data.
 *  Publishes to ```crash_recovery``` for publishing resolution advice. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "crash_recovery");

  ros::NodeHandle n;

  CrashRecoverer *recover = new CrashRecoverer();

  ros::Subscriber sub = n.subscribe(
      "base_scan", 1, &CrashRecoverer::receiveLaserScan, recover);

  ros::Publisher pub =
      n.advertise<geometry_msgs::Pose2D>("crash_recovery", 1);

  ros::Rate rate(10);

  while (ros::ok()) {

    pub.publish(recover->getResolution());

    ros::spinOnce();
    rate.sleep();
  }

  delete recover;

  return 0;
}
