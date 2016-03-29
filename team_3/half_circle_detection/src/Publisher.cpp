#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include "../include/HalfCircleDetector.h"

/** Main executable for half-circle detection.
 *
 *  Subscribes to ```base_scan``` for laser data.
 *  Publishes to ```half_circle_detection``` for publishing circle positions. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "half_circle_publisher");

  ros::NodeHandle n;

  HalfCircleDetector *detector = new HalfCircleDetector();

  ros::Subscriber sub = n.subscribe(
      "base_scan", 1, &HalfCircleDetector::receiveLaserScan, detector);

  ros::Publisher pub =
      n.advertise< geometry_msgs::Pose2D >("half_circle_detection", 1);

  ros::Rate rate(10);

  while (ros::ok()) {

    pub.publish(detector->getHalfCirclePose());

    ros::spinOnce();
    rate.sleep();
  }

  delete detector;

  return 0;
}
