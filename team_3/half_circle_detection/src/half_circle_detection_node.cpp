/** @file half_circle_detection_node.cpp
  * @brief Main executable for half-circle detection.
  * Subscribes to ```base_scan``` for laser data.
  * Publishes to ```half_circle_detection``` for publishing circle positions. 
  * @author Felix Schmoll (LiftnLearn) 
  */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include "../include/HalfCircleDetector.h"

/** @brief Starts the program. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "half_circle_publisher");

  ros::NodeHandle poseNode;

  HalfCircleDetector detector;

  ros::Subscriber sub = poseNode.subscribe(
      "base_scan", 1, &HalfCircleDetector::receiveLaserScan, &detector);

  ros::Publisher posePub =
      poseNode.advertise<geometry_msgs::Pose2D>("half_circle_detection", 1);

  ros::NodeHandle imageHandle;
  image_transport::ImageTransport it(imageHandle);
  image_transport::Publisher imagePub = it.advertise("laserScan_image", 1);

  ros::Rate rate(10);

  while (ros::ok()) {
    posePub.publish(detector.getHalfCirclePose());

    sensor_msgs::ImagePtr imagePtr = cv_bridge::CvImage(std_msgs::Header(), "CV_8U", detector.getLaserScanImage()).toImageMsg();
    imagePub.publish(imagePtr);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
