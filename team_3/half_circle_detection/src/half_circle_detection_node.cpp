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

/** @brief Starts the program. 
  * @param[in] argc Number of Arguments
  * @param[in] argv Array of Arguments 
  *   - [0] Program name
  *   - [1] Topic name for receiving LaserScans
  *   - [2] Topic name for publishing detected half circles
  *   - [3] Topic name for publishing OpenCV images
  *   - ... other ros-specific arguments
  */
int main(int argc, char **argv) {
  if (argc < 6) {
    ROS_ERROR("Not enough arguments for topic names. 6 expected, %d given. "
              "Terminating half_circle_detection.",
              argc);
    return 1;
  }

  ros::init(argc, argv, argv[1]);

  ros::NodeHandle poseNode;

  HalfCircleDetector detector;

  ros::Subscriber sub = poseNode.subscribe(
      "/base_scan", 1, &HalfCircleDetector::receiveLaserScan, &detector);

  ros::Publisher posePub =
      poseNode.advertise< geometry_msgs::Pose2D >(argv[2], 1);

  ros::NodeHandle imageHandle;
  image_transport::ImageTransport it(imageHandle);
  image_transport::Publisher imagePub = it.advertise(argv[3], 1);

  ros::Rate rate(10);

  while (ros::ok()) {
    posePub.publish(detector.getHalfCirclePose());

    sensor_msgs::ImagePtr imagePtr = cv_bridge::CvImage(std_msgs::Header(), "mono8", detector.getLaserScanImage()).toImageMsg();
    imagePub.publish(imagePtr);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
