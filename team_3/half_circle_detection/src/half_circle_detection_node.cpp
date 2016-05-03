/** @file half_circle_detection_node.cpp
  * @brief Main executable for half-circle detection.
  *
  * Subscribes to ```base_scan``` for laser data.
  * Publishes to ```half_circle_detection``` for publishing circle positions.
  *
  * @author Felix Schmoll (LiftnLearn)
  */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include "../include/HalfCircleDetector.h"

<<<<<<< HEAD
/** @brief Starts the node.
=======
/** @brief Starts the program.
>>>>>>> feature-crash-recovery
  * @param[in] argc Number of Arguments
  * @param[in] argv Array of Arguments
  *   - [0] Program name
  *   - [1] Topic name for receiving LaserScans
  *   - [2] Topic name for publishing detected half circles
  *   - [3] Topic name for publishing OpenCV images
  *   - ... other ros-specific arguments
  */
int main(int argc, char **argv) {
  ROS_ASSERT_MSG(argc > 5,
                 "Not enough arguments for topic names. 6 expected, %d given.",
                 argc);

  std::cout << argc << std::endl;

  ros::init(argc, argv, argv[1]);

  ros::NodeHandle node(argv[1]);

  float laserRange;
  float minimumDistance;
  int stretchFactor;
  float halfCircleRadius;
  float minCirclePercentage;
  float maxInlierDist;
  float maxCircleDensity;
  int maxCirclePoints;

  ROS_ASSERT(node.getParam("laserRange", laserRange));
  ROS_ASSERT(node.getParam("minimumDistance", minimumDistance));
  ROS_ASSERT(node.getParam("stretchFactor", stretchFactor));
  ROS_ASSERT(node.getParam("halfCircleRadius", halfCircleRadius));
  ROS_ASSERT(node.getParam("minCirclePercentage", minCirclePercentage));
  ROS_ASSERT(node.getParam("maxInlierDist", maxInlierDist));
  ROS_ASSERT(node.getParam("maxCircleDensity", maxCircleDensity));
  ROS_ASSERT(node.getParam("maxCirclePoints", maxCirclePoints));

  HalfCircleDetector detector(laserRange, minimumDistance, stretchFactor,
                              halfCircleRadius, minCirclePercentage,
                              maxInlierDist, maxCircleDensity, maxCirclePoints);

  /** Subscribers */
  ros::Subscriber sub = node.subscribe(
      "/base_scan", 1, &HalfCircleDetector::receiveLaserScan, &detector);

  /** Publishers */
  ros::Publisher posePub = node.advertise< geometry_msgs::Pose2D >(argv[2], 1);
  image_transport::ImageTransport it(node);
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
