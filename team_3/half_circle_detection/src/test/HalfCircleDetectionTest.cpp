/** @file HalfCircleDetectionTest.cpp
  * @brief Test cases for the half-circle detection.
  * @author Felix Schmoll (LiftnLearn)
  */

/* Includes */

/* gtest include */
#include <gtest/gtest.h>

/* ROS include */
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <ros/package.h>

/* Include header for class to be tested */
#include "../../include/HalfCircleDetector.h"

/** @brief This test case checks on a very basic example if the detector detects
 * an actual semicircle to a certain accuracy. */
TEST(HalfCircleDetectionTestSuite, circleDetectionTestCase) {
  HalfCircleDetector h;

  //read LaserScan-messages from .bag-file
  std::string path = ros::package::getPath("half_circle_detection");
  path += "/src/test/laserScan_HalfCircle.bag";

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"base_scan"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  //ensure semicircle is being detected
  for (rosbag::MessageInstance m : view) {

    sensor_msgs::LaserScan::ConstPtr ptr =
        m.instantiate<sensor_msgs::LaserScan>();
    if (ptr != NULL) {

      h.receiveLaserScan(ptr);

      geometry_msgs::Pose2D pose = h.getHalfCirclePose();

      ASSERT_NEAR(pose.x, 0, 5);
      ASSERT_NEAR(pose.y, 2, 5);
      ASSERT_NEAR(pose.theta, 1.6, 0.2);
    }
  }
}

/** @brief This test case checks if the detector is giving false positives, thus
 * signaling a halfCircle when there is no halfCircle.*/
TEST(HalfCircleDetectionTestSuite, falsePositiveTestCase) {
  HalfCircleDetector h;

  //read LaserScan-messages from .bag-file
  std::string path = ros::package::getPath("half_circle_detection");
  path += "/src/test/laserScan_noHalfCircle.bag";

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"base_scan"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  //certify that no semicircle is being detected
  for (rosbag::MessageInstance m : view) {

    sensor_msgs::LaserScan::ConstPtr ptr =
        m.instantiate<sensor_msgs::LaserScan>();
    if (ptr != NULL) {

      h.receiveLaserScan(ptr);

      geometry_msgs::Pose2D pose = h.getHalfCirclePose();

      ASSERT_NEAR(pose.x, -1, 0.01);
      ASSERT_NEAR(pose.y, -1, 0.01);
      ASSERT_NEAR(pose.theta, -1, 0.01);
    }
  }
}

/** @brief Main testing function */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
