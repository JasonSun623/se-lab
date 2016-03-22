/** @file test_CornerHandler.cpp
  * @brief Should test the functionality of overall corner handling.
  * Currently tests for detection of corners and if the minimum distance to the
  *walls decreases in the next step taken.
  *
  * @author Felix Schmoll (LiftnLearn)
  */

/* Includes */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>

/* Include header for class to be tested */
#include "../../include/CornerHandler.h"

/* gtest include */
#include <gtest/gtest.h>

/** @brief Tests if the node is successfully detecting corners and not giving
 * false positives.
  */
TEST(CornerHandlingTestSuite, cornerDetectionTestCase) {
  CornerHandler handler;
  sensor_msgs::LaserScan laserScan;

  rosbag::Bag bag;
  std::string path = ros::package::getPath("corner_handling");
  path += "/src/test/laserScan.bag";
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("base_scan"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (rosbag::MessageInstance m : view) {

    sensor_msgs::LaserScan::ConstPtr ptr =
        m.instantiate<sensor_msgs::LaserScan>();
    if (ptr != NULL) {
      ASSERT_TRUE(handler.detectCorner(ptr));
    }
  }

  bag.close();
}

/** @brief Should somehow test if the distance to the walls is getting bigger/at
 * least not smaller. */
TEST(CornerHandlingTestSuite, wallDistanceTestCase) {}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
