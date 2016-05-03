#include "../../include/CrashRecoverer.h"

/* Includes */

/* gtest include */
#include <gtest/gtest.h>

/* ROS include */
#include <ros/package.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>

TEST(CrashRecovererTestSuite, crashRecoveryTestCase) {
  // TODO Write test for crash recoverer
  CrashRecoverer r(0.05,20);

  // read LaserScan-messages from .bag-file
  std::string path = ros::package::getPath("half_circle_detection");
  path += "/src/test/laserScan_Corner.bag";

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

  std::vector< std::string > topics = {"base_scan"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Count the number of consecutive messages that are non-zero
  int counter = 0, longestStreak = 0;

  // ensure semicircle is being detected
  for (rosbag::MessageInstance m : view) {

    sensor_msgs::LaserScan::ConstPtr ptr =
        m.instantiate< sensor_msgs::LaserScan >();
    if (ptr != NULL) {
      r.receiveLaserScan(ptr);

      geometry_msgs::Twist pose = r.getResolution();

      if (pose.linear.x != 0) {
        counter++;
      } else {
        longestStreak = std::max(counter, longestStreak);
        counter = 0;
      }
    }
  }

  ASSERT_GE(15, longestStreak);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
