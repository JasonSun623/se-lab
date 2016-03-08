#include "../src/HalfCircleDetector.h"

/* Includes */

/* gtest include */
#include <gtest/gtest.h>

/* ROS include */
#include<ros/package.h>

TEST(HalfCircleDetectionTestSuite, circleDetectionTestCase) {
  cv::Mat image;

  std::string path = ros::package::getPath("half_circle_detection");
  image = cv::imread(path+"/test/image.jpg", 0); 
  std::cout << image.rows;
  std::cout <<path;

  HalfCircleDetector* h = new HalfCircleDetector();
  
  geometry_msgs::Pose2D pose = h->detectHalfCircle(image);

  ASSERT_NEAR(pose.x, 200, 10);
  ASSERT_NEAR(pose.y, 200, 10);
  ASSERT_NEAR(pose.theta, 2, 10);

  delete h;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
