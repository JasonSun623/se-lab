#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "../src/laser_publisher.h"

#include "../src/Robot.h"

TEST(GettersSettersTest, void) {
  Robot *test;
  test->setX(2.48);
  test->setY(12.44);
  test->setPhi(23.2);
  ASSERT_EQ(2.48, test->getX());
  ASSERT_EQ(12.44, test->getY());
  ASSERT_EQ(23.2, test->getPhi());
}

TEST(InitializationTest, void) {
  Robot *test;
  test->initialize();
  ASSERT_EQ(0, test->getX());
  ASSERT_EQ(0, test->getY());
  ASSERT_EQ(0, test->getPhi());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
