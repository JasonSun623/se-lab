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
    test->setVelX(2.4);
    test->setVelY(4.2);
    ASSERT_EQ(2.48, test->getX());
    ASSERT_EQ(12.44, test->getY());
    ASSERT_EQ(23.2, test->getPhi());
    ASSERT_EQ(2.4, test->getVelX());
    ASSERT_EQ(4.2,test->getVelY());
}
 
TEST(InitializationTest,void)
{
	Robot *test;
	test->initialize();
    ASSERT_EQ(0, test->getX());
    ASSERT_EQ(0, test->getY());
    ASSERT_EQ(0, test->getPhi());
    ASSERT_EQ(0, test->getVelX());
    ASSERT_EQ(0,test->getVelY());
}

TEST(VelocityIncrementTest,void)
{
	Robot *test;
	test->initialize();
	test->incrementVelX();
	test->incrementVelY();
	ASSERT_EQ(1.0, test->getVelX());
    ASSERT_EQ(1.0,test->getVelY());
}

TEST(UpdateTest, void)
{
	Robot *test;
	test->initialize();
	test->updateAll(1.2,2.3,3.4,4.5,5.6);
	ASSERT_EQ(1.2, test->getX());
    ASSERT_EQ(2.3, test->getY());
    ASSERT_EQ(3.4, test->getPhi());
    ASSERT_EQ(4.5, test->getVelX());
    ASSERT_EQ(5.6,test->getVelY());
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}