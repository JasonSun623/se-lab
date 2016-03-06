#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_publisher.h"
#include "Robot.h"

Robot *Robot::robot;

int main(int argc, char **argv) {
  Robot *r;
  r = Robot::getInstance();

  ros::init(argc, argv, "movement_publisher");

  ros::NodeHandle n;
  int num_readings = 100;
  double frequency = 40;
  LaserScanPublisher *laserWrap =
      new LaserScanPublisher(num_readings, frequency);
  r->initialize();

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  ros::Rate rate(1);

  while (n.ok()) {
    laserWrap->initializeLaser(ros::Time::now());

    // creating a message to cmd_vel and sending it to be executed
    geometry_msgs::Twist msg;

    if (r->getPhi() < 16.6)
      r->turn(msg, 1.2);
    else
      r->driveX(msg, 0.5);

    pub.publish(msg);
    ROS_INFO("Position and turning angle of the robot: \n    x:%lf\n    "
             "y:%lf\n    phi: %lf\n",
             r->getX(), r->getY(), r->getPhi());

    scan_pub.publish(laserWrap->getScan());
    rate.sleep();
  }
  delete laserWrap;

  return 0;
}