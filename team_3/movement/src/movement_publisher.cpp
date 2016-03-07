#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_publisher.h"
#include "movement_controller.h"
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

  MovementController *mCtr = new MovementController();

  r->initialize();

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  ros::Subscriber sub =
      n.subscribe("base_scan", 10, &MovementController::receiveLaserScan, mCtr);

  ros::Rate rate(10);

  while (ros::ok()) {
    laserWrap->initializeLaser(ros::Time::now());

    // creating a message to cmd_vel and sending it to be executed
    geometry_msgs::Twist msg;
    sensor_msgs::LaserScan scan = mCtr->getScan();

    float minim = mCtr->findMinim(scan, 250);
    cout << minim << endl;

    if (minim < 0.5)
      r->turn(msg, 0.5);
    else
      r->driveX(msg, 0.2);

    pub.publish(msg);
    ROS_INFO("Position and turning angle of the robot: \n    x:%lf\n    "
             "y:%lf\n    phi: %lf\n",
             r->getX(), r->getY(), r->getPhi());

    scan_pub.publish(laserWrap->getScan());
    ros::spinOnce();
    rate.sleep();
  }
  delete laserWrap;
  delete mCtr;

  return 0;
}