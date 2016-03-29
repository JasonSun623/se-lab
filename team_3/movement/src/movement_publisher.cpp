#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "../include/laser_publisher.h"
#include "../include/movement_controller.h"
#include "../include/Robot.h"

#define LINEAR_VELOCITY 0.2

Robot *Robot::robot;

int main(int argc, char **argv) {
  // creates an instance of a robot class
  Robot *r;
  r = Robot::getInstance();

  r->initialize();

  ros::init(argc, argv, "movement_publisher");

  ros::NodeHandle n;
  // initial data for laser publisher
  int num_readings = 100;
  double frequency = 40;

  LaserScanPublisher *laserWrap =
      new LaserScanPublisher(num_readings, frequency);

  MovementController *mCtr = new MovementController();

  // publishers to cmd_vel for movement and scan for laser samples to be
  // displayed
  ros::Publisher pub = n.advertise< geometry_msgs::Twist >("cmd_vel", 100);
  ros::Publisher scan_pub = n.advertise< sensor_msgs::LaserScan >("scan", 50);

  // subscriber to the base_scan topic for laser samples
  ros::Subscriber sub =
      n.subscribe("base_scan", 10, &MovementController::receiveLaserScan, mCtr);

  // listener to get a transform of a robot in xy-coordinate frame
  tf::TransformListener listener;

  ros::Rate rate(10);

  while (ros::ok()) {
    // gives a laser publisher initial configuration
    laserWrap->initializeLaser(ros::Time::now());

    tf::StampedTransform transform;

    // creating a message to cmd_vel and sending it to be executed
    geometry_msgs::Twist msg;
    geometry_msgs::Point pos;
    sensor_msgs::LaserScan scan = mCtr->getScan();

    float minim = mCtr->findMinim(scan, 250);

    // stop when the obstacle is found otherwise move forward
    if (minim < 0.5)
      r->turn(msg, 0.5);
    else
      r->driveX(msg, LINEAR_VELOCITY);

    // set the position according to the initial position to the robot object
    try {
      // ROS_INFO("Attempting to read pose...");
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);

      // ROS_INFO("Got a transform! x = %f, y =
      // %f",transform.getOrigin().x(),transform.getOrigin().y());
      r->setX((-1) * transform.getOrigin().x());
      r->setY((-1) * transform.getOrigin().y());
    } catch (tf::TransformException ex) {
      ROS_ERROR("Nope! %s", ex.what());
    }

    pub.publish(msg);

    scan_pub.publish(laserWrap->getScan());
    ros::spinOnce();
    rate.sleep();
  }

  delete laserWrap;
  delete mCtr;

  return 0;
}