#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "Robot.h"

Robot * Robot::robot;

int main(int argc, char** argv)
{
  Robot *r;
  r = Robot::getInstance();

  ros::init(argc, argv, "transform_publisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  double x = 0;
  double y = 0;
  double phi = 0;
  
  // initial rate of change in x, y and time (imitation of a circle movement)
  double vx = 0.1;
  double vy = -0.1;
  double vPhi = 0.1;

  double delta_x = 0;
  double delta_y = 0;
  double delta_phi = 0.1;

  double ranges[100];
  double intensities[100];

  int count = 0;

  ros::Rate rate(1);

  while( n.ok() )
  {
    ros::Time scan_time = ros::Time::now();

    for(unsigned int i = 0; i < 100; ++i){
       ranges[i] = count;
       intensities[i] = 100 + count;
    }
    
    r->setX(x);
    r->setY(y);
    r->setPhi(phi);
    r->setVelX(vx);
    r->setVelY(vy);

    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / 100;
    scan.time_increment = (1 / 40) / 100;
    scan.range_min = 0.0;
    scan.range_max = 100.0;
   
    scan.ranges.resize(100);
    scan.intensities.resize(100);
    
    for(unsigned int i = 0; i < 100; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }


    if (phi > 1.5){
      delta_x = (vx * cos(vPhi) - vy * sin(vPhi))/10;
      delta_y = (vy * sin(vPhi) + vx * cos(vPhi))/10;
      phi = 0;
    }

    x+= delta_x;
    y+= delta_y;
    phi+= delta_phi;

    // creating a message to cmd_vel and sending it to be executed
    geometry_msgs::Twist msg;

    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = phi;

    pub.publish(msg);
    ROS_INFO("Position and turning angle of the robot: \n    x:%lf\n    y:%lf\n    phi: %lf\n",x,y,phi);

    scan_pub.publish(scan);
    ++count;
    rate.sleep();
  }

  return 0;
}
