/** @file CornerHandler.h
  * Should detect critical situations and maneuvre out of them. There should
 * also be some mechanism that prevents the robot from driving back into the
 * same corner.
  * @author Felix Schmoll (LiftnLearn)
  */

#ifndef CORNERHANDLER_H
#define CORNERHANDLER_H

/** Includes */

/** include ROS */
#include <ros/ros.h>

/** include messages */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define CRITICAL_DISTANCE 0.3

/**
  * @brief Compound class for handling critical corner situations.
  */
class CornerHandler {
public:
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  void setTwist(geometry_msgs::Twist &twist);
  geometry_msgs::Twist getTwist();

private:
  geometry_msgs::Twist twist;

  /** @brief Detects critical situations (the robot is too close to an obstacle.
    * @param laserScan LaserScan-message containing obstacle-distances */
  bool detectCorner(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** @brief Handles critical corner situations. Should slowly move the robot
    * out again without actually touching anything.
    * @param laserScan LaserScan-message containing obstacle-distances
    * @return Returns a Twist-message that has all fields set to 0 if no corner
   * is detected and otherwise returns the next suggested move.
    */
  geometry_msgs::Twist
  handleCorner(const sensor_msgs::LaserScan::ConstPtr &laserScan);
};

#endif
