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

#define CRITICAL_DISTANCE 0.1 // in meters

/**
  * @brief Compound class for handling critical corner situations.
  */
class CornerHandler {
public:
  /**
    * @brief Callback function for corner handling.
    */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
   * @brief Setter for latest computed corner handling twist.
   */
  void setTwist(geometry_msgs::Twist &twist);

  /**
   * @brief Getter for latest computed corner handling twist.
   */
  geometry_msgs::Twist getTwist();

private:
  /**
    * @brief Latest computed twist.
    */
  geometry_msgs::Twist twist;

  /** @brief Detects critical situations (the robot is too close to an obstacle.
    * @param laserScan LaserScan-message containing obstacle-distances */
  bool detectCorner(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** @brief Handles critical corner situations. Should slowly move the robot
    * out again without actually touching anything.
    * This module is basically just supposed to handle situations when the
 * robot is put into a corner as a starting position. The most basic case here
 * implemented would then be not to bump into the sidewall when turning  away
 * from the same wall.
    * @param laserScan LaserScan-message containing obstacle-distances
    * @return Returns a Twist-message that has all fields set to 0 if no corner
   * is detected and otherwise returns the next suggested move.
    */
  geometry_msgs::Twist
  handleCorner(const sensor_msgs::LaserScan::ConstPtr &laserScan);
};

#endif
