/** @file corner_handling.h
  * Should detect critical situations and maneuvre out of them. There should
 * also be some mechanism that prevents the robot from driving back into the
 * same corner.
  * @author Felix Schmoll (LiftnLearn)
  */

#ifndef CORNERHANDLING_H
#define CORNERHANDLING_H

/** Includes */

/** include ROS */
#include <ros/ros.h>

/** include messages */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

/**
  * @brief Compound class for handling critical corner situations.
  */
class CornerHandler {
public:
  /** @brief Detects critical situations (the robot is too close to an obstacle.
    * @param laserScan LaserScan-message containing obstacle-distances */
  bool detectCorner(sensor_msgs::LaserScan &laserScan);

  /** @brief Handles critical corner situations. Should slowly move the robot
   * out again without actually touching anything.
    * @param laserScan LaserScan-message containing obstacle-distances */
  geometry_msgs::Pose2D handleCorner(sensor_msgs::LaserScan &laserScan);

private:
};

#endif
