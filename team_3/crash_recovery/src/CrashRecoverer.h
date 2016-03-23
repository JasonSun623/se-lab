/** @file CrashRecoverer.h
 *  @brief Determines wether or not a crash has occured, and tries to resolve it.
 *
 *  Provides an interface that gives resolution advice if a crash is detected.
 *
 *  Returns a Twist message containing zeros when all is good, otherwise
 *  a Twist message indicating how to move, to resolve it.
 *
 * @author Leonhard Kuboschek
 */

#ifndef CRASHRECOVERER_H
#define CRASHRECOVERER_H

/* -- Includes -- */

/* ROS include */
#include <ros/ros.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

/**
  * @brief Compound class for resolving crashes
  */
class CrashRecoverer {
public:


private:

};

#endif
