/** @file CrashRecoverer.h
 *  @brief Determines wether or not a crash has occured, and tries to resolve
 * it.
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
#include <geometry_msgs/Twist.h>

/**
  * @brief Compound class for resolving crashes.
  */
class CrashRecoverer {
public:
  /**
    * @brief Setter for LaserScans. Calls logic to update output.
    * @param laserScan LaserScan with distances to walls.
    * @return No return value.
    */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
    * @brief Returns latest resolution advice
    * Decides on where to move next and by how much.
    * @return Next move to be done.
    */
  const geometry_msgs::Twist getResolution();

private:
  /**
    * @brief Current output of the algorithm
    */
  geometry_msgs::Twist currentOut;

  /**
    * @brief Find the smallest laser scan reading
    * @return Minimum distance to an object.
    */
  float findMinim(const sensor_msgs::LaserScan::ConstPtr &scan,
                  int num_readings);

  /**
    * @brief States of this class
    */
  enum State { OK, CRASH };

  /**
    * @brief Current state of the class
    */
  CrashRecoverer::State currentState;

  int recoveryTimer;
};

#endif