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
 * @brief Number of laser samples
 */
#define RANGES 250


/**
  * @brief Compound class for resolving crashes
  */
class CrashRecoverer {
public:
  /**
    * @brief Setter for LaserScans.
    * @param laserScan LaserScan with distances to walls.
    * @return No return value.
    */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
    * @brief Contains the actual logic of the strategy.
    * Decides on where to move next and by how much.
    * @return Next move to be done.
    */
  const geometry_msgs::Twist getControlOutput();



private:
  /**
    * @brief Last received LaserScan.
    */
  sensor_msgs::LaserScan lastScan;

  /**
    * @brief Find the smallest laser scan reading
    * @return Minimum distance to an object.
    */
  float findMinim(int num_readings);

  /**
    * @brief States of this class
    */
  enum State { OK, CRASH };

  /**
    * @brief Current state of the class
    */
  CrashRecoverer::State currentState;

  /**
    * @brief Timer used to run crash recovery maneuver
    */
  int recoveryTimer = 0;
};

#endif
