/** @file RandomWalkStrategy.h
  * @brief Receives data for wall-distance and suggested half-circle-position
  *and computes which move to make next.
  *
  * If a half-circle is detected it moves towards it and otherwise it moves
  *forward until it comes to close to an object and turns. It stops after
  *hitting a half-circle.
  *
  * @author Leonhard Kuboschek
  */
#ifndef RANDOMWALKSTRATEGY_H
#define RANDOMWALKSTRATEGY_H

/* -- Includes -- */

/* ROS include */
#include <ros/ros.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

/* libc includes */
#include <cmath>
#include <ctime>
#include <cstdlib>

/**
  * @brief Speed
  */
#define LINEAR_VEL 1.0

/**
 * @brief The angle of circle detector that is straight ahead [deg]
 */
#define SCAN_CENTER 90

/**
 * @brief The amount of variation that is allowed before direction is corrected
 * [deg]
 */
#define VARIATION_THRESHOLD 8

/**
  * @brief The value for which realignment is being done. If the value is
 * smaller we don't correct anymore. [deg]
  */
#define HYSTERESIS 2

/**
 * @brief Angular velocity value that is used to turn
 */
#define TURN_CORRECTION 0.05

/**
  * @brief Upper speed for turning rate
  */
#define MAX_TURN 0.8

/**
 * @brief Angular velocity for scanning for circle
 */
#define SCAN_VELOCITY 0.5

/**
 * @brief Minimum allowed distance to obstacles
 */
#define MIN_DISTANCE 0.5

/**
 * @brief Number of laser samples
 */
#define RANGES 250

/**
 * @brief Deviation for comparing floating point numbers
 */
#define EPSILON 0.05

/**
  * @brief Strategy for Random Walk
  */
class RandomWalkStrategy {
public:
  /**
    * @brief Receives a message with a halfcircle-pose or a dummy message and
   * sets the corresponding class-internal values.
    * @param circlePose A message with a supposed circle or x == -1 for a dummy.
    * @return No return value.
    */
  void receiveCirclePosition(const geometry_msgs::Pose2D::ConstPtr &circlePose);

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
const geometry_msgs::Twist produceControlOutput(const sensor_msgs::LaserScan::ConstPtr& laserScan);

const geometry_msgs::Twist getControlOutput();

  /*
   * @brief Getter returning if half-circle is visible.
   */
  bool getCircleVisible();

private:
  bool circleVisible;
  float circleAngle;
  float circleDistance;

  bool correcting;
  bool turning = false;
  int correctingSign = 1;
  geometry_msgs::Twist twist;
  bool justDriveForward = false;

  /**
    * @brief Different states within the strategy.
    */
  enum State { STARTUP, LOOKING, FOUND };

  RandomWalkStrategy::State currentState;
};

#endif
