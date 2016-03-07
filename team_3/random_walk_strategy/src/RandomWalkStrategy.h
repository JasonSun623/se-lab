#ifndef RANDOMWALKSTRATEGY_H
#define RANDOMWALKSTRATEGY_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>

#define LINEAR_VEL 1.0

// The angle of circle detector that is straight ahead [deg]

#define SCAN_CENTER 90

// The amount of variation that is allowed before direction is corrected [deg]
#define VARIATION_THRESHOLD 8
#define HYSTERESIS 2

// Angular velocity value that is used to turn
#define TURN_CORRECTION 0.05
#define MAX_TURN 0.8

// Angular velocity for scanning for circle
#define SCAN_VELOCITY 0.5

// Minimum allowed distance to obstacles
#define MIN_DISTANCE 0.5

/** Implements the random walk strategy.
  * If a half-circle is detected it moves towards it, otherwise it just
 * turns at
  * the same spot.
  */
class RandomWalkStrategy {
public:
  /**
    * Receives a message with a halfcircle-pose or a dummy message.
    * Currently just uses the angle for actual navigation.
    * @param circlePose A message with a supposed circle or x == -1 for a dummy.
    */
  void receiveCirclePosition(const geometry_msgs::Pose2D::ConstPtr &circlePose);

  /**
    * Receives a LaserScan in order to avoid walls.
    * @param laserScan LaserScan with distances to walls.
    */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
    * Cointains the actual logic of the strategy.
    * Decides on where to move next and by how much.
    * @return Next move to be done.
    */
  const geometry_msgs::Twist getControlOutput();

  bool getCircleVisible();

private:
  bool circleVisible;
  float circleAngle;
  float circleDistance;

  bool correcting;

  /**
    * Last received LaserScan.
    */
  sensor_msgs::LaserScan lastScan;

  /**
    * Find the smallest laser scan reading
    */
  float findMinim(int num_readings);

  /**
    * Different states within the strategy.
    */
  enum State { STARTUP, LOOKING, FOUND };

  RandomWalkStrategy::State currentState;
};

#endif
