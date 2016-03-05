#ifndef RANDOMWALKSTRATEGY_H
#define RANDOMWALKSTRATEGY_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>

#define LINEAR_VEL 1.0

// The angle of circle detector that is straight ahead [deg]
#define SCAN_CENTER 100

// The amount of variation that is allowed before direction is corrected [deg]
#define VARIATION_THRESHOLD 8
#define HYSTERESIS 2

// Angular velocity value that is used to turn
#define TURN_CORRECTION 0.05
#define MAX_TURN 0.8

// Angular velocity for scanning for circle
#define SCAN_VELOCITY 0.5

class RandomWalkStrategy
{
  public:
    void receiveCirclePosition(const geometry_msgs::Pose2D::ConstPtr& circlePose);
    void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan);

    const geometry_msgs::Twist getControlOutput();

  private:
    bool circleVisible;
    float circleAngle;
    float circleDistance;

    bool correcting;

    sensor_msgs::LaserScan lastScan;

    enum State {
      STARTUP,
      LOOKING,
      FOUND
    };

    RandomWalkStrategy::State currentState;
};

#endif
