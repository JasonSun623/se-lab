#ifndef RANDOMWALKSTRATEGY_H
#define RANDOMWALKSTRATEGY_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

#define LINEAR_VEL 0.2

// The angle of circle detector that is straight ahead [deg]
#define SCAN_CENTER 120

// The amount of variation that is allowed before direction is corrected [deg]
#define VARIATION_THRESHOLD 8

// Angular velocity value that is used to turn
#define TURN_CORRECTION 0.2


class RandomWalkStrategy
{
  public:
    void receiveCirclePosition(bool visible, float angle, float distance);
    void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan);

    const geometry_msgs::Twist getControlOutput();

  private:
    bool circleVisible;
    float circleAngle;
    float circleDistance;

    sensor_msgs::LaserScan lastScan;

    enum State {
      STARTUP,
      LOOKING,
      FOUND
    };

    RandomWalkStrategy::State currentState;
};

#endif
