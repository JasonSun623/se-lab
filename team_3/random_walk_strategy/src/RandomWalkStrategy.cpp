#include "RandomWalkStrategy.h"

void RandomWalkStrategy::receiveCirclePosition(bool visible, float angle, float distance)
{
  circleVisible = visible;
  circleAngle = angle;
  circleDistance = distance;
}

void RandomWalkStrategy::receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
  lastScan = *laserScan;
}

const geometry_msgs::Twist RandomWalkStrategy::getControlOutput()
{
  geometry_msgs::Twist msg;

  if(circleVisible)
  {
    float variation = SCAN_CENTER - circleAngle;

    if(abs(variation) > VARIATION_THRESHOLD)
    {
      if(variation > 0)
        msg.angular.z = TURN_CORRECTION;
      else
        msg.angular.z = TURN_CORRECTION * -1;
    } else {
        msg.linear.x = LINEAR_VEL;
    }
  } else {
    msg.linear.x = 0;
    msg.angular.z = TURN_CORRECTION;
  }

  return msg;
}
