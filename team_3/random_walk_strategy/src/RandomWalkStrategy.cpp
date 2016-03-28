/** @file RandomWalkStrategy.cpp
  * Implementation of the corresponding header.
  *
  * @author Leonhard Kuboschek
  * @bug Sometimes drives into walls if placed closely to multiple walls.
*/

#include "RandomWalkStrategy.h"

/**
  * @brief Returns if circles is in sight.
  * @return Returns bool if circle is visible.
  */
bool RandomWalkStrategy::getCircleVisible() { return circleVisible; }

/**
  * @brief Checks if pose represents actual circle and sets values accordingly.
  */
void RandomWalkStrategy::receiveCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePose) {
  // compare to -1
  if (fabs(circlePose->x + 1) < 0.02) {
    circleVisible = false;
    return;
  }

  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));
}

void RandomWalkStrategy::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
   twist = produceControlOutput(laserScan);
}


/**
 * @brief If circle is visible it drives straight towards it. If not it moves
 * either forward or turns if it gets too close to a wall.
 * @return Next move to be done.
 */
const geometry_msgs::Twist RandomWalkStrategy::produceControlOutput(const sensor_msgs::LaserScan::ConstPtr& laserScan) {
  geometry_msgs::Twist msg;

    float min = *std::min_element(laserScan->ranges.begin(), laserScan->ranges.end());

  if (circleVisible) {
    float variation = SCAN_CENTER - circleAngle;

    if (abs(variation) > VARIATION_THRESHOLD) {
      correcting = true;
    }

    // stop correting if alignment sufficient or sufficiently close
    if (abs(variation) < VARIATION_THRESHOLD - HYSTERESIS || min < 0.4) {
      correcting = false;
    }

    ROS_DEBUG("V: %f\tC: %s", variation, correcting ? "true" : "false");

    if (correcting) {
      // turn
      msg.angular.z = std::min(MAX_TURN, TURN_CORRECTION * variation);
      msg.linear.x = LINEAR_VEL / std::min(4.0, abs(variation) / 10.0);
    } else {
      msg.angular.z = 0;
      msg.linear.x = LINEAR_VEL;
    }
  } else {

    if (min > MIN_DISTANCE) {
      msg.linear.x = LINEAR_VEL / 2;
      msg.angular.z = 0;
      turning = false;
    } else {

      if(!turning) {
        turning = true;
        srand(time(0));
        int randomVal = rand()%2;
        if(randomVal == 0) {
          correctingSign = 1;
        } else {
          correctingSign = -1; 
        }
      }
      msg.linear.x = 0;
      msg.angular.z = correctingSign * SCAN_VELOCITY;
    }
  }

  return msg;
}

const geometry_msgs::Twist RandomWalkStrategy::getControlOutput(){

  return twist;

}
