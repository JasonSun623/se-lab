#include <cstdlib>
#include <iostream>
using namespace std;
#ifndef CONTROLLER_H
#define CONTROLLER_H
/** @file movement_controller.h
 *  @brief Introduces a controller for a robot based on LaserScan data
 *
 *  @author Mariia Gladkova
 */
class MovementController {
private:
  sensor_msgs::LaserScan lastScan;

public:
  /**
   * @brief callback function for subscriber to get laser samples
   */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan) {
    lastScan = *laserScan;
  }

  /**
   * @brief Returns the sensor message for movement controller
   * @return sensor message
   */

  sensor_msgs::LaserScan getScan() { return lastScan; }

  /**
   * @brief Finds the minimum laser sample
   * @param laserscan message and number of laser samples
   * @return minimum value
   */
  float findMinim(sensor_msgs::LaserScan scan, int num_readings) {
    scan.ranges.resize(num_readings);
    float minim = scan.ranges[0];

    for (int i = 0; i < num_readings; i++) {
      minim = min(minim, scan.ranges[i]);
    }

    return minim;
  }
};
#endif