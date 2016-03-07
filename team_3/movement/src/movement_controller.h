#include <cstdlib>
#include <iostream>
using namespace std;
#ifndef CONTROLLER_H
#define CONTROLLER_H
class MovementController {
private:
  sensor_msgs::LaserScan lastScan;

public:
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan) {
    lastScan = *laserScan;
  }

  sensor_msgs::LaserScan getScan() { return lastScan; }

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