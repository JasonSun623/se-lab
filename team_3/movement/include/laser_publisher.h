/** @file laser_publisher.h
 *  @brief Introduces a publisher of the laser scan
 *  Used in case of running movementSimulator instead of lab_simulator
 *
 *  Defines a message to be published to scan topic
 *
 *  @author Mariia Gladkova
 */

#include <cstdlib>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
using namespace std;
#ifndef LASER_H
#define LASER_H

class LaserScanPublisher {
private:
  int num_readings;
  double laser_frequency;
  sensor_msgs::LaserScan scan;

public:
  /** @brief A constructor for LaserScanPublisher
   *  @param number of read laser samples and laser frequency
   */
  LaserScanPublisher(int num_readings, double laser_frequency) {
    this->num_readings = num_readings;
    this->laser_frequency = laser_frequency;
  }

  /**
   * @brief Gets the number of laser samples read
   * @return the number of read samples
   */

  int getNum() { return num_readings; }

  /**
   * @brief Sets the number of laser samples read
   * @param the number of read samples
   */

  void setNum(int num_readings) { this->num_readings = num_readings; }

  /**
   * @brief Gets the frequency of laser
   * @return frequency of the laser
   */

  double getFreq() { return laser_frequency; }

  /**
   * @brief Sets the frequency of laser
   * @param laser frequency
   */

  void setFreq(double laser_frequency) {
    this->laser_frequency = laser_frequency;
  }

  /**
   * @brief Gets the sensor message to be published
   * @return sensor message
   */

  sensor_msgs::LaserScan getScan() { return this->scan; }

  /**
   * @brief Sets the sensor message to be published
   * @param sensor message
   */

  void setScan(sensor_msgs::LaserScan scan) { this->scan = scan; }

  /**
   * @brief Initializes the sensor message at particular time
   * @param time to be initialized in
   */

  void initializeLaser(ros::Time scan_time) {
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / num_readings;
    scan.range_min = 0.0;
    scan.range_max = 100.0;
  }
};
#endif
