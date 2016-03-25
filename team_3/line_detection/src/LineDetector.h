
#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <memory>

/**
  * @brief Threshold from which on distance values are not considered objects
 * anymore
  */
#define LASER_RANGE 3.9

class LineDetector {
public:
  static int compareStart(cv::Vec4i one, cv::Vec4i two) {
    return (one[0] < two[0]);
  }

  static int compareEnd(std::pair<cv::Vec4i, float> one,
                        std::pair<cv::Vec4i, float> two) {
    return (one.first[2] > two.first[2]);
  }

  static int compareSlope(std::pair<cv::Vec4i, float> one,
                          std::pair<cv::Vec4i, float> two) {
    return (one.second < two.second);
  }

  int compareY(int y1, int y2) {
    if (y1 > y2) {
      return (y1 - y2);
    } else {
      return (y2 - y1);
    }
  }

  float calcSlope(cv::Vec4i);
  cv::Vec4i getAverSlope(std::vector<std::pair<cv::Vec4i, float>>);
  int getDifference(int, int);
  void printLinesImage(cv::Mat, std::vector<cv::Vec4i>);
  void removeLines(std::vector<cv::Vec4i>);
  cv::Mat
  createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  float interpolate(int, int, std::vector<float>);
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  int getNumLines() { return res.size(); }
  sensor_msgs::LaserScan::ConstPtr getLaserScan() const { return lastScan; }
  cv::Mat getImage() { return src; }

  void clearData() { res.clear(); }

  std::vector<cv::Vec4i> getLines() { return res; }

private:
  std::vector<cv::Vec4i> res;
  std::vector<std::pair<int, int>> points;
  sensor_msgs::LaserScan::ConstPtr lastScan;
  cv::Mat src;
};

#endif
