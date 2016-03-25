
#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

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

  float calcSlope(cv::Vec4i);
  cv::Vec4i getAverSlope(std::vector<std::pair<cv::Vec4i, float>>);
  int getDifference(int, int);
  void printLinesImage(cv::Mat, std::vector<cv::Vec4i>);
  void removeLines(std::vector<cv::Vec4i>);
  cv::Mat
  createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  int getNumLines() { return res.size(); }

  std::vector<cv::Vec4i> getLines() { return res; }

private:
  std::vector<cv::Vec4i> res;
};

#endif
