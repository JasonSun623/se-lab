/** @file LineDetector.h
  * Eliminates the unnecessary lines
  * detected by Hough Transform algorithm
  * Returns an image with the lines after elimination
  * that sufficiently represent all walls' locations
  * in a laser range
  * @author Mariia Gladkova
  */

#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

/** include ROS */
#include <ros/ros.h>
#include <ros/package.h>

/** include messages */
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

/** include OpenCV */
#include <opencv2/opencv.hpp>

/** include C library */
#include <vector>
#include <algorithm>
#include <memory>
#include <stdlib.h>

#define LASER_RANGE 3.9

/**
  * @brief Limiting integers to be within a certain range.
  */

#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

/**
  * @brief Factor to define an image dimensions
  */

#define STRETCH_FACTOR 100

class LineDetector {
public:
  /**
  * @brief Comparison function for std::sort
  * @details Compares the x-coordinates of starting points of the line segments
  * @param one, two Starting and ending position coordinates of line segments
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareStart(cv::Vec4i one, cv::Vec4i two) {
    return (one[0] < two[0]);
  }

  /**
  * @brief Comparison function for std::sort
  * @details Compares the x-coordinates of ending points of the line segments
  * @param one, two Vectors with 4 parameters (Starting and ending position
  * coordinates of line segments)
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareEnd(std::pair< cv::Vec4i, float > one,
                        std::pair< cv::Vec4i, float > two) {
    return (one.first[2] > two.first[2]);
  }

  /**
  * @brief Comparison function for std::sort
  * @details Compares the slopes of the line segments
  * @param one, two pairs of a vector with 4 parameters and a slope
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareSlope(std::pair< cv::Vec4i, float > one,
                          std::pair< cv::Vec4i, float > two) {
    return (one.second < two.second);
  }

  /**
  * @brief Finds the absolute difference between two integers
  * @param y1, y2 integers to find the difference between
  * @return Difference between two numbers
  */
  int compareY(int y1, int y2) {
    if (y1 > y2) {
      return (y1 - y2);
    } else {
      return (y2 - y1);
    }
  }

  /**
  * @brief Calculates the slope of the line segment
  * @param one line segment which slope is found
  * @return Slope of the line segment
  */
  float calcSlope(cv::Vec4i);
  /**
  * @brief Finds the average line segment
  * @details All the similar lines (please see the strategy in line detection
  * package)
  * are gathered in the vector, then the critical points are taken for the
  * endpoints
  * of average line segment that is generated
  * @param vec a vector of pairs of line segment and slope
  * @return Slope of the line segment
  */
  cv::Vec4i getAverLine(std::vector< std::pair< cv::Vec4i, float > >);

  /**
  * @brief Finds the difference(not absolute one) between two integers
  * @param a, b integers to find the difference between
  * @return Difference between two numbers
  */
  int getDifference(int, int);

  /**
  * @brief Print lines from a vector to the image (cv::Mat)
  * @param dst, lines  destination image, lines to be mapped to the image
  */
  void printLinesImage(cv::Mat, std::vector< cv::Vec4i >);

  /**
  * @brief Remove unnecessary lines from a vector of line segments (cv::Mat)
  * @details As laser scan has noise we detect for each wall a lot of unnecessary
  * lines
  * we eliminate the lines that represent the same wall by finding the slope
  * along with computing the location of endpoints with respect to each other
  * @param lines1 a vector of line segments taken for processing after applying
  * HoughLinesP function from OpenCV
  */
  void removeLines(std::vector< cv::Vec4i >);

  /**
  * @brief Converts the laserScan-data from polar into cartesian coordinates.
  * Then cleans the data from various problems and finally translates the points
  * into pixels on an actual image (in form of an OpenCV-matrix).
  */
  cv::Mat
  createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &);

  /**
  * @brief Receives a laser scan message and creates an OpenCV image
  * @param laserScan LaserScan Message with information about the distances
  */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &);

  /** @brief Gets number of line segments after elimination
  */
  int getNumLines() { return res.size(); }

  /** @brief Gets last laser scan message
  */
  sensor_msgs::LaserScan::ConstPtr getLaserScan() const { return lastScan; }

  /** @brief Sets the image which represents the last scan message received
  *   @param src image to be set
  */
  cv::Mat getImage() { return src; }

  /** @brief Clears the vector of line segments in order to process the new
  * image from laser scan
  */
  void clearData() { res.clear(); }

  /** @brief Gets the vector of line segments that were processed after line
   * removal
   */
  std::vector< cv::Vec4i > getLines() { return res; }

private:
  std::vector< cv::Vec4i > res;
  std::vector< std::pair< int, int > > points;
  sensor_msgs::LaserScan::ConstPtr lastScan;
  cv::Mat src;
};

#endif
