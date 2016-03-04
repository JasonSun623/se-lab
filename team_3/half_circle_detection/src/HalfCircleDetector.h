#ifndef HALFCIRCLEDETECTOR_H
#define HALFCIRCLEDETECTOR_H 

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


typedef std::pair<int, int> Point;

class HalfCircleDetector 
{

  public:
    void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan);
    cv::Mat createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan);
    Point detectHalfCircle(cv::Mat& image);

  private:
    std::vector<Point> points; 
    float interpolate(int index, int resolution, std::vector<float> data, int size);
};

#endif
