/** @file LaserScanAggregator.cpp
    @brief Implements LaserScanAggregator.h
    
    @author Felix Schmoll (LiftnLearn)
*/

#include "../include/LaserScanAggregator.h"

void LaserScanAggregator::receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan){
  
  laserScans.push_back(*laserScan);
  if(laserScans.size() > AGGREGATOR_SIZE) {
    laserScans.erase(laserScans.begin());
  }

}

void LaserScanAggregator::aggregatePoints(sensor_msgs::LaserScan& laserScan, std::vector<cv::Point2f>& points) {

  int numOfValues = laserScan.ranges.size();

  for (int i = 0; i < numOfValues; ++i) {
    // everything more distant than the laserRange can mean just the end of the
    // sensor and distorts the actual measurements
    if (laserScan.ranges[i] > LASER_RANGE) {
      continue;
    }

    float alpha =
        laserScan.angle_min + i * laserScan.angle_increment;
    int sign = alpha < 0 ? -1 : 1;

    float hyp = laserScan.ranges[i];
    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    cv::Point2f point(opp*sign, adj);

    points.push_back(point);
  }
}

cv::Mat LaserScanAggregator::createOpenCVImage(std::vector<cv::Point2f>& imagePoints) {
  std::vector<cv::Point2f> points;

  //aggregate all points to be drawn on image
  for(auto it = laserScans.begin(); it != laserScans.end(); ++it) {
    aggregatePoints(*it, points);
  }

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8U, cv::Scalar::all(0));

  for(int i = 0; i < points.size(); ++i) {
    // make sure that values are always within bounds
    int x = RANGE(0, (int)((imageWidth / 2) + STRETCH_FACTOR * points[i].x),
                  imageWidth - 1);
    int y = RANGE(0, (int)((imageHeight / 2) + STRETCH_FACTOR * points[i].y),
                  imageHeight - 1);

    image.at<unsigned char>(cv::Point(x, y)) += 20;//255;
    imagePoints.push_back(cv::Point(x,y));
  }

  return image;
}

