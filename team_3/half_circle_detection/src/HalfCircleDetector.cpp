/** @file HalfCircleDetector.cpp
  * @brief Implementation of the corresponding header.
  *
  * @author Felix Schmoll
  */
#include "../include/HalfCircleDetector.h"

HalfCircleDetector::HalfCircleDetector(
    float laserRange, float minimumDistance, int stretchFactor,
    float halfCircleRadius, float minCirclePercentage, float maxInlierDist,
    float maxCircleDensity, int maxCirclePoints) {
  this->laserRange = laserRange;
  this->minimumDistance = minimumDistance;
  this->halfCircleRadius = halfCircleRadius;
  this->stretchFactor = stretchFactor;
  this->minCirclePercentage = minCirclePercentage;
  this->maxInlierDist = maxInlierDist;
  this->maxCircleDensity = maxCircleDensity;
  this->maxCirclePoints = maxCirclePoints;
}

void HalfCircleDetector::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  geometry_msgs::Pose2D pose;
  pose.x = -1;
  pose.y = -1;
  pose.theta = -1;

  // ignore measurements if wall too close
  //if (*std::min_element(laserScan->ranges.begin(), laserScan->ranges.end()) >
     // minimumDistance) {
    cv::Mat image =
        HalfCircleDetector::createOpenCVImageFromLaserScan(laserScan);
    setLaserScanImage(image);
    pose = HalfCircleDetector::detectHalfCircle(image);
  //}

  setHalfCirclePose(pose);
}

cv::Mat HalfCircleDetector::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  HalfCircleDetector::points.clear();

  int imageHeight = 8 * stretchFactor;
  int imageWidth = 8 * stretchFactor;

  cv::Mat image(imageHeight, imageWidth, CV_8U, cv::Scalar::all(0));

  for (int i = 0; i < laserScan->ranges.size(); i++) {
    float hyp = laserScan->ranges[i];

    // everything more distant than the laserRange can mean just the end of the
    // sensor and distorts the actual measurements
    if (hyp > laserRange) {
      continue;
    }

    float alpha = laserScan->angle_min + i * laserScan->angle_increment;
    int sign = alpha < 0 ? -1 : 1;

    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    /* Make sure that values are always within bounds */
    int x = RANGE(
        0, static_cast< int >((imageWidth / 2) + stretchFactor * opp * sign),
        imageWidth - 1);
    int y =
        RANGE(0, static_cast< int >((imageHeight / 2) + adj * stretchFactor),
              imageHeight - 1);

    HalfCircleDetector::points.push_back(cv::Point2f(x, y));

    image.at< unsigned char >(cv::Point(x, y)) = 255;
  }

  //  for(int i = 0; i < HalfCircleDetector::points.size()-10; ++i) {
  //    cv::Point a = points[i];
  //    for(int j = 0; j < 10; ++j) {
  //      cv::Point b = points[i+j];
  //
  //      /* Very basic interpolation. Makes up for the fact that at
  //      lines/corners
  //       * the resolution is much higher than at semicircles. */
  //      image.at< unsigned char >(cv::Point((a.x+b.x)/2, (a.y+b.y)/2));
  //    }
  //  }

  return image;
}

float HalfCircleDetector::verifyCircle(cv::Mat dt, cv::Point2f center,
                                       float radius, float semiCircleStart) {
  int counter = 0;
  int inlier = 0;

  // choose samples along the circle and count inlier percentage
  for (float t = semiCircleStart + M_PI / 5; t < semiCircleStart + 4 * M_PI / 5;
       t += 0.01f) {
    counter++;
    float cX = radius * cos(t) + center.x;
    float cY = radius * sin(t) + center.y;

    if (cX < dt.cols && cX >= 0) {
      if (cY < dt.rows && cY >= 0) {
        if (dt.at< float >(cY, cX) < maxInlierDist) {
          inlier++;
        }
      }
    }
  }

  return (static_cast< float >(inlier) / static_cast< float >(counter));
}

void HalfCircleDetector::getCircle(cv::Point2f &p1, cv::Point2f &p2,
                                   cv::Point2f &p3, cv::Point2f &center,
                                   float &radius) {
  float x1 = p1.x;
  float x2 = p2.x;
  float x3 = p3.x;

  float y1 = p1.y;
  float y2 = p2.y;
  float y3 = p3.y;

  center.x = (x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
             (x3 * x3 + y3 * y3) * (y1 - y2);
  center.x /= (2 * (x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2));

  center.y = (x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
             (x3 * x3 + y3 * y3) * (x2 - x1);
  center.y /= (2 * (x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2));

  center.x = fabs(center.x);
  center.y = fabs(center.y);

  radius = sqrt((center.x - x1) * (center.x - x1) +
                (center.y - y1) * (center.y - y1));
}

void HalfCircleDetector::getSamplePoints(int &first, int &second, int &third,
                                         int &rightIndex, int &leftIndex,
                                         std::vector< cv::Point2f > &v) {

  ++rightIndex;
  float errorMargin = 0.02 * stretchFactor;
  float halfCircleRadiusPixel = halfCircleRadius * stretchFactor;

  while (rightIndex < static_cast< int >(v.size())) {
    float difference = fabs(norm(v[rightIndex] - v[leftIndex]));

    if (difference > (halfCircleRadiusPixel * 2 + errorMargin)) {
      ++leftIndex;
    } else if (difference < (halfCircleRadiusPixel * 2 - errorMargin)) {
      ++rightIndex;
    } else {
      first = leftIndex;
      second = rightIndex;
      third = (leftIndex + rightIndex) / 2;
      return;
    }
  }

  first = leftIndex;
  second = rightIndex - 1;
  third = (leftIndex + rightIndex - 1) / 2;
}

bool HalfCircleDetector::verifyCircleDensity(
    std::pair< int, int > &boundaries) {
  int difference = boundaries.second - boundaries.first;

  ROS_ASSERT(difference > 0);

  float sum = 0;
  for (int i = boundaries.first; i < boundaries.second; ++i) {
    sum += cv::norm(points[i + 1] - points[i]);
  }

  sum /= static_cast< float >(difference);

  ROS_INFO("\nDifference in index: %d\n Average dist: %lf", difference, sum);

  return (sum > maxCircleDensity) && (difference < maxCirclePoints);
}

geometry_msgs::Pose2D HalfCircleDetector::detectHalfCircle(cv::Mat &image) {
  cv::threshold(image, image, 0, 255, CV_THRESH_BINARY);

  // create distance transform to efficiently evaluate distance to nearest edge
  cv::Mat dt;
  cv::distanceTransform(255 - image, dt, CV_DIST_L1, 3);

  cv::Point2f bestCircleCenter;
  std::pair< int, int > bestCircleBoundaries;
  float bestCircleRadius = 0;
  float bestCirclePercentage = 0;
  float minRadius = halfCircleRadius - 0.01;
  float maxRadius = halfCircleRadius + 0.01;

  int rightIndex = 0;
  int leftIndex = 0;
  int maxNrOfIterations = HalfCircleDetector::points.size();

  for (int its = 0; std::max(its, rightIndex) < maxNrOfIterations; ++its) {
    // choose 3 point and create a circle:
    int idx1 = 0;
    int idx2 = 0;
    int idx3 = 0;
    getSamplePoints(idx1, idx2, idx3, leftIndex, rightIndex,
                    HalfCircleDetector::points);

    // create circle from 3 points:
    float radius;
    cv::Point2f center;
    std::pair< int, int > boundaries(idx1, idx3);
    getCircle(points[idx1], points[idx2], points[idx3], center, radius);

    float m_x = (image.cols / 2 - center.x);
    float m_y = (image.rows / 2 - center.y);

    float alpha = atan2(-m_x, m_y);

    // verify or falsify the circle by inlier counting:
    float cPerc = verifyCircle(dt, center, radius, alpha);

    // update best circle information if necessary
    if (cPerc >= bestCirclePercentage && radius / stretchFactor >= minRadius &&
        radius / stretchFactor <= maxRadius) {
      bestCirclePercentage = cPerc;
      bestCircleRadius = radius;
      bestCircleCenter = center;
      bestCircleBoundaries = boundaries;
    }
  }

  geometry_msgs::Pose2D pose;
  pose.x = -1;
  pose.y = -1;
  pose.theta = -1;

  // signal if good circle was found
  if (bestCirclePercentage >= minCirclePercentage &&
      verifyCircleDensity(bestCircleBoundaries)) {
    pose = createPose(bestCircleCenter.x, bestCircleCenter.y, image.cols / 2,
                      image.rows / 2);
  }

  drawHalfCircle(image, bestCircleRadius, bestCircleCenter);

  ROS_DEBUG("Circle certainty: %lf", bestCirclePercentage);

  return pose;
}

void HalfCircleDetector::drawHalfCircle(cv::Mat image, float bestCircleRadius,
                                        cv::Point2f bestCircleCenter) {
  cv::Mat color(image);

  cv::cvtColor(color, color, CV_GRAY2BGR);

  // cv::circle(color, bestCircleCenter, bestCircleRadius, cv::Scalar(255, 255,
  // 0), 1);

  float m_x = (image.cols / 2 - bestCircleCenter.x);
  float m_y = (image.rows / 2 - bestCircleCenter.y);

  float semiCircleStart = atan2(-m_x, m_y);

  cv::Point s(bestCircleCenter.x + cos(semiCircleStart) * bestCircleRadius,
              bestCircleCenter.y + sin(semiCircleStart) * bestCircleRadius);

  // draw tested points
  for (float t = semiCircleStart + M_PI / 5; t < semiCircleStart + 4 * M_PI / 5;
       t += 0.01f) {
    float cX = bestCircleRadius * cos(t) + bestCircleCenter.x;
    float cY = bestCircleRadius * sin(t) + bestCircleCenter.y;
    cv::circle(color, cv::Point2f(cX, cY), 1, cv::Scalar(0, 0, 255), 1);
  }

  line(color, bestCircleCenter, cv::Point(color.cols / 2, color.rows / 2),
       cv::Scalar(0, 255, 0));

  std::string path = ros::package::getPath("half_circle_detection");
  path += "/halfCircle.jpg";

  cv::imwrite(path, color);
}

geometry_msgs::Pose2D HalfCircleDetector::getHalfCirclePose() {
  return HalfCircleDetector::halfCirclePose;
}

void HalfCircleDetector::setHalfCirclePose(geometry_msgs::Pose2D &pose) {
  HalfCircleDetector::halfCirclePose = pose;
}

geometry_msgs::Pose2D HalfCircleDetector::createPose(int posX, int posY,
                                                     int robotX, int robotY) {
  geometry_msgs::Pose2D msg;

  if (posX == -1) {
    msg.x = msg.y = msg.theta = -1;
  } else {
    msg.x = (posX - robotX) / static_cast< float >(stretchFactor);
    msg.y = (posY - robotY) / static_cast< float >(stretchFactor);
    msg.theta = std::atan2(msg.y, msg.x);
  }

  return msg;
}
