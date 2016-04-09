/** @file HalfCircleDetector.cpp
  * @brief Implementation of the corresponding header.
  *
  * @author Felix Schmoll
  */
#include "../include/HalfCircleDetector.h"

void HalfCircleDetector::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {

  geometry_msgs::Pose2D pose;
  pose.x = -1;
  pose.y = -1;
  pose.theta = -1;

  //ignore measurements if wall too close
  if(*std::min_element(laserScan->ranges.begin(), laserScan->ranges.end()) > MIN_WALL_DISTANCE) {
    cv::Mat image = HalfCircleDetector::createOpenCVImageFromLaserScan(laserScan);
    pose = HalfCircleDetector::detectHalfCircle(image);
  }
  
  setHalfCirclePose(pose);
}

cv::Mat HalfCircleDetector::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  HalfCircleDetector::points.clear();

  int numOfValues = laserScan->ranges.size();

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8U, cv::Scalar::all(0));

  for (int i = 0; i < numOfValues; ++i) {
    float hyp = laserScan->ranges[i];

    // everything more distant than the laserRange can mean just the end of the
    // sensor and distorts the actual measurements
    if (hyp > LASER_RANGE) {
      continue;
    }

    float alpha =
        laserScan->angle_min + i * laserScan->angle_increment;
    int sign = alpha < 0 ? -1 : 1;

    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    // make sure that values are always within bounds
    int x = RANGE(0, static_cast<int>((imageWidth / 2) + STRETCH_FACTOR * opp * sign),
                  imageWidth - 1);
    int y = RANGE(0, static_cast<int>((imageHeight / 2) + adj * STRETCH_FACTOR),
                  imageHeight - 1);

    HalfCircleDetector::points.push_back(cv::Point2f(x, y));

    image.at<unsigned char>(cv::Point(x, y)) = 255;
  }

  return image;
}

float HalfCircleDetector::verifyCircle(cv::Mat dt, cv::Point2f center,
                                       float radius,
                                       std::vector<cv::Point2f> &inlierSet,
                                       float semiCircleStart) {
  int counter = 0;
  int inlier = 0;

  float minInlierDist = 1.0f;
  float maxInlierDistMax = 100.0f;
  float maxInlierDist = 2.0f;

  if (maxInlierDist < minInlierDist)
    maxInlierDist = minInlierDist;
  if (maxInlierDist > maxInlierDistMax)
    maxInlierDist = maxInlierDistMax;

  int discontinuity = 0;
  int inlierMax = 0;

  // choose samples along the circle and count inlier percentage
//  for (float t = 0; t < 2 * 3.14159265359f; t += 0.05f) {
for (float t = semiCircleStart; t < semiCircleStart + 3.14159265359f; t += 0.05f) {
    counter++;
    float cX = radius * cos(t) + center.x;
    float cY = radius * sin(t) + center.y;

    if(discontinuity > 2) {inlier = 0;}

    if (cX < dt.cols)
      if (cX >= 0)
        if (cY < dt.rows) {
          if (cY >= 0) {
            if (dt.at<float>(cY, cX) < maxInlierDist) {
              inlier++;
              inlierSet.push_back(cv::Point2f(cX, cY));
              discontinuity = 0;
              inlierMax = std::max(inlier, inlierMax);
            }
            else { ++discontinuity; }
          }
         }
  }

  return (static_cast<float>(inlier) / static_cast<float>(counter));
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

float HalfCircleDetector::distance(cv::Point2f &a, cv::Point2f &b) {

  return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

void HalfCircleDetector::getSamplePoints(int &first, int &second, int &third,
                                         int &rightIndex, int &leftIndex,
                                         std::vector<cv::Point2f> &v) {

  ++rightIndex;
  float errorMargin = 0.03 * STRETCH_FACTOR;
  float halfCircleRadius =
      0.15 * STRETCH_FACTOR; // about 30 cm height for circle

  while (rightIndex < static_cast<int>(v.size())) {
    // semi-circle dimensions: 30cm x 16cm

    float difference = fabs(distance(v[rightIndex], v[leftIndex]));
    if (difference > (halfCircleRadius * 2 + errorMargin)) {
      ++leftIndex;
    } else if (difference < (halfCircleRadius * 2 - errorMargin)) {
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
  return;
}

float computeAngleBetweenVectors(float v1_x, float v1_y, float v2_x, float v2_y) {
  //normalize vectors
  float norm_v1 = sqrt((v1_x*v1_x) + (v1_y*v1_y));
  float norm_v2 = sqrt((v2_x*v2_x) + (v2_y*v2_y));

  v1_x /= norm_v1;
  v1_y /= norm_v1;

  v2_x /= norm_v2;
  v2_y /= norm_v2;

  //compute dot-product
  float dot_product = v1_x * v2_x + v1_y * v2_y;

  //return result
  if(dot_product >= 1.0) {
    return 0.0;
  } else if (dot_product <= -1.0) {
    return 3.14; //use some PI constant (consistency!)
  } else {
    return acos(dot_product);
  }

}

geometry_msgs::Pose2D HalfCircleDetector::detectHalfCircle(cv::Mat &image) {

  cv::threshold(image, image, 100, 255, CV_THRESH_BINARY);

  std::vector<cv::Point2f> edgePositions;
  edgePositions = points;

  // create distance transform to efficiently evaluate distance to nearest edge
  cv::Mat dt;
  cv::distanceTransform(255 - image, dt, CV_DIST_L1, 3);

  unsigned int nIterations = 0;

  cv::Point2f bestCircleCenter;
  float bestCircleRadius;
  float bestCirclePercentage = 0;
  float minRadius = 0.22; // TODO: make this configurable/macro/put in config file
  float maxRadius = 0.25; //in meter [m]

  float minCirclePercentage = 0.45f;
  float maxCirclePercentage = 1.0f;

  int maxNrOfIterations =
      edgePositions.size(); 

  int rightIndex = 0;
  int leftIndex = 0;

  for (unsigned int its = 0;
       its < maxNrOfIterations && rightIndex < static_cast<int>(edgePositions.size()); ++its) {
    // RANSAC: randomly choose 3 point and create a circle:
    int idx1 = 0;
    int idx2 = 0;
    int idx3 = 0;

    getSamplePoints(idx1, idx2, idx3, leftIndex, rightIndex, edgePositions);

    // create circle from 3 points:
    cv::Point2f center;
    float radius;
    getCircle(edgePositions[idx1], edgePositions[idx2], edgePositions[idx3],
              center, radius);
    // inlier set unused at the moment but could be used to approximate a (more
    // robust) circle from alle inlier
    std::vector<cv::Point2f> inlierSet;

    float m_x = (image.cols/2 - center.x);
    float m_y = (image.rows/2 - center.y);

    float alpha = atan2(-m_x, m_y);


    // verify or falsify the circle by inlier counting:
    float cPerc = verifyCircle(dt, center, radius, inlierSet, alpha);

    // update best circle information if necessary
    if (cPerc >= bestCirclePercentage && cPerc < maxCirclePercentage)
      if (radius/STRETCH_FACTOR >= minRadius && radius/STRETCH_FACTOR <= maxRadius) {
        bestCirclePercentage = cPerc;
        bestCircleRadius = radius;
        bestCircleCenter = center;
      }
  }

  geometry_msgs::Pose2D pose;
  pose.x = -1;
  pose.y = -1;
  pose.theta = -1;

  // signal if good circle was found
  if (bestCirclePercentage >= minCirclePercentage) {
    pose = createPose(bestCircleCenter.x, bestCircleCenter.y, image.cols / 2,
                      image.rows / 2);
  }

    cv::circle(image, bestCircleCenter, bestCircleRadius, cv::Scalar(255,255,0), 1);

    float m_x = (image.cols/2 - bestCircleCenter.x);
    float m_y = (image.rows/2 - bestCircleCenter.y);

    float alpha = atan2(-m_x, m_y);
    //make color image, draw red circle at starting point

    cv::Mat color(image);
    cv::cvtColor(color, color, CV_GRAY2BGR);

    cv::Point s(bestCircleCenter.x + cos(alpha)*bestCircleRadius, bestCircleCenter.y + sin(alpha)*bestCircleRadius);

    cv::circle(color, s, 6, cv::Scalar(0,0,255),1);     

    //draw tested points
    for (float t = alpha; t < alpha + 3.14159265359f; t += 0.05f) {
      float cX = bestCircleRadius * cos(t) + bestCircleCenter.x;
      float cY = bestCircleRadius * sin(t) + bestCircleCenter.y;
      cv::circle(color, cv::Point2f(cX, cY), 1, cv::Scalar(0,0,255),1);
    }
    line(color, bestCircleCenter, cv::Point(color.cols/2, color.rows/2),cv::Scalar(0,255,0)); 
    
    std::cout << bestCirclePercentage << std::endl;

  cv::imwrite("/home/robotics/color.jpg", color);

  return pose;
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
    msg.x = (posX - robotX) / static_cast<float>(STRETCH_FACTOR);
    msg.y = (posY - robotY) / static_cast<float>(STRETCH_FACTOR);
    msg.theta = std::atan2(msg.y, msg.x);
  }

  return msg;
}
