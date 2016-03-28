/** @file HalfCircleDetector.cpp
  * @brief Implementation of the corresponding header.
  *
  * @author Felix Schmoll
  */
#include "../include/HalfCircleDetector.h"

/**
  * @detail Limiting integers to be within a certain range.
  */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

#define STRETCH_FACTOR 100

/**
 * @detail The actual magic happens in the called functions.
 * This is just the glue combining everything.
 */
void HalfCircleDetector::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  cv::Mat image = HalfCircleDetector::createOpenCVImageFromLaserScan(laserScan);

  geometry_msgs::Pose2D pose = HalfCircleDetector::detectHalfCircle(image);

  setHalfCirclePose(pose);
}

/** @detail Interpolates the data up to the requested resolution using linear
 * interpolation.
 */
float HalfCircleDetector::interpolate(int index, int resolution,
                                      std::vector<float> data) {
  int size = data.size();
  float step = 1.0 / ((float)resolution);

  // finding closest actual data in the dataset
  int leftIndex = RANGE(0, (int)(step * index), size - 1);
  int rightIndex = RANGE(0, leftIndex + 1, size - 1);

  // everthing more distant than the laserRange can mean just the end of the
  // sensor and distorts the actual measurements
  if (data[leftIndex] > LASER_RANGE || data[rightIndex] > LASER_RANGE) {
    return -1.0;
  }

  // interpolation
  float offset = step * index - leftIndex;
  float value = (1 - offset) * data[leftIndex] + offset * data[rightIndex];

  return value;
}

/**
 * @detail Converts the laserScan-data from polar into cartesian coordinates.
 * Then cleans the data from various problems and finally translates the points
 * into pixels on an actual image (in form of an OpenCV-matrix).
 */
cv::Mat HalfCircleDetector::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  HalfCircleDetector::points.clear();

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) /
                    laserScan->angle_increment;

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8U, cv::Scalar::all(0));

  int resolution = 8;
  for (int i = 0; i < numOfValues * resolution; ++i) {
    float hyp =
        HalfCircleDetector::interpolate(i, resolution, laserScan->ranges);

    // skip invalid values
    if (hyp < 0) {
      continue;
    }

    float alpha =
        laserScan->angle_min + (i / resolution) * laserScan->angle_increment;
    int sign = alpha < 0 ? -1 : 1;

    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    // make sure that values are always within bounds
    int x = RANGE(0, (int)((imageWidth / 2) + STRETCH_FACTOR * opp * sign),
                  imageWidth - 1);
    int y = RANGE(0, (int)((imageHeight / 2) + adj * STRETCH_FACTOR),
                  imageHeight - 1);

    HalfCircleDetector::points.push_back(cv::Point2f(x, y));

    image.at<unsigned char>(cv::Point(x, y)) = 255;
  }

  return image;
}

float HalfCircleDetector::verifyCircle(cv::Mat dt, cv::Point2f center,
                                       float radius,
                                       std::vector<cv::Point2f> &inlierSet) {
  int counter = 0;
  int inlier = 0;
  float minInlierDist = 1.0f;
  float maxInlierDistMax = 100.0f;
  float maxInlierDist = 2.0f; // radius/25.0f;
  if (maxInlierDist < minInlierDist)
    maxInlierDist = minInlierDist;
  if (maxInlierDist > maxInlierDistMax)
    maxInlierDist = maxInlierDistMax;

  int discontinuity = 0;
  int inlierMax = 0;

  // choose samples along the circle and count inlier percentage
  for (float t = 0; t < 2 * 3.14159265359f; t += 0.05f) {
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

  return (float)inlierMax / float(counter);
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

  while (rightIndex < v.size()) {
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

/**
* @detail Computes possible circles and checks for inlier-percentage. Returns a
* pose initialised to -1 if no circle is detected.
* Partially taken from http://stackoverflow.com/a/26234137.
*/
geometry_msgs::Pose2D HalfCircleDetector::detectHalfCircle(cv::Mat &image) {

  // cv::threshold(input, mask, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
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
  float minRadius = 10; // TODO: ADJUST THIS PARAMETER; currently not used

  float minCirclePercentage = 0.40f;
  float maxCirclePercentage = 0.50f;

  int maxNrOfIterations =
      edgePositions.size(); // TODO: adjust this parameter or include some real
                            // ransac criteria with inlier/outlier percentages
                            // to decide when to stop

  int rightIndex = 0;
  int leftIndex = 0;

  for (unsigned int its = 0;
       its < maxNrOfIterations && rightIndex < edgePositions.size(); ++its) {
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

    // verify or falsify the circle by inlier counting:
    float cPerc = verifyCircle(dt, center, radius, inlierSet);

    // update best circle information if necessary
    if (cPerc >= bestCirclePercentage && cPerc < maxCirclePercentage)
      if (radius >= minRadius) {
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

  return pose;
}

/**
  * @detail Get half-circle pose.
  */
geometry_msgs::Pose2D HalfCircleDetector::getHalfCirclePose() {
  return HalfCircleDetector::halfCirclePose;
}

/**
  * @detail Set half-circle pose.
  */
void HalfCircleDetector::setHalfCirclePose(geometry_msgs::Pose2D &pose) {
  HalfCircleDetector::halfCirclePose = pose;
}

/**
 * @detail Basically converts coordinates from cartesian in the image frame to
 * polar in the world frame.
 */
geometry_msgs::Pose2D HalfCircleDetector::createPose(int posX, int posY,
                                                     int robotX, int robotY) {
  geometry_msgs::Pose2D msg;

  if (posX == -1) {
    msg.x = msg.y = msg.theta = -1;
  } else {
    msg.x = (posX - robotX) / (float)STRETCH_FACTOR;
    msg.y = (posY - robotY) / (float)STRETCH_FACTOR;
    msg.theta = std::atan2(msg.y, msg.x);
  }

  return msg;
}
