/** @file HalfCircleDetector.cpp
  * @brief Implementation of the corresponding header.
  *
  * @author Felix Schmoll
  */
//#include "../include/HalfCircleDetector.h"

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
                                      std::vector< float > data) {
  int size = static_cast< int >(data.size());
  auto step = 1.0 / (static_cast< float >(resolution));

  // finding closest actual data in the dataset
  int leftIndex = RANGE(0, (int)(step * index), size - 1);
  int rightIndex = RANGE(0, leftIndex + 1, size - 1);

  // everthing more distant than the laserRange can mean just the end of the
  // sensor and distorts the actual measurements
  // float diff = fabs(data[leftIndex] - data[rightIndex]);
  if (data[leftIndex] > LASER_RANGE ||
      data[rightIndex] > LASER_RANGE) { //|| diff > 0.4) {
    return -1.0;
  }

  // bilinear interpolation
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

    float opp = fabs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    // make sure that values are always within bounds
    int x = RANGE(
        0, static_cast< int >((imageWidth / 2) + STRETCH_FACTOR * opp * sign),
        imageWidth - 1);
    int y =
        RANGE(0, static_cast< int >((imageHeight / 2) + adj * STRETCH_FACTOR),
              imageHeight - 1);

    points.push_back(std::make_pair(opp, adj));

    image.at< unsigned char >(cv::Point(x, y)) = 255;
  }

  return image;
}

float distance(Point &a, Point &b) {

  return sqrt(pow((a.first - b.first), 2) + pow((a.second - b.second), 2));
}

/**
 * @detail Uses OpenCV to detect all lines in the image. Then calculates the
 * lines
 * equations and checks for all drawn points if they are in proximity of a line.
 * If not we consider it a deviation. If we have more than a certain amount of
 * deviations we assume that we found a half-circle. The position is assumed to
 * be at the average of all deviation points.
 */
geometry_msgs::Pose2D HalfCircleDetector::detectHalfCircle(cv::Mat &image) {

  cv::Mat pic(image);
  cvtColor(pic, pic, CV_GRAY2BGR);

  // 5cm error allowed for where the semicircle is
  float errorMargin = 0.03;
  float halfCircleRadius = 0.12; // about 30 cm height for circle

  unsigned int leftIndex = 0;
  unsigned int rightIndex = 0;

  while (rightIndex < points.size()) {
    // semi-circle dimensions: 30cm x 16cm

    float difference = fabs(distance(points[rightIndex], points[leftIndex]));

    if (difference > (halfCircleRadius * 2 + errorMargin)) {
      ++leftIndex;
    } else if (difference < (halfCircleRadius * 2 - errorMargin)) {
      ++rightIndex;
    } else {

      // check if point in between is approx. where it should be for a
      // semicircle
      float numerator = points[leftIndex].second - points[rightIndex].second;
      float denominator = points[leftIndex].first - points[rightIndex].first;

      if (fabs(numerator) < 0.01 || fabs(denominator) < 0.01) {
        ++rightIndex;
        continue;
      }

      // normal to line connecting both points
      // TODO: make sure direction is always inwards
      float m = -1.0 / (numerator / denominator);

      // compute expected values for semicircle
      float alpha = atan(m);
      float dy = sin(alpha) * halfCircleRadius;
      float dx = cos(alpha) * halfCircleRadius;

      float expected_x =
          (points[leftIndex].first + points[rightIndex].first) / 2 + dx;
      float expected_y =
          (points[leftIndex].second + points[rightIndex].second) / 2 + dy;

      Point expected_middlepoint = std::make_pair(expected_x, expected_y);

      int imageHeight = 8 * STRETCH_FACTOR;
      int imageWidth = 16 * STRETCH_FACTOR;

      cv::Point drawOne =
          cv::Point(imageWidth / 2 + expected_x * STRETCH_FACTOR,
                    imageHeight / 2 + expected_y * STRETCH_FACTOR);

      int index = leftIndex;
      Point actual_middlepoint = points[leftIndex];
      while (fabs(distance(actual_middlepoint, points[leftIndex]) -
                  distance(actual_middlepoint, points[rightIndex])) > 0.02 &&
             index < rightIndex) {
        actual_middlepoint = points[++index];
      }

      float deviation = distance(actual_middlepoint, expected_middlepoint);

      if (deviation < 0.05) {
        ROS_INFO("semicircle detected");
        //    line(pic, drawOne, drawTwo, cv::Scalar(0,0,255), 3, 8);
        cv::circle(pic, drawOne, halfCircleRadius * STRETCH_FACTOR,
                   cv::Scalar(0, 0, 255), 3, 8);

        cv::circle(
            pic, cv::Point((expected_middlepoint.first - dx) * STRETCH_FACTOR +
                               imageWidth / 2,
                           (expected_middlepoint.second - dy) * STRETCH_FACTOR +
                               imageHeight / 2),
            2, cv::Scalar(255, 0, 0), 3, 8);

        std::cout << "first: " << points[leftIndex].first << " "
                  << points[leftIndex].second
                  << "second: " << points[rightIndex].first << " "
                  << points[rightIndex].second << "expected: " << expected_x
                  << " " << expected_y
                  << " actual: " << actual_middlepoint.first << " "
                  << actual_middlepoint.second << std::endl;
      }
      ++rightIndex;
    }
  }

  cv::imwrite("/home/robotics/lines.jpg", pic);
  // returns negative values if nothing detected
  int halfCircleX = -1;
  int halfCircleY = -1;

  geometry_msgs::Pose2D pose; // =
  //      createPose(halfCircleX, halfCircleY, (image.cols / 2), (image.rows /
  //      2));

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
    msg.x = (posX - robotX) / static_cast< float >(STRETCH_FACTOR);
    msg.y = (posY - robotY) / static_cast< float >(STRETCH_FACTOR);
    msg.theta = std::atan2(msg.y, msg.x);
  }

  return msg;
}
