#include "HalfCircleDetector.h"

#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

/*
 * The actual magic happens in the called functions.
 * This is just the glue combining everything.
 */
void HalfCircleDetector::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  cv::Mat image = HalfCircleDetector::createOpenCVImageFromLaserScan(laserScan);

  geometry_msgs::Pose2D pose = HalfCircleDetector::detectHalfCircle(image);

  setHalfCirclePose(pose);
}

/* Interpolates the data up to the requested resolution using linear
 * interpolation.
 */
float HalfCircleDetector::interpolate(int index, int resolution,
                                      std::vector<float> data) {
  int size = data.size();

  float step = 1.0 / ((float)resolution);

  // finding closest actual data in the dataset
  int leftIndex = RANGE(0, (int)(step * index), size - 1);
  int rightIndex = RANGE(0, leftIndex + 1, size - 1);

  // interpolation
  float offset = step * index - leftIndex;
  float value = (1 - offset) * data[leftIndex] + offset * data[rightIndex];

  return value;
}

/*
 * Converts the laserScan-data from polar into cartesian coordinates.
 * Then cleans the data from various problems and finally translates the points
 * into pixels on an actual image (in form of an OpenCV-matrix).
 */
cv::Mat HalfCircleDetector::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  HalfCircleDetector::points.clear();

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) /
                    laserScan->angle_increment;

  int imageHeight = 800;
  int imageWidth = 1600;

  cv::Mat image(imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

  int resolution = 8;
  for (int i = 0; i < numOfValues * resolution; ++i) {
    float hyp =
        HalfCircleDetector::interpolate(i, resolution, laserScan->ranges);

    float laserRange = 0.9 * 4.0;

    // everthing more distant than the laserRange can mean just the end of the
    // sensor and distorts the actual measurements
    if (hyp > laserRange || laserScan->ranges[i / resolution] > laserRange ||
        laserScan->ranges[std::min((i / resolution) + 1, numOfValues - 1)] >
            laserRange) {
      continue;
    }

    float alpha =
        laserScan->angle_min + (i / resolution) * laserScan->angle_increment;
    int sign = alpha < 0 ? -1 : 1;

    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    // make sure that values are always within bounds
    int x =
        RANGE(0, (int)((imageWidth / 2) + 100 * opp * sign), imageWidth - 1);
    int y = RANGE(0, (int)((imageHeight / 2) + adj * 100), imageHeight - 1);

    HalfCircleDetector::points.push_back(std::make_pair(x, y));

    image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(200, 200, 200);
  }

  return image;
}

/*
 * Uses OpenCV to detect all lines in the image. Then calculates the lines
 * equations and checks for all drawn points if they are in proximity of a line.
 * If not we consider it a deviation. If we have more than a certain amount of
 * deviations we assume that we found a half-circle. The position is assumed to
 * be at the average of all deviation points.
 */
geometry_msgs::Pose2D HalfCircleDetector::detectHalfCircle(cv::Mat &image) {

  cv::cvtColor(image, image, CV_BGR2GRAY);
  cv::Mat res = image.clone();

  // use probabilistic Hough-transform to find lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(image, lines, 1, CV_PI / 180, 20, 20, 20);

  int deviationCount = 0;
  float errorMargin = 20;

  // variables for averaging the deviation
  long sumDeviationX = 0;
  long sumDeviationY = 0;

  // iterate over all points and check if it is close to a line
  for (int i = 0; i < HalfCircleDetector::points.size(); ++i) {
    bool detected = false;
    for (int j = 0; j < (int)lines.size(); ++j) {
      cv::Vec4i l = lines[j];
      float numerator = (l[1] - l[3]);
      float denominator = (l[0] - l[2]);

      float error;
      float epsilon = 1;
      //     if(std::abs(numerator) < epsilon)
      //     {
      //       error = abs(l[1] - HalfCircleDetector::points[i].second);
      //     }
      // else if(std::abs(denominator) < epsilon))
      //{
      //     error = abs(l[0] - HalfCircleDetector::points[i].first);
      //} else {
      //        if(denominator < epsilon) m = 0;

      if (abs(denominator) < epsilon) {
        detected = true;
        break;
      }
      float m = numerator / denominator;
      float lineY = l[1] + m * (HalfCircleDetector::points[i].first - l[0]);

      error = abs(lineY - HalfCircleDetector::points[i].second);
      //}

      if (error < errorMargin) {
        detected = true;
        break;
      }
    }

    if (!detected) {
      ++deviationCount;
      sumDeviationX += HalfCircleDetector::points[i].first;
      sumDeviationY += HalfCircleDetector::points[i].second;
    }
  }

  // returns negative values if nothing detected
  int halfCircleX = -1;
  int halfCircleY = -1;

  int deviationThreshold = 100;
  if (deviationCount > deviationThreshold) {
    halfCircleX = (int)sumDeviationX / deviationCount;
    halfCircleY = (int)sumDeviationY / deviationCount;

    ROS_INFO("Half-circle detected: %d at x: %d y: %d", deviationCount,
             halfCircleX, halfCircleY);

    cv::circle(res, cv::Point(halfCircleX, halfCircleY), 20,
               cv::Scalar(255, 255, 255));
  }

  geometry_msgs::Pose2D pose =
      createPose(halfCircleX, halfCircleY, image.cols / 2, image.rows / 2);

  return pose;
}

geometry_msgs::Pose2D HalfCircleDetector::getHalfCirclePose() {
  return HalfCircleDetector::halfCirclePose;
}

void HalfCircleDetector::setHalfCirclePose(geometry_msgs::Pose2D &pose) {
  HalfCircleDetector::halfCirclePose = pose;
}

/*
 * Just uses basic trigonometry.
 */
geometry_msgs::Pose2D HalfCircleDetector::createPose(int posX, int posY,
                                                     int robotX, int robotY) {
  geometry_msgs::Pose2D msg;

  if (posX == -1) {
    msg.x = msg.y = msg.theta = -1;
  } else {
    msg.x = posX - robotX;
    msg.y = posY - robotY;
    msg.theta = std::atan2(msg.y, msg.x);
  }

  return msg;
}
