#include "LineDetector.h"

/**
  * @brief Limiting integers to be within a certain range.
  */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

#define STRETCH_FACTOR 100

void LineDetector::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = laserScan;
  LineDetector::src = LineDetector::createOpenCVImageFromLaserScan(laserScan);
}

float LineDetector::calcSlope(cv::Vec4i one) {
  return ((one[3] - one[1]) * 1.0 / (one[2] - one[0]));
}

cv::Vec4i
LineDetector::getAverSlope(std::vector<std::pair<cv::Vec4i, float>> vec) {
  // std::sort(vec.begin(), vec.end(), compareSlope);
  // int idx = vec.size()/2;
  cv::Vec4i average;
  average[0] = vec[0].first[0];
  average[1] = vec[0].first[1];

  std::sort(vec.begin(), vec.end(), compareEnd);

  average[2] = vec[0].first[2];
  average[3] = vec[0].first[3];

  // std::cout << average[0] << " , " << average[1] << " , " << average[2] << "
  // , "<< average[3] << std::endl;

  return average;
}

int LineDetector::getDifference(int a, int b) { return abs(a - b); }

void LineDetector::printLinesImage(cv::Mat dst, std::vector<cv::Vec4i> lines) {
  for (size_t i = 0; i < lines.size(); i++) {
    line(dst, cv::Point(lines[i][0], lines[i][1]),
         cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
    // std::cout << lines[i][0] << " , " << lines[i][1] << " , " << lines[i][2]
    //<< " , " << lines[i][3] << std::endl;
  }
}

/** @brief Interpolates the data up to the requested resolution using linear
 * interpolation.
 */
float LineDetector::interpolate(int index, int resolution,
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
 * @brief Converts the laserScan-data from polar into cartesian coordinates.
 * Then cleans the data from various problems and finally translates the points
 * into pixels on an actual image (in form of an OpenCV-matrix).
 */
cv::Mat LineDetector::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  LineDetector::points.clear();

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) /
                    laserScan->angle_increment;

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

  int resolution = 8;
  for (int i = 0; i < numOfValues * resolution; ++i) {
    float hyp = LineDetector::interpolate(i, resolution, laserScan->ranges);

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

    LineDetector::points.push_back(std::make_pair(x, y));

    image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(200, 200, 200);
  }

  cv::imwrite(
      "/home/mgladkova/copy_ws/src/team-3/team_3/line_detection/src/Image.jpg",
      image);

  return image;
}

void LineDetector::removeLines(std::vector<cv::Vec4i> lines1) {
  std::vector<std::pair<cv::Vec4i, float>> temp;

  std::sort(lines1.begin(), lines1.end(), compareStart);

  bool pushed = true;

  int count = 0;
  for (size_t i = 0; i < lines1.size(); i++) {
    float slope = calcSlope(lines1[i]);
    // std::cout << slope << std::endl;

    if (temp.empty()) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      // std::cout << "1" << std::endl;
    } else if (slope > 0 && fabs(temp[0].second - slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               abs(temp[temp.size() - 1].first[1] - lines1[i][1]) <
                   getDifference(temp[temp.size() - 1].first[1],
                                 temp[temp.size() - 1].first[3])) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      // std::cout << "3 ?? " << fabs(temp[0].second - slope) << std::endl;
    } else if (slope < 0 && fabs(temp[temp.size() - 1].second + slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               abs(temp[temp.size() - 1].first[1] - lines1[i][1]) <
                   getDifference(temp[temp.size() - 1].first[1],
                                 temp[temp.size() - 1].first[3])) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      // std::cout << "4 ?? " << fabs(temp[0].second + slope) << std::endl;
    } else if (abs(temp[temp.size() - 1].first[2] - lines1[i][0]) < 2 &&
               abs(temp[temp.size() - 1].first[3] - lines1[i][1]) < 2) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      // std::cout << "5 ?? " << fabs(temp[0].second + slope) << std::endl;
    } else {
      res.push_back(getAverSlope(temp));
      temp.clear();
      // std::cout << "6" << std::endl;

      for (int j = 0; j < res.size(); j++) {
        if (slope > 0 && fabs(calcSlope(res[j]) - slope) < 2 ||
            slope < 0 && fabs(calcSlope(res[j]) + slope) < 2) {
          // std::cout << "HH" << std::endl;
          pushed = false;
          break;
        }
      }

      if (pushed) {
        std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
        // std::cout << "YES" << std::endl;
        temp.push_back(p);
        pushed = true;
      }

      // std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i],slope);
      // temp.push_back(p);
    }

    if (!temp.empty() && i == lines1.size() - 1) {
      res.push_back(getAverSlope(temp));
      temp.clear();
      // std::cout << "7" << std::endl;
    }

    // std::cout << "LINES " << lines1[i][0] << " , " << lines1[i][1] << " , "
    //<< lines1[i][2] << " , " << lines1[i][3] << std::endl;
    // std::cout << variationStart << std::endl;
  }
}
