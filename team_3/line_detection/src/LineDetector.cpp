/** @file LineDetector.cpp
  * Implementation of LineDetector.h
  * @author Mariia Gladkova
  */

#include "../include/LineDetector.h"

void LineDetector::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = laserScan;
  LineDetector::src = LineDetector::createOpenCVImageFromLaserScan(laserScan);
}

float LineDetector::calcSlope(cv::Vec4i one) {
  // by formula m = (y-y0)/(x-x0)
  return ((one[3] - one[1]) * 1.0 / (one[2] - one[0]));
}

cv::Vec4i
LineDetector::getAverLine(std::vector< std::pair< cv::Vec4i, float > > vec) {
  cv::Vec4i average;
  // initial vector is sorted by x position of the starting point
  // thus the leftmost is taken
  average[0] = vec[0].first[0];
  average[1] = vec[0].first[1];

  // sort by x coordinate of the end point and take first element
  // endpoint position
  std::sort(vec.begin(), vec.end(), compareEnd);

  average[2] = vec[0].first[2];
  average[3] = vec[0].first[3];

  return average;
}

int LineDetector::getDifference(int a, int b) { return abs(a - b); }

void LineDetector::printLinesImage(cv::Mat dst,
                                   std::vector< cv::Vec4i > lines) {
  for (size_t i = 0; i < lines.size(); i++) {
    line(dst, cv::Point(lines[i][0], lines[i][1]),
         cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
  }
}


cv::Mat LineDetector::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  LineDetector::points.clear();

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) /
                    laserScan->angle_increment;

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

  for (int i = 0; i < numOfValues; ++i) {
    float hyp = laserScan->ranges[i];

    // skip invalid values
    if (hyp > LASER_RANGE) {
      continue;
    }

    float alpha =
        laserScan->angle_min + i * laserScan->angle_increment;
    int sign = alpha < 0 ? -1 : 1;

    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    // make sure that values are always within bounds
    int x = RANGE(0, (int)((imageWidth / 2) + STRETCH_FACTOR * opp * sign),
                  imageWidth - 1);
    int y = RANGE(0, (int)((imageHeight / 2) + adj * STRETCH_FACTOR),
                  imageHeight - 1);

    LineDetector::points.push_back(std::make_pair(x, y));

    image.at< cv::Vec3b >(cv::Point(x, y)) = cv::Vec3b(200, 200, 200);
  }

  std::string path = ros::package::getPath("wall_following_strategy");
  path += "/src/Image.jpg";
  cv::imwrite(path, image);
  cv::waitKey(20);

  return image;
}

void LineDetector::removeLines(std::vector< cv::Vec4i > lines1) {
  // "bucket" for each of the group of "similar" lines
  std::vector< std::pair< cv::Vec4i, float > > temp;
  // sort the lines by the x coordinate of the starting point
  std::sort(lines1.begin(), lines1.end(), compareStart);

  bool pushed = true;

  int count = 0;
  // loop through all the lines that have been detected by HoughLinesP
  for (size_t i = 0; i < lines1.size(); i++) {
    float slope = calcSlope(lines1[i]);
    // if the bucket is empty push the current line to be processed
    if (temp.empty()) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      // case 1 of similar line segments: the slopes are almost the same,
      // starting position of the x-coordinate of the second line segment
      // that is being processed lies between starting and ending
      // points of first line segment. Taking into account that the line
      // segments
      // are not too far apart in y-direction (e.g. two  parallel walls)

    } else if (fabs(temp[temp.size() - 1].second - slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               compareY(temp[temp.size() - 1].first[1], lines1[i][1]) <
                   compareY(temp[temp.size() - 1].first[1],
                            temp[temp.size() - 1].first[3])) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      // case 2 and 3 of similar lines: the slopes are almost the same, but
      // for the second line segment x coordinate of the starting point lies a
      // bit farther
      // from the endpoint of the first line segment
    } else if (slope < 0 && fabs(temp[temp.size() - 1].second - slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[2] - lines1[i][0]) < 2) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
    } else if (slope > 0 &&
               compareY(temp[temp.size() - 1].first[3], lines1[i][1]) < 2 &&
               fabs(temp[temp.size() - 1].second - slope) < 0.3) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
    } else {
      // all the similar line segments were pushed - find the average of them
      // and push
      // in a resulting vector of line segments
      res.push_back(getAverLine(temp));
      temp.clear();
      // as line segments sorted by x-coordinate of the starting point can be
      // shuffled
      // with respect to the wall they belong to we need to check before
      // processing
      // a new line segment whether the lines similar to it were processed
      // before
      for (int j = 0; j < res.size(); j++) {
        if (slope > 0 && fabs(calcSlope(res[j]) - slope) < 2 ||
            slope < 0 && fabs(calcSlope(res[j]) + slope) < 2) {
          pushed = false;
          break;
        }
      }

      if (pushed) {
        std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
        temp.push_back(p);
        pushed = true;
      }
    }

    // always process the current bucket before the end of vector of line
    // segments
    if (!temp.empty() && i == lines1.size() - 1) {
      res.push_back(getAverLine(temp));
      temp.clear();
    }
  }
}
