#include "WallFollowingStrategy.h"

bool WallFollowingStrategy::getCircleVisible() { return circleVisible; }

bool WallFollowingStrategy::getCrashMode() { return crashMode; }

bool WallFollowingStrategy::getCornerHandle() { return cornerStuck; }

void WallFollowingStrategy::getLines(
    const geometry_msgs::Polygon::ConstPtr &lines) {
  lastLineArray = *lines;
}

void WallFollowingStrategy::getCornerRecovery(
    const geometry_msgs::Twist::ConstPtr &cornerOut) {
  if (cornerOut->linear.x == 0 && cornerOut->linear.y == 0 &&
      cornerOut->angular.x == 0 && cornerOut->angular.y == 0) {
    cornerStuck = false;
    cornerHandler.linear.x = 0;
    cornerHandler.linear.y = 0;

    cornerHandler.angular.x = 0;
    cornerHandler.angular.y = 0;
    return;
  }

  cornerStuck = true;
  cornerHandler.linear.x = cornerOut->linear.x;
  cornerHandler.linear.y = cornerOut->linear.y;

  cornerHandler.angular.x = cornerOut->angular.x;
  cornerHandler.angular.y = cornerOut->angular.y;
}

/**
  * @brief Checks if pose represents actual circle and sets values accordingly.
  */
void WallFollowingStrategy::receiveCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePose) {
  // compare to -1
  if (fabs(circlePose->x + 1) < 0.02) {
    circleVisible = false;
    return;
  }

  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));
}

/**
  * @brief Limiting integers to be within a certain range.
  */

void WallFollowingStrategy::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
  WallFollowingStrategy::src =
      WallFollowingStrategy::createOpenCVImageFromLaserScan(laserScan);
}

float WallFollowingStrategy::calcSlope(cv::Vec4i one) {
  return ((one[3] - one[1]) * 1.0 / (one[2] - one[0]));
}

cv::Vec4i WallFollowingStrategy::getAverSlope(
    std::vector< std::pair< cv::Vec4i, float > > vec) {
  cv::Vec4i average;
  average[0] = vec[0].first[0];
  average[1] = vec[0].first[1];

  std::sort(vec.begin(), vec.end(), compareEnd);

  average[2] = vec[0].first[2];
  average[3] = vec[0].first[3];

  return average;
}

int WallFollowingStrategy::getDifference(int a, int b) { return abs(a - b); }

void WallFollowingStrategy::printLinesImage(cv::Mat dst,
                                            std::vector< cv::Vec4i > lines) {
  for (size_t i = 0; i < lines.size(); i++) {
    line(dst, cv::Point(lines[i][0], lines[i][1]),
         cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
    ROS_DEBUG("%d\t %d\t %d\t %d\n", lines[i][0], lines[i][1], lines[i][2],
              lines[i][3]);
  }
}

/** @brief Interpolates the data up to the requested resolution using linear
 * interpolation.
 */
float WallFollowingStrategy::interpolate(int index, int resolution,
                                         std::vector< float > data) {
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
cv::Mat WallFollowingStrategy::createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  WallFollowingStrategy::points.clear();

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) /
                    laserScan->angle_increment;

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

  int resolution = 8;
  for (int i = 0; i < numOfValues * resolution; ++i) {
    float hyp =
        WallFollowingStrategy::interpolate(i, resolution, laserScan->ranges);

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

    WallFollowingStrategy::points.push_back(std::make_pair(x, y));

    image.at< cv::Vec3b >(cv::Point(x, y)) = cv::Vec3b(200, 200, 200);
  }

  src = image;

  cv::imwrite("/home/mgladkova/copy_ws/src/team-3/team_3/"
              "wall_following_strategy/src/Image.jpg",
              image);
  cv::waitKey(20);

  assert(this->getImage().data);

  return image;
}

void WallFollowingStrategy::removeLines(std::vector< cv::Vec4i > lines1) {
  std::vector< std::pair< cv::Vec4i, float > > temp;

  std::sort(lines1.begin(), lines1.end(), compareStart);

  bool pushed = true;

  int count = 0;
  for (size_t i = 0; i < lines1.size(); i++) {
    float slope = calcSlope(lines1[i]);
    if (temp.empty()) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
    } else if (slope > 0 && fabs(temp[0].second - slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               compareY(temp[temp.size() - 1].first[1], lines1[i][1]) <
                   compareY(temp[temp.size() - 1].first[1],
                            temp[temp.size() - 1].first[3])) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
    } else if (slope < 0 && fabs(temp[temp.size() - 1].second - slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               compareY(temp[temp.size() - 1].first[1], lines1[i][1]) <
                   compareY(temp[temp.size() - 1].first[1],
                            temp[temp.size() - 1].first[3])) {
      std::pair< cv::Vec4i, float > p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
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
      res.push_back(getAverSlope(temp));
      temp.clear();

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

    if (!temp.empty() && i == lines1.size() - 1) {
      res.push_back(getAverSlope(temp));
      temp.clear();
    }
  }
}

geometry_msgs::Twist WallFollowingStrategy::turnLeft() {
  geometry_msgs::Twist msg;
  msg.angular.z = M_PI;
  return msg;
}

std::pair< float, float > WallFollowingStrategy::findMinimDistance(int left,
                                                                   int right) {
  sensor_msgs::LaserScan scan = WallFollowingStrategy::getLaserScan();
  scan.ranges.resize(RANGES);
  std::pair< float, float > p = std::make_pair(scan.ranges[left], 0);

  for (int i = left; i < right; i++) {
    p.first = std::min(p.first, scan.ranges[i]);
    if (p.first == scan.ranges[i]) {
      p.second = i;
      ROS_DEBUG("Min %f\n", scan.ranges[i]);
    }
  }

  return p;
}

const geometry_msgs::Twist WallFollowingStrategy::controlMovement() {
  geometry_msgs::Twist msg;
  int minim = INT_MAX;
  float theta = 0;
  int k = 0;
  std::pair< float, float > line =
      WallFollowingStrategy::findMinimDistance(0, RANGES - 1);
  std::vector< cv::Vec4i > vec = getLines();

  std::pair< float, float > right = this->findMinimDistance(180, RANGES - 1);

  if (!src.data) {
    msg.linear.x = 0;
    return msg;
  }

  if (cornerStuck) {
    ROS_INFO("Stuck!");
    msg.linear.x = cornerHandler.linear.x;
    msg.linear.y = cornerHandler.linear.y;

    msg.angular.x = cornerHandler.angular.x;
    msg.angular.y = cornerHandler.angular.y;

    if (circleDistance > 0.5) {
      return msg;
    }
  }

  if (circleVisible && circleDistance != 0 && circleDistance < 3) {
    circleFoundMode = true;
    ROS_INFO("Found Circle!");
    if (circleDistance > 0.05) {
      float variationToCircle = 90 - circleAngle;
      msg.angular.z = std::min(MAX_TURN, 0.05 * variationToCircle);
      std::cout << std::min(MAX_TURN, 0.05 * variationToCircle) << std::endl;
      msg.linear.x = 0.3;
    }
    return msg;
  }

  if (line.first > 0.3 && !followWall && !circleFoundMode) {
    msg.linear.x = 0.3;
    return msg;
  }

  if (line.first < 0.3 && right.first < 0.7 && !circleFoundMode) {
    for (auto i = 0; i < vec.size(); i++) {
      if (minim < abs(vec[i][0] - src.rows / 2)) {
        minim = abs(vec[i][0] - src.rows / 2);
        k = i;
      }
    }
    float den = vec[k][2] - vec[k][0];
    float num = vec[k][3] - vec[k][1];
    float m = 0;
    if (den < 0.1) {
      m = M_PI / 2;
    } else if (num < 0.1) {
      m = 0;
    }
    msg.angular.z = (this->getCurrentAngle() + m) / 10;
    this->setCurrentAngle(this->getCurrentAngle() + m);

    followWall = true;
    return msg;
  } else if (right.first > 0.5 && !circleFoundMode) {
    cornerEdge = true;
    msg.linear.x = 0.16;
    if (!circleVisible)
      msg.angular.z = -M_PI / 10;
    else {
      msg.angular.z = -M_PI / 5;
    }
    return msg;
  }

  if (right.first > 0.5 && !circleFoundMode) {
    this->setCorrecting(true);
    followWall = false;
  } else {
    msg.linear.x = 0.3;
  }
  if (correcting) {
    float variation = 90 - right.second;
    msg.angular.z = std::min(MAX_TURN, TURN_CORRECTION * variation);
    this->setCorrecting(false);
  }

  return msg;
}