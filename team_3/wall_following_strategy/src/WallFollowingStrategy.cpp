/** @file WallFollowingStrategy.cpp
  * Implementation of WallFollowingStrategy.h
  * @author Mariia Gladkova
  * @author Felix Schmoll
  */
#include "../include/WallFollowingStrategy.h"

bool WallFollowingStrategy::getCircleVisible() { return circleVisible; }

bool WallFollowingStrategy::getCrashMode() { return crashMode; }

void WallFollowingStrategy::getCrashRecovery(
    const geometry_msgs::Twist::ConstPtr &crashOut) {
  if (crashOut->linear.x == 0 && crashOut->angular.z == 0) {
    crashMode = false;
    crashHandler.linear.x = 0;
    crashHandler.angular.z = 0;
    return;
  }

  crashMode = true;
  crashHandler.linear.x = crashOut->linear.x;
  crashHandler.angular.z = crashOut->angular.z;
}

void WallFollowingStrategy::receiveCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePose) {
  if (fabs(circlePose->x + 1) < 0.02) {
    circleVisible = false;
    return;
  }
  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));
}

void WallFollowingStrategy::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
}

void WallFollowingStrategy::receiveOpenCVImage(
    const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    setImage(cv_ptr->image);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

float WallFollowingStrategy::calcSlope(cv::Vec4i one) {
  // by formula m = (y-y0)/(x-x0)
  return ((one[3] - one[1]) * 1.0 / (one[2] - one[0]));
}

cv::Vec4i WallFollowingStrategy::getAverLine(
    std::vector<std::pair<cv::Vec4i, float>> vec) {
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

int WallFollowingStrategy::getDifference(int a, int b) { return abs(a - b); }

void WallFollowingStrategy::printLinesImage(cv::Mat dst,
                                            std::vector<cv::Vec4i> lines) {
  for (size_t i = 0; i < lines.size(); i++) {
    line(dst, cv::Point(lines[i][0], lines[i][1]),
         cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
    ROS_DEBUG("%d\t %d\t %d\t %d\n", lines[i][0], lines[i][1], lines[i][2],
              lines[i][3]);
  }
}

bool WallFollowingStrategy::lineCondition(std::pair<cv::Vec4i, float> line,
                                          cv::Vec4i initialLineSegment) {
  float slope = calcSlope(initialLineSegment);
  return fabs(line.second - slope) < 0.3 &&
         abs(line.first[0] - initialLineSegment[0]) <
             getDifference(line.first[0], line.first[2]) &&
         compareY(line.first[1], initialLineSegment[1]) <
             compareY(line.first[1], line.first[3]);
}

void WallFollowingStrategy::removeLines(std::vector<cv::Vec4i> lines) {
  // "bucket" for each of the group of "similar" lines
  std::vector<std::pair<cv::Vec4i, float>> temp;
  // sort the lines by the x coordinate of the starting point
  std::sort(lines.begin(), lines.end(), compareStart);

  bool pushed = true;

  int count = 0;
  // loop through all the lines that have been detected by HoughLinesP
  for (size_t i = 0; i < lines.size(); i++) {
    float slope = calcSlope(lines[i]);
    std::pair<cv::Vec4i, float> lastLine;
    if (!temp.empty()) {
      lastLine = temp.back();
    }
    // if the bucket is empty push the current line to be processed
    if (temp.empty()) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines[i], slope);
      temp.push_back(p);

      /* case 1 of similar line segments: the slopes are almost the same,
         starting position of the x-coordinate of the second line segment
         that is being processed lies between starting and ending
         points of first line segment. Taking into account that the line
         segments are not too far apart in y-direction
         (e.g. two  parallel walls) */
    } else if (lineCondition(lastLine, lines[i])) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines[i], slope);
      temp.push_back(p);

      /* case 2 and 3 of similar lines: the slopes are almost the same, but
         for the second line segment x coordinate of the starting point lies a
         bit farther from the endpoint of the first line segment */
    } else if (slope < 0 && fabs(lastLine.second - slope) < 0.3 &&
               abs(lastLine.first[2] - lines[i][0]) < 2) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines[i], slope);
      temp.push_back(p);
    } else if (slope > 0 && compareY(lastLine.first[3], lines[i][1]) < 2 &&
               fabs(lastLine.second - slope) < 0.3) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines[i], slope);
      temp.push_back(p);
    } else {
      /* all the similar line segments were pushed - find the average of them
         and push in a resulting vector of line segments */
      res.push_back(getAverLine(temp));
      temp.clear();
      /* as line segments sorted by x-coordinate of the starting point can be
         shuffle with respect to the wall they belong to we need to check before
         processing a new line segment whether the lines similar to it were
         processed before */
      for (int j = 0; j < res.size(); j++) {
        if (slope > 0 && fabs(calcSlope(res[j]) - slope) < 2 ||
            slope < 0 && fabs(calcSlope(res[j]) + slope) < 2) {
          pushed = false;
          break;
        }
      }

      if (pushed) {
        std::pair<cv::Vec4i, float> p = std::make_pair(lines[i], slope);
        temp.push_back(p);
        pushed = true;
      }
    }

    // always process the current bucket before the end of vector of line
    // segments
    if (!temp.empty() && i == lines.size() - 1) {
      res.push_back(getAverLine(temp));
      temp.clear();
    }
  }
}

std::pair<float, float> WallFollowingStrategy::findMinimDistance(int left,
                                                                 int right) {
  sensor_msgs::LaserScan scan = WallFollowingStrategy::getLaserScan();
  std::pair<float, float> p = std::make_pair(scan.ranges[left], 0);

  for (int i = left; i < right; i++) {
    p.first = std::min(p.first, scan.ranges[i]);
    if (p.first == scan.ranges[i]) {
      // will represent an angle from the robot to the line segment
      p.second = i;
    }
  }

  return p;
}

void WallFollowingStrategy::incrementTurn(float m){
  this->setCurrentAngle(this->getCurrentAngle() + m);
}

const geometry_msgs::Twist WallFollowingStrategy::controlMovement() {
  geometry_msgs::Twist msg;
  int minim = INT_MAX;
  int k = 0;

  if (getLaserScan().ranges.size() == 0) {
    return msg;
  }
  int scanSize = WallFollowingStrategy::getLaserScan().ranges.size();

  // the global closest line to the robot
  std::pair< float, float > line =
      WallFollowingStrategy::findMinimDistance(0, scanSize/2);
  // closest line to the left from the robo
  std::pair<float, float> leftRangeLine =
      WallFollowingStrategy::findMinimDistance(scanSize / 2, scanSize - 1);
  // closest line to the right of the robot
  std::pair<float, float> rightRangeLine =
      WallFollowingStrategy::findMinimDistance(0, scanSize / 2);
  std::vector< cv::Vec4i > vec = getLines();

  /* As wall following strategy considers right-hand rule we consider the range to the right
     from the robot. We disregard all the information besides the one lies between [150,210] degree range*/
  std::pair< float, float > right = this->findMinimDistance(scanSize/8, scanSize/3);
  ROS_INFO("Min right %f", right.first);
  // if no data is received yet
  if (!src.data) {
    msg.linear.x = 0;
    return msg;
  }

  // finding of circle is prioritized to other maneuvers
  if (circleVisible && circleDistance != 0) {
    ROS_INFO("Found Circle!");
    circleCallCount++;
    /*if more than 3 reports are registered - move to the circle*/
    if (circleCallCount > CIRCLE_COUNT) {
      circleFoundMode = true;
    }
    // the angle to follow with respect to the norm
    float variationToCircle = 90 - circleAngle;
    msg.angular.z =
        std::min(MAX_TURN, turnCircleCorrection * variationToCircle);
    msg.linear.x = linearVelocity;
    return msg;
  } else {
    circleCallCount = 0;
    //circleFoundMode = false;
  }

  if (crashMode && !circleFoundMode) {
    ROS_INFO("Crash!");
    msg.linear.x = crashHandler.linear.x;
    msg.angular.z = crashHandler.angular.z;

    return msg;
  }

  // movement of the robot in free space
  if (line.first > wallDistance + GLOBAL_WALL_VARIATION && !followWall && !circleFoundMode) {
    msg.linear.x = linearVelocity;
    // move forward from the starting point or in case of being lost in space
    if (start || lostMode){
      return msg;
    }
  }

  // if the robot is too close to the obstacle on its left
  std::cout << leftRangeLine.first << std::endl;
  if (leftRangeLine.first < GLOBAL_WALL_VARIATION && !circleFoundMode) {
    // move backwards from the wall
    msg.angular.z = M_PI;
    msg.linear.x = crashVelocity;
    return msg;
  }

  // when a robot is next to the wall
  if (line.first < wallDistance + GLOBAL_WALL_VARIATION && right.first < 0.5 && !circleFoundMode) {
    start = false;
    for (auto i = 0; i < vec.size(); i++) {
      // take the closest line to the robot with respect to its front
      if (minim < abs(vec[i][0] - src.rows / 2)) {
        minim = abs(vec[i][0] - src.rows / 2);
        k = i;
      }
    }

    //finds the slope of the closest line to the robot with respect to its front
    float den = vec[k][2] - vec[k][0];
    float num = vec[k][3] - vec[k][1];
    float m = 0;
    // if the slope is very small/close to 0 or 90 degrees - turn 90 degrees
    if (den < SLOPE_EPSILON || num < SLOPE_EPSILON || num/den < SLOPE_EPSILON) {
      m = M_PI / 2;
    // otherwise find the slope of the line to align to it
    } else {
      m = num/den;
    }

    // update the current angle of rotation for the robot
    msg.angular.z = (this->getCurrentAngle() + m) / 5;
    incrementTurn(m);
    ROS_INFO("Turning Mode %f", right.first);
    // set the flag of following the wall
    followWall = true;
    return msg;
    // if the robot is approaching the end of the wall or staying too far from the wall
  } else if (right.first > wallDistance + 2*GLOBAL_WALL_VARIATION && !circleFoundMode && followWall) {
    ROS_INFO("Turning End Wall Mode %f", right.first);
    msg.linear.x = linearVelocity - linearVelocity/6;
    msg.angular.z = -M_PI*(linearVelocity/0.3)/ 5;
    // if the robot is far from all the walls cosider it being "lost"
    if (rightRangeLine.first > LOST_THRESHOLD && leftRangeLine.first > LOST_THRESHOLD){
      msg.angular.z = 0;
      lostMode = true;
      followWall = false;
    }
    // turning around the wall
    if (right.first > wallDistance + 3*GLOBAL_WALL_VARIATION){
      return msg;
    // otherwise correcting the movement
    } else {
      this->setCorrecting(true);
    }
  } else {
    msg.linear.x = linearVelocity;
  }

  // in case of deviation turn according to the wall to the right
  if (correcting) {
    float variation = 90 - right.second;
    msg.angular.z = -std::min(MAX_TURN, turnCorrection * variation)/5;
    this->setCorrecting(false);
  }

  return msg;
}