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
//    circleSeenCount = std::max(circleSeenCount - 1, 0);
//    if(circleSeenCount == 0) {
      circleVisible = false;
      //circleFoundMode = false;
//    }
    return;
  }

//  circleSeenCount = std::min(circleSeenCount + 1, 4);
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

bool WallFollowingStrategy::lineCondition(std::pair<cv::Vec4i, float> line, cv::Vec4i initialLineSegment){
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
    if (!temp.empty()){
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
      //ROS_INFO("Min %f\n", scan.ranges[i]);
    }
  }

  return p;
}

const geometry_msgs::Twist WallFollowingStrategy::controlMovement() {
  geometry_msgs::Twist msg;
  int minim = INT_MAX;
  float theta = 0;
  int k = 0;

  if (getLaserScan().ranges.size() == 0) {
    return msg;
  }

  int scanSize = WallFollowingStrategy::getLaserScan().ranges.size();
  // the global closest line to the robot
  std::pair<float, float> rightRangeLine =
      WallFollowingStrategy::findMinimDistance(0, scanSize/2);
  std::pair<float, float> leftRangeLine =
      WallFollowingStrategy::findMinimDistance(scanSize/2, scanSize -1);
  std::vector<cv::Vec4i> vec = getLines();

  // closest line segment on the right range of laser scan with respect to the
  // robot
  std::pair<float, float> right = this->findMinimDistance(scanSize/4, scanSize/3);

  // if no data is received yet
  if (!src.size().height) {
    msg.linear.x = 0;
    return msg;
  }

  if (crashMode) {
    ROS_INFO("Crash!");
    msg.linear.x = crashHandler.linear.x;
    msg.angular.z = crashHandler.angular.z;
    return msg;
  }

  // finding of circle is prioritized to other maneuvers
  if (circleVisible && circleDistance != 0) {
    ROS_INFO("Found Circle!");
    circleCallCount++;
    if (circleCallCount > 2){
      circleFoundMode = true; 
    }
    // the angle to follow with respect to the norm
    float variationToCircle = 90 - circleAngle;
    msg.angular.z = std::min(MAX_TURN, TURN_CIRCLE_CORRECTION * variationToCircle);
    msg.linear.x = LINEAR_VELOCITY;
    return msg;
    
  }

  //TODO: create macros/variables for all magic numbers here (0.3,0.5,...)
  // movement of the robot in free space
  if (rightRangeLine.first > WALL_DISTANCE && !followWall && !circleFoundMode) {
    msg.linear.x = LINEAR_VELOCITY;
    return msg;
  }

  // if the robot is too close to the obstacle on its left
  if (leftRangeLine.first < WALL_DISTANCE/3){
    msg.angular.z = M_PI / 10;
    msg.linear.x = CRASH_VELOCITY;
    // case if a robot is too close to the circle
    if (circleFoundMode){
      circleFoundMode = false;
      circleCallCount = 0;
    }
    return msg;
  }

  // when a robot is next to the wall
  if (rightRangeLine.first < WALL_DISTANCE && right.first < WALL_DISTANCE * 2 && !circleFoundMode) {
    for (auto i = 0; i < vec.size(); i++) {
      // take the closest line to the robot with respect to its front
      if (minim < abs(vec[i][0] - src.rows / 2)) {
        minim = abs(vec[i][0] - src.rows / 2);
        k = i;
      }
    }

    float den = vec[k][2] - vec[k][0];
    float num = vec[k][3] - vec[k][1];
    float m = 0;
    if (den < EPSILON_SLOPE || num < EPSILON_SLOPE) {
      m = M_PI / 2;
    }

    msg.angular.z = (this->getCurrentAngle() + m) / 10;
    this->setCurrentAngle(this->getCurrentAngle() + m);

    followWall = true;
    return msg;
    // if the robot is approaching the end of the wall
  } else if (right.first > WALL_DISTANCE * 1.5 && !circleFoundMode) {
    cornerEdge = true;
    msg.linear.x = LINEAR_VELOCITY/2;
    if (!circleVisible)
      msg.angular.z = -M_PI / 10;
    else {
      // angle is less than for usual case as we want to
      // move towards the goal
      msg.angular.z = -M_PI / 5;
    }
    return msg;
  }

  // robot is following the wall and deviates from it
  if (right.first > WALL_DISTANCE * 1.5 && !circleFoundMode) {
    this->setCorrecting(true);
    followWall = false;
  } else {
    msg.linear.x = LINEAR_VELOCITY;
  }

  // in case of deviation turn according to the wall to the right
  if (correcting) {
    float variation = 90 - right.second;
    msg.angular.z = std::min(MAX_TURN, TURN_CORRECTION * variation);
    this->setCorrecting(false);
  }
  return msg;
}

