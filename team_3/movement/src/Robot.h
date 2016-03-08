#include <cstdlib>
#include <iostream>
using namespace std;
#ifndef ROBOT_H
#define ROBOT_H
/** @file Robot.h
 *  @brief Introduces a robot in the simulation and gives it
 *  parameters
 *
 *  Implemented as a singleton class to ensure existance of one robot
 *
 *  Returns a position of the robot with respect to the initial
 *  state and its current velocity
 *
 *  Please note that the coordinates of the robot are not defined
 *  Mapping is planned to be done later
 *
 * @author Mariia Gladkova
 */

class Robot {
private:
  double x;
  double y;
  double phi;
  static Robot *robot;
  static bool instanceFlag;

  /** @brief A default constructor for Robot object
   */
  Robot() {}

  Robot(Robot const &);          // Don't implement
  void operator=(Robot const &); // Don't implement

public:
  /** @brief Returns a unique instance of the class
   *  @return Robot instance of the class
   */
  static Robot *getInstance() {
    if (!instanceFlag) {
      robot = new Robot();
      instanceFlag = true;
      return robot;
    } else {
      return robot;
    }
  }
  void initialize() {
    this->setX(0);
    this->setY(0);
    this->setPhi(0);
  }

  /** @brief Sets the x coordinate of the robot
   *  @param new x coordinate to be set
   */
  void setX(double newX) { x = newX; }

  /** @brief Gives the x coordinate of the robot
   *  @return x coordinate of the robot
   */

  double getX() { return x; }

  /** @brief Sets the y coordinate of the robot
   *  @param new y coordinate to be set
   */

  void setY(double newY) { y = newY; }

  /** @brief Gives the y coordinate of the robot
   *  @return y coordinate of the robot
   */

  double getY() { return y; }

  /** @brief Sets the angle to be turned angle by the robot
   *  @param new angle to be turned
   */

  void setPhi(double newPhi) { phi = newPhi; }

  /** @brief Gives the angle to be turned by the robot
   *  @return y coordinate of the robot
   */

  double getPhi() { return phi; }

  ~Robot() { instanceFlag = false; }

  /** @brief Sets a message to cmd_vel to movement in x direction
   *  @param message to cmd_vel, new move in x direction (distance in meters)
   */

  void driveX(geometry_msgs::Twist &msg, double x) { msg.linear.x = x; }

  /** @brief Sets a message to cmd_vel to movement in y direction
   *  @param message to cmd_vel, new move in y direction (distance in meters)
   */

  void driveY(geometry_msgs::Twist &msg, double y) { msg.linear.y = y; }

  /** @brief Sets a message to cmd_vel to turn
   *  @param message to cmd_vel, new turn clockwise
   */

  void turn(geometry_msgs::Twist &msg, double phi) { msg.angular.z = phi; }

  void getFlag() {
    if (instanceFlag)
      ROS_INFO("Instance created");
    else
      ROS_INFO("Instance does not exist");
  }
};
// initially no instance created

bool Robot::instanceFlag = false;
#endif // ROBOT_H
