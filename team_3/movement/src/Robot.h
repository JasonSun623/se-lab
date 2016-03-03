#include <cstdlib>
#include <iostream>
using namespace std;
#ifndef ROBOT_H
#define ROBOT_H
class Robot{
private:
    double x;
    double y;
    double phi;
    double velX;
    double velY;
    static Robot* robot;
    static bool instanceFlag;

    Robot(){
    	x = 0;
    	y = 0;
    	phi = 0;
    	velX = 0;
    	velY = 0;
    }

     Robot(Robot const&);              // Don't implement
     void operator=(Robot const&); // Don't implement


public:
	static Robot* getInstance(){
		if(!instanceFlag){
			robot = new Robot();
			instanceFlag = true;
			return robot;
		} else {
			return robot;
		}
	}
	
	void setX(double newX){
		x = newX;
	}

	double getX(){
		return x;
	}

	void setY(double newY){
		y = newY;
	}

	double getY(){
		return y;
	}

	void setPhi(double newPhi){
		phi = newPhi;
	}

	double getPhi(){
		return phi;
	}

	void setVelX(double newVel){
		velX = newVel;
	}

	double getVelX(){
		return velX;
	}

	void setVelY(double newVel){
		velY = newVel;
	}

	double getVelY(){
		return velY;
	}

	~Robot()
    {
        instanceFlag = false;
    }

};

bool Robot::instanceFlag = false;
#endif // ROBOT_H
