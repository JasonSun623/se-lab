#include <cstdlib>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
using namespace std;
#ifndef LASER_H
#define LASER_H

class LaserScanPublisher{
private:
	int num_readings;
	double laser_frequency;
	sensor_msgs::LaserScan scan;

public:
	LaserScanPublisher(int num_readings, double laser_frequency){
		this->num_readings = num_readings;
		this->laser_frequency = laser_frequency;
	}

	int getNum(){
		return num_readings;
	}

	void setNum(int num_readings){
		this->num_readings = num_readings;
	}

	double getFreq(){
		return laser_frequency;
	}

	void setFreq(double laser_frequency){
		this->laser_frequency = laser_frequency;
	}

	sensor_msgs::LaserScan getScan(){
		return this->scan;
	}

	void setScan(sensor_msgs::LaserScan scan){
		this->scan = scan;
	}

	void initializeLaser(ros::Time scan_time){
		scan.header.stamp = scan_time;
    		scan.header.frame_id = "laser_frame";
    		scan.angle_min = -1.57;
    		scan.angle_max = 1.57;
    		scan.angle_increment = 3.14 / num_readings;
    		scan.time_increment = (1 / laser_frequency) / num_readings;
    		scan.range_min = 0.0;
    		scan.range_max = 100.0;
	}

	void setRanges(sensor_msgs::LaserScan input){
		scan.ranges.resize(num_readings);
		for(int i = 0; i < num_readings; i++){
			scan.ranges[i] = input.ranges[i];
		}
	}

	void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr &laserScan) {
    		this->setRanges(*laserScan);
	}

};
#endif 
