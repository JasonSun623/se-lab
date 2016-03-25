/**@file line_detection.cpp
  *@detail Implementation of line detection
  *
  *@author Sulav Timilsina
  */

/* OpenCV includes */
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <cmath>
#include <ros/ros.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

/**
  * @detail Limiting integers to be within a certain range.
  * used while creating image from laserscan data like in HalfCircleDetection node.
  */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))
#define LASER_RANGE 3.9
#define RANGE_ 10.0
#define STRETCH_FACTOR 100

/*returns this value when slope is infinite*/
#define INFY 999999
 /*a gradient range for finding nearby lines*/
#define EPS 0.1


using namespace cv;
using namespace std;

/** @detail Interpolates the data up to the requested resolution using linear
 * interpolation.
 */
float interpolate(int index, int resolution,
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
 * @detail Converts the laserScan-data from polar into cartesian coordinates.
 * Then cleans the data from various problems and finally translates the points
 * into pixels on an actual image (in form of an OpenCV-matrix).
 */
cv::Mat createOpenCVImageFromLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) /
                    laserScan->angle_increment;

  int imageHeight = 8 * STRETCH_FACTOR;
  int imageWidth = 16 * STRETCH_FACTOR;

  cv::Mat image(imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

  int resolution = 8;
  for (int i = 0; i < numOfValues * resolution; ++i) {
    float hyp =
        interpolate(i, resolution, laserScan->ranges);

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

  //  HalfCircleDetector::points.push_back(std::make_pair(x, y));

    image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(200, 200, 200);
  }

  return image;
}

/**
 * @detail Template function that prints a vector
 */
template <typename T>
void printvec ( vector<T> vec)
{

    for (unsigned int i=0; i<vec.size(); i++)
    {
        cout << vec[i] << endl;
    }
    cout << endl;
}

/**
 * @detail sorts lines on the ascending basis of gradient using
 * insertion sort
 */
void sort_lines (vector<Vec4f>& data, vector<float>& slope)
{
    int i, j;
    float tmp;
    Vec4f temp;
    for (i=1; i<data.size(); i++)
    {
        j=i;
        tmp=slope[i];
        temp=data[i];
        while (j>0 && tmp<slope[j-1])
        {
           data[j]=data[j-1];
           slope[j]=slope[j-1];
           j--;
        }
        data[j]=temp;
        slope[j]=tmp;
    }
}

/**
 * @detail returns average of the Vec4f vector
 */
Vec4f Average(vector<Vec4f> data)
{
    Vec4f res;
    for(unsigned int i=0;i<data.size();i++)
    {
        res+=data[i];
    }
    res[0]=res[0]/data.size();
    res[1]=res[1]/data.size();
    res[2]=res[2]/data.size();
    res[3]=res[3]/data.size();
    return res;
}

/**
 * @detail Removes all the duplicate lines if multiple lines are present in the input vector
 * If the endpoints of lines are within certain RANGE_ only a single line is calculated by taking
 * the average.
 */
vector<Vec4f> removeDuplicateLines(vector<Vec4f>&data)
{
    vector<Vec4f> copyData[data.size()];

    copyData[0].push_back(data[0]);

    int size=0;
    int sizeloc=0;
    int cur=1;
    while(cur<data.size())
    {
        if (abs(copyData[size][0][0]-data[cur][0])<=RANGE_ && abs(copyData[size][0][1]-data[cur][1])<=RANGE_ && abs(copyData[size][0][2]-data[cur][2])<=RANGE_ && abs(copyData[size][0][3]-data[cur][3])<=RANGE_)
        {
            copyData[size].push_back(data[cur]);
            cur++;
            size=0;
        }
        else if (size==sizeloc)
        {
            sizeloc++;
            copyData[sizeloc].push_back(data[cur]);
            size=0;
            cur++;
        }
        else
        {
            size++;
        }
    }

    vector<Vec4f> result;
    Vec4f avg;
    for(unsigned int i=0;i<=sizeloc;i++)
    {
        if (copyData[i].size()==1)
            result.push_back(copyData[i][0]);

        else
        {
            avg=Average(copyData[i]);
            result.push_back(avg);
        }
    }

    return result;
 }

/**
 *@detail returns a vector of gradient that maps to the input vector of Vec4f data
 */
vector<float> get_slope(vector<Vec4f> data)
{
    vector<float>slope;
    for(unsigned int i=0;i<data.size();i++)
    {
         if ((data[i][2]-data[i][0])!=0)
            slope.push_back((data[i][3]-data[i][1])/(data[i][2]-data[i][0]));
         else
            slope.push_back(INFY);
    }
    return slope;
}

/**
 *@detail Returns average of the input vector of points
*/
Point Average(vector<Point>points)
{
    float x,y=0;
    for(unsigned int i=0;i<points.size();i++)
    {
        x+=points[i].x;
        y+=points[i].y;
    }
    x/=points.size();
    y/=points.size();
    return Point(x,y);
}

/**
 *@detail Takes a vector of Vec4f as input and extracts points.
 * Then, all the points in a certain RANGE_ is grouped and averaged.
 * Set of points is returned.
 */
vector<Point>getRefinedPoints(vector<Vec4f>data)
{
    vector<Point>points;

    for(unsigned int i=0;i<data.size();i++)
    {
        points.push_back(Point(data[i][0],data[i][1]));
        points.push_back(Point(data[i][2],data[i][3]));
    }

    vector<Point> copyPoints[points.size()];

    copyPoints[0].push_back(points[0]);

    int size=0;
    int sizeloc=0;
    int cur=1;
    while(cur<points.size())
    {
        if (abs(copyPoints[size][0].x-points[cur].x)<=RANGE_ && abs(copyPoints[size][0].y-points[cur].y)<=RANGE_)
        {
            copyPoints[size].push_back(points[cur]);
            cur++;
            size=0;
        }
        else if (size==sizeloc)
        {
            sizeloc++;
            copyPoints[sizeloc].push_back(points[cur]);
            size=0;
            cur++;
        }
        else
        {
            size++;
        }
    }

    vector<Point> result;
    Point avg;
    for(unsigned int i=0;i<=sizeloc;i++)
    {
        if (copyPoints[i].size()==1)
            result.push_back(copyPoints[i][0]);

        else
        {
            avg=Average(copyPoints[i]);
            result.push_back(avg);
        }
    }

    return result;
}

/**
 *@detail takes refined points and replaces the points in the line if they are
 * within a certain RANGE_
 */
void replacePoints(vector<Vec4f>&lines, vector<Point>&points)
{
    for(int j=0;j<3;j+=2)
    {
        for(int i=0;i<lines.size();i++)
        {
            for(int k=0;k<points.size();k++)
            {
                if (abs(lines[i][j]-points[k].x)<=RANGE_ && abs(lines[i][j+1]-points[k].y)<=RANGE_)
                {
                    lines[i][j]=points[k].x;
                    lines[i][j+1]=points[k].y;
                    break;
                }
            }
        }
    }
}


/**
 *@detail processes vector of lines and joint lines are merged.
 * Everytime two lines are merged, the process again restarts from beginning.
 * In the iteration when none of the lines are merged, we are done.
 */
void processLines(vector<Vec4f>&lines)
{
    vector<Point>points;
    vector<float>slope=get_slope(lines);
    int merger=0;
    vector<Vec4f> newSet;
    int x=0;
    while (merger==0)
    {
        merger=1;
        for(unsigned int i=0;i<lines.size()-1;i++)
        {
            if(i==lines.size()-1)
            {
                newSet.push_back(lines[i]);
                break;
            }
            int j=i+1;
            while(abs(slope[i]-slope[j])<=EPS && j<lines.size())
            {
                if (Point(lines[i][2],lines[i][3])==Point(lines[j][0],lines[j][1]))
                {
                    newSet.push_back(Vec4f(lines[i][0],lines[i][1],lines[j][2],lines[j][3]));
                    merger=0;
                    j+=2;
                    break;
                }
                else if ((Point(lines[i][0],lines[i][1])==Point(lines[j][2],lines[j][3])))
                {
                    newSet.push_back(Vec4f(lines[j][0],lines[j][1],lines[i][2],lines[i][3]));
                    merger=0;
                    j+=2;
                    break;
                }
                else
                    j++;
            }
            if(merger==0)
            {
                for(unsigned int k=j;k<lines.size();k++)
                    newSet.push_back(lines[k]);
                    break;
            }
            newSet.push_back(lines[i]);
        }
        lines=newSet;
        newSet.clear();
        slope=get_slope(lines);
        sort_lines(lines,slope);
    }
}

/**
 *@detail The main function that uses above helper functions.
 * It receives laserScan data, converts it to image and processes the
 * image. It returns set of processed lines.
 */
vector<Vec4f> Points(const sensor_msgs::LaserScan::ConstPtr &laserScan)
{

    Mat image=createOpenCVImageFromLaserScan(laserScan);
    if(image.empty())
    {
        cout << "invalid image "<< endl;
    }

    Mat dst, cdst, c2dst;
    Canny(image, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);

    vector<Vec4f> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 30, 30, 10 );

    lines=removeDuplicateLines(lines);

    vector<Point> points = getRefinedPoints(lines);

    replacePoints(lines,points);

    vector<float>slope=get_slope(lines);

    sort_lines(lines,slope);

    processLines(lines);

    return lines;

}

int main(int argc, char** argv)
{

}
