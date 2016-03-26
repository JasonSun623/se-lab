/**@file line_detection_Check.cpp
  *@detail testing if line_detection.cpp works on Image.jpg
  * 9 lines are detected at first. After processing, 3 lines are seen.
  * Seperate images are created before processing and after processing and shown on screen.
  *@author Sulav Timilsina
  */

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <iostream>
#include <cmath>
 /*for finding nearby points*/
#define RANGE_ 10.0
/*returns this value when slope is infinite*/
#define INFY 999999
 /*a gradient range for finding nearby lines*/
#define EPS 0.1

using namespace std;
using namespace cv;
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
 * @detail prints line with slope
 */
void print(vector<Vec4f>A,vector<float>B)
{
    for(unsigned int i=0;i<A.size();i++)
        cout<<A[i]<<"  "<<B[i]<<endl;
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
 *@detail returns a Vec4f with a minimum and maximum x value and their corresponding y
 */
Vec4f get_Val(Vec4f a, Vec4f b)
{
    float v[]={a[0],a[1],a[2],a[3],b[0],b[1],b[2],b[3]};

    float min[]={a[0],a[1]};

    float max[]={a[0],a[1]};

    for(int i=2;i<8;i+=2)
    {
        if (v[i]<min[0])
        {
            min[0]=v[i];
            min[1]=v[i+1];
        }
        if (v[i]>max[0])
        {
            max[0]=v[i];
            max[1]=v[i+1];
        }
    }

    return Vec4f(min[0],min[1],max[0],max[1]);
}

/**
 *@detail Calculates and returns all the points within a line
 */
vector<Point> getAllPoints(Vec4f line, float slope)
{
    vector<Point>points;

    if (line[0]>line[2])
    {
        swap(line[0],line[2]);
        swap(line[1],line[3]);
    }

    Point p;

    for(float i=line[0];i<=line[2];i++)
    {
        points.push_back(Point(i,(i-line[0])*slope));
    }

    return points;
}

/**
 *@detail returns true of the lines are connected or overlapping. It checks every point
 * of both the lines to find overlapping. The precondition is that the two lines are within
 * a certion slope EPSILON
 */
bool findOverlap(Vec4f lineOne, Vec4f lineTwo, float slopeOne, float slopeTwo)
{
    vector<Point>pointsOne=getAllPoints(lineOne, slopeOne);
    vector<Point>pointsTwo=getAllPoints(lineTwo, slopeTwo);


    for(int i=0;i<pointsOne.size();i++)
    {
        for(int j=0;j<pointsTwo.size();j++)
        {
            if (abs(pointsOne[i].x-pointsTwo[j].x)<=RANGE_ && abs(pointsOne[i].y-pointsTwo[j].y)<=RANGE_)
                return true;
        }
    }
    return false;
}


/**
 *@detail processes vector of lines and joint and overlapping lines are merged.
 * Two lines with slope withon EPSILON with a gap within RANGE_ are merged too.
 * Everytime two lines are merged, the process again restarts from beginning.
 * In the iteration when none of the lines are merged, we are done.
 */
void processLines(vector<Vec4f>&lines)
{
    vector<Point>points;
    vector<float>slope=get_slope(lines);
    sort_lines(lines,slope);
    int merger=0;
    vector<Vec4f> newSet;
    int x=0;
    while (merger==0)
    {
        merger=1;
        for(unsigned int i=0;i<lines.size();i++)
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
                else if(findOverlap(lines[i],lines[j], slope[i], slope[j])==true)
                {
                    Vec4f val=get_Val(lines[i],lines[j]);
                    newSet.push_back(val);
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
 *@detail testing program on Image.jpg
 */
int main(int argc, char** argv)
{

 cv::Mat image = cv::imread("Image.jpg", 0);
 if(image.empty())
 {
     cout << "No file in directory" << endl;
     return -1;
 }

    Mat dst, cdst, c2dst;
    Canny(image, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    cvtColor(dst, c2dst, CV_GRAY2BGR);

    vector<Vec4f> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 30, 30, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( cdst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
    }

    cout<<"Total lines detected:"<<lines.size()<<endl;
    vector<float>slope=get_slope(lines);

    sort_lines(lines,slope);

    print(lines,slope);

    processLines(lines);

    slope=get_slope(lines);
    cout<<"After processing:"<<endl;
    cout<<"Total lines detected:"<<lines.size()<<endl;
    print(lines,slope);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( c2dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
    }

    imshow("source", image);
    imshow("Unrefined hough transform", cdst);
    imshow("Refined hough transform",c2dst);

    waitKey();
    return 0;
}
