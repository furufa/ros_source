#include <iostream>
#include <stdexcept>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/opencv.hpp"

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 377.552737, 0.000000, 331.799012, 0.000000, 374.855213, 255.395621, 0.000000, 0.000000, 1.000000);
	vector<cv::Mat> checkerImgs;
	double apertureWidth = 0, apertureHeight = 0;
	double fovx = 0, fovy = 0;
	double focalLength = 0;
	Point2d principalPoint(0, 0);
	double aspectRatio = 0;


	calibrationMatrixValues(cameraMatrix, Size(640, 480), apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectRatio);
	cout << "focalLength" <<  focalLength << endl;
	cout << "fovx, fovy" << fovx << fovy << endl;
	cout << principalPoint << endl;
	cout << aspectRatio << endl;

}
