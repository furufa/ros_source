#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdexcept>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/opencv.hpp"

// for calibration.cpp
#include <sstream>
#include <math.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hog_peopledetect/DetectAngle.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

hog_peopledetect::DetectAngle angle;

static void detectAndDraw(const cv::HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();

	// check
	double focalLength = 377.553;
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
    t = (double) getTickCount() - t;
    cout << "detection time = " << (t*1000./getTickFrequency()) << " ms" << endl;

    for(size_t i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];

        size_t j;
        // Do not add small detections inside a bigger detection.
        for ( j = 0; j < found.size(); j++ )
            if ( j != i && (r & found[j]) == r )
                break;

        if ( j == found.size() )
            found_filtered.push_back(r);
    }

    for (size_t i = 0; i < found_filtered.size(); i++)
    {
        Rect r = found_filtered[i];

        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
		cout << "r.x=" << r.x << ", r.width=" << r.width << ", r.y=" << r.y <<", r.height=" << r.height << endl;
        rectangle(img, r.tl(), r.br(), Scalar(0,255,0), 3);

		cout << atan(r.x/focalLength)*180/M_PI << endl;
		cout << "correct" << endl;
		cout << atan((r.x+r.width)/focalLength)*180/M_PI << endl;
		cout << "correct2" << endl;
		
		angle.left_angle  = atan(r.x/focalLength);
		angle.right_angle = atan((r.x+r.width)/focalLength);

    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher angle_pub_;

public:
  	//ros::Publisher angle_pub_;
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
				&ImageConverter::imageCb, this);
		angle_pub_ = nh_.advertise<hog_peopledetect::DetectAngle>("angle", 1);

		namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		Mat frame;
		HOGDescriptor hog;
		hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}	
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		frame = cv_ptr->image;
		detectAndDraw(hog, frame);
		angle_pub_.publish(angle);
		// Update GUI Window
		imshow(OPENCV_WINDOW, cv_ptr->image);
		waitKey(3);

		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
	}
};
				
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
