#include <stdexcept>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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

// invert to opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW  = "Image window";

using namespace cv;
using namespace std;

hog_peopledetect::DetectAngle angle;
HOGDescriptor hog;
Mat frame; 
//AngleDetect angle_detect;

const char* keys =
{
    "{ help h      |                     | print help message }"
    "{ image i     |                     | specify input image}"
    "{ camera c    | 0                    | enable camera capturing }"
    "{ video v     | data/vtest.avi   | use video as input }"
    "{ directory d |                     | images directory}"
};


static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();

	double focalLength = 377.553;
    // Run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    //hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
    t = (double) getTickCount() - t;
    cout << "detection time = " << (t*1000./cv::getTickFrequency()) << " ms" << endl;

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
        rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 3);

		cout << atan(r.x/focalLength)*180/M_PI << endl;
		cout << "correct" << endl;
		cout << atan((r.x+r.width)/focalLength)*180/M_PI << endl;
		cout << "correct2" << endl;
		
		angle.left_angle  = atan(r.x/focalLength);
		angle.right_angle = atan((r.x+r.width)/focalLength);

    }
}


class AngleDetect
{
	public:
		AngleDetect();
		//static void detectAndDraw(const HOGDescriptor &hog, Mat &img);
		ros::Publisher angle_pub_;
	//private:
		//ros::NodeHandle nh_;
		

};

AngleDetect::AngleDetect() {
	angle_pub_ = nh_.advertise<hog_peopledetect::DetectAngle>("angle", 1);
}


//AngleDetect angle_detect;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //Mat frame; 
  //image_transport::Publisher image_pub_;

public:
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
				&ImageConverter::imageCb, this);
		//image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
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
		//cout << frame << endl;
		
		/*
		if (parser.has("help"))
		{
			cout << "\nThis program demonstrates the use of the HoG descriptor using\n"
				" HOGDescriptor::hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());\n";
			parser.printMessage();
			cout << "During execution:\n\tHit q or ESC key to quit.\n"
				"\tUsing OpenCV version " << CV_VERSION << "\n"
				"Note: camera device number must be different from -1.\n" << endl;
			return 0;
		}
		*/

		//HOGDescriptor hog;
		/*
		hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
		namedWindow("people detector", 1);
		
        for (;;)
        {

            if (frame.empty())
                break;

            //angle_detect.detectAndDraw(hog, frame);
			detectAndDraw(hog, frame);
			//前と値が一緒だったら送らない条件分岐が必要
			angle_detect.angle_pub_.publish(angle);

            imshow("people detector", frame);
        }
		*/

	}
};


int main(int argc, char** argv)
{
	angle.left_angle = 0.0;
	angle.right_angle = 0.0;

    //CommandLineParser parser(argc, argv, keys);

	// ROS
	ros::init(argc, argv, "hog_angle");
	ImageConverter ic;
	AngleDetect angle_detect;
	//ImageConverter ic;

	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	namedWindow("people detector", 1);

	for (;;)
	{
		cout << "detect start" << endl;
		if (frame.empty()){
			cout << "frame empty" << endl;
			break;
		}
		//angle_detect.detectAndDraw(hog, frame);
		cout << "before detect" << endl;
		detectAndDraw(hog, frame);
		//前と値が一緒だったら送らない条件分岐が必要
		angle_detect.angle_pub_.publish(angle);

		imshow("people detector", frame);
	}

    //return 0;
	ros::spin();
}
