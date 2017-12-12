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

using namespace cv;
using namespace std;

//double focalLength = 0;
//std_msgs::Float64 angle1 = std_msgs::Float64(0.0), angle2 = std_msgs::Float64(0.0);
//std_msgs::Float64 angle1, angle2;

std_msgs::Float64 angle1;
std_msgs::Float64 angle2;
//int a = 0;

const char* keys =
{
    "{ help h      |                     | print help message }"
    "{ image i     |                     | specify input image}"
    "{ camera c    | 0                    | enable camera capturing }"
    "{ video v     | data/vtest.avi   | use video as input }"
    "{ directory d |                     | images directory}"
};

//class AngleDetect;
//void AngleDetect::detectAndDraw(const HOGDescriptor &hog, Mat &img);


static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();

	double focalLength = 377.553;
	//double angle1 = 0, angle2 = 0;
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
		
		angle1.data = atan(r.x/focalLength);
		angle2.data = atan((r.x+r.width)/focalLength);

		//angle1_pub_.publish(angle1);
		//angle2_pub_.publish(angle2);
    }
}
class AngleDetect
{
	public:
		AngleDetect();
		//static void detectAndDraw(const HOGDescriptor &hog, Mat &img);
		ros::Publisher angle1_pub_;
		ros::Publisher angle2_pub_;
	private:
		ros::NodeHandle nh_;
		

};

AngleDetect::AngleDetect() {
	angle1_pub_ = nh_.advertise<std_msgs::Float64>("angle1", 1);
	angle2_pub_ = nh_.advertise<std_msgs::Float64>("angle2", 2);

}
/*
static void AngleDetect::detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();

	double focalLength = 377.553;
	//double angle1 = 0, angle2 = 0;
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
		
		angle1 = atan(r.x/focalLength)*180/M_PI;
		angle2 = atan((r.x+r.width)/focalLength)*180/M_PI;

		//angle1_pub_.publish(angle1);
		//angle2_pub_.publish(angle2);
    }
}
*/

int main(int argc, char** argv)
{
	/*	
	cv::VideoCapture cap(0); // device open

	if(!cap.isOpened()) {	// open clear check
		// failed
		return -1;
	}
	*/

	angle1.data = angle2.data = 0.0;

    CommandLineParser parser(argc, argv, keys);

	// ROS
	ros::init(argc, argv, "angle_detect");
	AngleDetect angle_detect;


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

	cout << "hello" << endl;
    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    namedWindow("people detector", 1);

    string pattern_glob = "";
    string video_filename = "data/vtest.avi";
    int camera_id = -1;
    if (parser.has("directory")) // .hasで中身があるか確認
    {
        pattern_glob = parser.get<string>("directory");
    }
    else if (parser.has("image"))
    {
        pattern_glob = parser.get<string>("image");  // getで中身を取得
    }
    else if (parser.has("camera"))
    {
        camera_id = parser.get<int>("camera");
    }
    else if (parser.has("video"))
    {
        video_filename = parser.get<string>("video");
		cout << video_filename << endl;
    }

    if (!pattern_glob.empty() || camera_id != -1 || !video_filename.empty())
    {
        //Read from input image files
        vector<String> filenames;
        //Read from video file
        VideoCapture vc;
        Mat frame;

        if (!pattern_glob.empty())
        {
            String folder(pattern_glob);
            glob(folder, filenames);
        }
        else if (camera_id != -1)
        {
            vc.open(camera_id);
            if (!vc.isOpened())
            {
                stringstream msg;
                msg << "can't open camera: " << camera_id;
                throw runtime_error(msg.str());
            }
        }
        else
        {
            vc.open(video_filename.c_str());
            if (!vc.isOpened())
                throw runtime_error(string("can't open video file: " + video_filename));
        }

        vector<String>::const_iterator it_image = filenames.begin();

        for (;;)
        {
            if (!pattern_glob.empty())
            {
                bool read_image_ok = false;
                for (; it_image != filenames.end(); ++it_image)
                {
                    cout << "\nRead: " << *it_image << endl;
                    // Read current image
                    frame = imread(*it_image);

                    if (!frame.empty())
                    {
                        ++it_image;
                        read_image_ok = true;
                        break;
                    }
                }

                //No more valid images
                if (!read_image_ok)
                {
                    //Release the image in order to exit the while loop
                    frame.release();
                }
            }
            else
            {
                vc >> frame;
            }

            if (frame.empty())
                break;

            //angle_detect.detectAndDraw(hog, frame);
			detectAndDraw(hog, frame);
			//前と値が一緒だったら送らない条件分岐が必要
			angle_detect.angle1_pub_.publish(angle1);
			angle_detect.angle2_pub_.publish(angle2);

            imshow("people detector", frame);
            int c = waitKey( vc.isOpened() ? 30 : 0 ) & 255;
            if ( c == 'q' || c == 'Q' || c == 27)
                break;
        }
    }

    //return 0;
	ros::spin();
}

/*
static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();

	double focalLength = 377.553;
	double angle1 = 0, angle2 = 0;
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
		
		angle1 = atan(r.x/focalLength)*180/M_PI;
		angle2 = atan((r.x+r.width)/focalLength)*180/M_PI;

		angle_detect.angle1_pub_.publish(angle1);
		angle_detect.angle2_pub_.publish(angle2);
		//angle1_pub_.publish(angle1);
		//angle2_pub_.publish(angle2);
    }
}
*/
