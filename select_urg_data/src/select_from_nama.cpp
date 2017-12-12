#include <ros/ros.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
//#include <urg_node/Status.h>
#include <hog_peopledetect/DetectAngle.h>

hog_peopledetect::DetectAngle select_angle;
int flag = 0;

class SelectUrgData
{
	public:
		SelectUrgData();

	private:
		void namadataCallback(const sensor_msgs::LaserScan::ConstPtr& nama);
		//void angle1Callback(const std_msgs::Float64::ConstPtr& angle1);
		//void angle2Callback(const std_msgs::Float64::ConstPtr& angle2);
		void angleCallback(const hog_peopledetect::DetectAngle::ConstPtr& angle);

		int linear_, angular_;
		ros::NodeHandle nh_;
		ros::Publisher select_pub_;
		ros::Subscriber nama_sub_;
		//ros::Subscriber angle1_sub_;
		//ros::Subscriber angle2_sub_;
		ros::Subscriber angle_sub_;

};

SelectUrgData::SelectUrgData()
{
	select_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 20);
	nama_sub_   = nh_.subscribe<sensor_msgs::LaserScan>("namadata", 20, &SelectUrgData::namadataCallback, this);	
	angle_sub_ = nh_.subscribe<hog_peopledetect::DetectAngle>("angle", 20, &SelectUrgData::angleCallback, this);
}

void SelectUrgData::angleCallback(const hog_peopledetect::DetectAngle::ConstPtr& angle) {
	select_angle.left_angle = angle->left_angle;
	select_angle.right_angle = angle->right_angle;
	flag = 1;
}

void SelectUrgData::namadataCallback(const sensor_msgs::LaserScan::ConstPtr& nama)
{
	sensor_msgs::LaserScan select;
	//select_angle.left_angle = 0;
	//select_angle.right_angle = 0;
	int detect_element = 0;
	select = *nama;
	if(flag == 1){
		detect_element = (select_angle.left_angle + 2.356)/0.006;
		select.ranges[detect_element] = 0.0/0.0;
		select.ranges[detect_element+1] = 0.0/0.0;
	}
	select_pub_.publish(select);
	flag = 0;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "select_urg_data");
	select_angle.left_angle = 0;
	select_angle.right_angle = 0;
	SelectUrgData select_urg_data;

	ros::spin();
}
