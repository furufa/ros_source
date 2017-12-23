#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <angle_particle/Peoplestat.h>

using namespace std;

angle_particle::Peoplestat stat;

class AngleParticle
{
    public:
        AngleParticle();
        ros::Publisher angle_pub_;
        ros::Subscriber xy_sub_;
    private:
        void xyCallback(const particle_ros::Peoplestat::ConstPtr& stat);
        ros::NodeHandle nh_;
};

AngleParticle::AngleParticle()
{
    angle_pub_ = nh_.advertise<std_msgs::Float64>("particle_angle", 2);
    xy_sub_ = nh_.subscribe<particle_ros::Peoplestat>("people_stat", 1);
}

void AngleParticle::xyCallback(const particle_ros::Peoplestat::ConstPtr& status)
{
    double angle = 0;
    stat = status;

    angle = atan(stat.x, stat.y);
    angle_pub_.publish(angle);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_pub_particle");
    AngleParticle angle_particle;

    ros::spin()
}
