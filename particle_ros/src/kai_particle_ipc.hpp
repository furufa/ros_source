#ifndef NEW_PARTICLE_IPC_HPP
#define NEW_PARTICLE_IPC_HPP

#include <particle_ros/Peoplestat.h>
//#include <particle_ros/Peoplestat2.h>

#define MAX_FILTERS 50

typedef struct {
    int		id;
    int 	active;
    double 	x;
    double 	y;
    double 	theta;
    double 	theta2;
    double 	v;
    double 	dist;
    double 	rho;
    double 	std_x;
    double 	std_y;
    int		within_corridor;/* INSIDE / OUTSIDE */
    int		direction;	/* IN / OUT / REST */
    int 	predict;	/* STOP / PASS / UNSURE */
    int 	bayes_predict;	/* STOP / PASS / UNSURE based on bayes rule */
    bool	flag_greet;	/* true: greeted, false: not greeted yet */
    double	 prob_pass;	/* probability of PASS */
    double	 prob_stop;	/* probability of STOP */
    double	 bayes_pass;	/* probability of PASS based on bayes rule */
    double	 bayes_stop;	/* probability of STOP based on bayes rule */
    double	 Ps, Pp;	/* for bayes */
    double	 PPp, PPs;
    double	 PSp, PSs;
    double	 PpP, PsS;
} people_status;

typedef struct people_stat {
    int		num;
    int		id[MAX_FILTERS];
    int		direction[MAX_FILTERS];	// IN2/OUT2/REST
    int		predict[MAX_FILTERS];	// STOP/PASS/UNSURE
    double	x[MAX_FILTERS];
    double	y[MAX_FILTERS];
} people_stat;

class ParticleRos
{
    public:
        ParticleRos();
        void peoplestatHandler(MSG_INSTANCE ref, void *data, void *dummy);
        ros::Publisher particle_pub_;
    private:
        ros::NodeHandle nh_;
};

ParticleRos::ParticleRos()
{
    particle_pub_ = nh_.advertise<particle_ros::Peoplestat>("people_stat", 1);
}

inline void ParticleRos::peoplestatHandler(MSG_INSTANCE ref, void *data, void *dummy)
{
    
    //extern people_status g_people_stat;
    extern people_stat g_people_stat;
    extern particle_ros::Peoplestat stat;
    int num_status = 0;
    
    //g_people_stat = *(people_status *)data;
    g_people_stat = *(people_stat *)data;

    num_status = g_people_stat.num;
    for(int i=0; i < num_status; i++){
        //std::cout << "-----------------" << std::endl;
        //std::cout << "x" << i << ":" << g_people_stat.x[i] << std::endl;
        //std::cout << "y" << i << ":" << g_people_stat.y[i] << std::endl;
        stat.id = g_people_stat.id[i];
        stat.direction = g_people_stat.direction[i];
        stat.predict = g_people_stat.predict[i];
        stat.x = g_people_stat.x[i];
        stat.y = g_people_stat.y[i];
        particle_pub_.publish(stat);
    }
}

#endif // NEW_PARTICLE_IPC_HPP