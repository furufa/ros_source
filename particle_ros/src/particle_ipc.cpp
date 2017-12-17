#include <ros/ros.h>
#include <string>
#include <sys/time.h>
#include <ipc.h>

#include <particle_ros/Peoplestat.h>

/* people status for decision trees */
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

using namespace std;

#define PEOPLE_STAT_MSG "people_stat"
#define PEOPLE_STAT_MSG_FMT "{int, [int: 50], [int: 50], [double: 50], [double: 50]}"

static void ipc_init(void);
static void ipc_close(void);
static void *ipc_listen(void *arg);
void        peoplestatHandler(MSG_INSTANCE ref, void *data, void *dummy);
void        sigcatch(int sig);

bool    g_flag_listen; // IPC_listen thread
bool	g_flag_publish;
bool	g_flag_etc;

// multi-threaded IPC communication 
pthread_mutex_t g_mutex_ipc;
pthread_mutex_t g_mutex_urg;
pthread_t g_ipc_listen_thread;
pthread_t g_ipc_publish_thread;
pthread_t g_etc_thread;

people_status g_people_stat;

particle_ros::Peoplestat stat;

int flag = 0;

//ros::NodeHandle nh_;
//ros::Publisher particle_pub_;

class ParticleRos
{
    public:
        ParticleRos();
        //void peoplestatHandler(MSG_INSTANCE ref, void *data, void *dummy);
        //static void ipc_init(void);
        ros::Publisher particle_pub_;
    private:
        ros::NodeHandle nh_;
        //ros::Publisher particle_pub_;
};

ParticleRos::ParticleRos()
{
    // new message define required
    particle_pub_ = nh_.advertise<particle_ros::Peoplestat>("people_stat", 1);
}

//ParticleRos hoge;


int main(int argc, char** argv){
    ros::init(argc, argv, "particle_ros");
    //ParticleRos hoge;
    //particle_pub_ = nh_.advertise<particle_ros::Peoplestat>("people_stat", 1);
    //ParticleRos particle_ros;

    // Initialize IPC
    ipc_init();

    // Start IPC listening and threads
    g_flag_listen = true;
    if(pthread_create(&g_ipc_listen_thread, NULL, &ipc_listen, NULL) != 0)
    perror("pthread_create()\n");
    cout << "publish yeaahhhhhhh" << endl;
    //hoge.particle_pub_.publish(stat);

    ros::spin();

}

static void ipc_init(void)
{
    /* Connect to the central server */
    if (IPC_connect("ros_ipc_urt") != IPC_OK) {
	fprintf(stderr, "IPC_connect: ERROR!!\n");
	exit(-1);
    }
    
    IPC_defineMsg(PEOPLE_STAT_MSG, IPC_VARIABLE_LENGTH, PEOPLE_STAT_MSG_FMT);
    IPC_subscribeData(PEOPLE_STAT_MSG, peoplestatHandler, NULL);
    cout << "ipc_init yeahhhhhhhhhhhhhh" << endl;
}

static void ipc_close(void)
{
    /* close IPC */
    printf("Close IPC connection\n");
    IPC_disconnect();
}


static void *ipc_listen(void *arg)
{
    int		cnt=0;
    
    fprintf(stderr, "Start ipc_listen\n");
    
    g_flag_listen = true;
    while (g_flag_listen == true) {
	if (IPC_isConnected() == false) {
	    usleep(20*1000);		/* 20 [ms] */
	    continue;
	}
	
	pthread_mutex_lock(&g_mutex_ipc);
	IPC_listen(10);	/* 20 is stable */
	pthread_mutex_unlock(&g_mutex_ipc);
	
	if (cnt % 50 == 0)
	    fprintf(stderr, "IPC_listen: true\n");
	cnt++;
	usleep(10*1000);
    }
    
    fprintf(stderr, "Stop ipc_listen\n");
}

void sigcatch(int sig)
{
  fprintf(stderr, "Catch signal %d\n", sig);

  /* wait thread join */
  g_flag_publish = false;
  g_flag_listen = false;
  g_flag_etc = false;
  pthread_join(g_ipc_publish_thread, NULL);
  pthread_join(g_ipc_listen_thread, NULL);
  pthread_join(g_etc_thread, NULL);

  /* Close IPC */
  ipc_close();

  pthread_mutex_destroy(&g_mutex_ipc);

  exit(1);
}


void peoplestatHandler(MSG_INSTANCE ref, void *data, void *dummy)
{
    cout << "start handler" << endl;
    g_people_stat = *(people_status *)data;
    stat.id = g_people_stat.id;
    stat.active = g_people_stat.active;
    stat.x = g_people_stat.x;
    stat.y = g_people_stat.y;
    stat.theta = g_people_stat.theta;
    stat.theta2 = g_people_stat.theta2;
    //hoge.particle_pub_.publish(stat);
    cout << "handlerrrr yeah" << endl;
}