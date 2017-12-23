#include <ros/ros.h>
#include <string>
#include <sys/time.h>
#include <ipc.h>
#include "kai_particle_ipc.hpp"

#include <particle_ros/Peoplestat.h>
//#include <particle_ros/Peoplestat2.h>

using namespace std;

#define PEOPLE_STAT_MSG "people_stat"
#define PEOPLE_STAT_MSG_FMT "{int, [int: 50], [int: 50], [double: 50], [double: 50]}"

//people_status g_people_stat;
people_stat g_people_stat;
particle_ros::Peoplestat stat;

static void ipc_init(ParticleRos &status); // fool
static void ipc_close(void);
static void *ipc_listen(void *arg);
void dummy_callback_function(MSG_INSTANCE ref, void *data, void *dummy);
//void        peoplestatHandler(MSG_INSTANCE ref, void *data, void *dummy);
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



int main(int argc, char** argv){
    ros::init(argc, argv, "particle_ros");
    ParticleRos hoge;
    //particle_pub_ = nh_.advertise<particle_ros::Peoplestat>("people_stat", 1);
    //ParticleRos particle_ros;

    // Initialize IPC
    ipc_init(hoge);

    // Start IPC listening and threads
    g_flag_listen = true;
    if(pthread_create(&g_ipc_listen_thread, NULL, ipc_listen, NULL) != 0)
    perror("pthread_create()\n");
    cout << "publish yeaahhhhhhh" << endl;
    //hoge.particle_pub_.publish(stat);

    ros::spin();

}

//ros::init(argc, argv, "particle_ros");
//ParticleRos hoge;

static void ipc_init(ParticleRos &status)
{
    /* Connect to the central server */
    if (IPC_connect("ros_ipc_urt") != IPC_OK) {
	fprintf(stderr, "IPC_connect: ERROR!!\n");
	exit(-1);
    }

    // peoplestatus class declare
    //people_status media_status;
    //media_status = status;
    
    IPC_defineMsg(PEOPLE_STAT_MSG, IPC_VARIABLE_LENGTH, PEOPLE_STAT_MSG_FMT);
    IPC_subscribeData(PEOPLE_STAT_MSG, dummy_callback_function, (void *)&status);
    //IPC_subscribeData(PEOPLE_STAT_MSG, dummy_callback_function, NULL); fool
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

//間にダミー噛ませるよ fool
void dummy_callback_function(MSG_INSTANCE ref, void *data, void *dummy)
{
    ParticleRos *hoge = reinterpret_cast<ParticleRos*>(dummy);
    hoge->peoplestatHandler(ref, data, dummy);
}