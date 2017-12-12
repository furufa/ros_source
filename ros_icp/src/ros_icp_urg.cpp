#include <ros/ros.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <sys/time.h>		/* gettimeofday() */
#include <ipc.h>

using namespace std;

// for IPC
// Number of laser measurement points (URG-04LX)
#define	POINTS_URG04LX	682
// Number of laser measurement points (UTM-30LX)
#define	POINTS_UTM30LX	1081

#define MAXMIN_I_MSG		"maxmin_i"
#define MAXMIN_I_MSG_FMT	"{int, int}"

#define URG_MSG			"urg"
#define URG_MSG_FMT		"{ \
	int, int, <double: 1>, <double: 1>, \
	double, double, double, double, double, double, \
	double, int, "MAXMIN_I_MSG_FMT"}"

typedef struct maxmin_d {
    double max;
    double min;
} maxmin_d;

typedef struct maxmin_f {
    float max;
    float min;
} maxmin_f;

typedef struct maxmin_i {
    int max;
    int min;
} maxmin_i;

/* When *obs_x and *obs_y are at the head of the structure, segmentation fault
   has occured in IPC_publishData(). I don't know why... (by Masahiko) */
typedef struct urg_struct {
    int		max_data_size;	// number of points
    int		id;	// id number
    double	*obs_x;	// [m]: position of measured point related to LRF center 
    double	*obs_y;	// [m]:
    double	x;	// [m]: LRF center position related to robot coord. sys.
    double	y;
    double	z;
    double	roll;	// [rad]: roll: x_axis, pitch: y_axis, yaw: z_axis
    double	pitch;
    double	yaw;
    double	timestamp;
    int		smptime;// sampling time [micro sec]
    maxmin_i	dist;	// max and min of distance [mm]
} urg_struct, *urg_struct_p;

static void	ipc_init(void);
static void	ipc_close(void);
static void	*ipc_listen(void *arg);
static bool	urg_struct_init(urg_struct &urg_st, int num_data);
static void	laserscan2urg_struct(sensor_msgs::LaserScan &media_urg,
				     urg_struct &urg_st);
static double	getmicrotimeofdayd(void);

bool			g_flag_listen;	/* IPC_listen thread */

/* multi-threaded IPC communication */
pthread_mutex_t	g_mutex_ipc;
pthread_mutex_t	g_mutex_urg;
pthread_t		g_ipc_listen_thread;
urg_struct		g_urg;
  
class UrgRosIcp
{
    public:
        UrgRosIcp();
        
    private:
        void urgnodeCallback(const sensor_msgs::LaserScan::ConstPtr& urg);

        ros::NodeHandle nh_;
        ros::Subscriber urgnode_sub_;

};

UrgRosIcp::UrgRosIcp()
{
    urgnode_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("namadata", 20, &UrgRosIcp::urgnodeCallback, this);
}

void UrgRosIcp::urgnodeCallback(const sensor_msgs::LaserScan::ConstPtr& urg)
{
    sensor_msgs::LaserScan media_urg;
    media_urg = *urg;
    //cout << media_urg << endl;

    laserscan2urg_struct(media_urg, g_urg);
    // send urg data
    IPC_publishData(URG_MSG, &g_urg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ros_urg_icp");
    UrgRosIcp ros_urg_icp;

    /* Initialize utm and urg respectively */
    //urg_struct_init(*urg_dt, urg_st);
    
    /* Initialize IPC */
    ipc_init();
    urg_struct_init(g_urg, POINTS_UTM30LX);
    
    /* Start IPC listening and threads */
    g_flag_listen = true;
    if (pthread_create(&g_ipc_listen_thread, NULL, &ipc_listen, NULL) != 0)
	perror("pthread_create()\n");
    
    ros::spin();
}

static void ipc_init(void)
{
    /* Connect to the central server */
    if (IPC_connect("ros_ipc_urt") != IPC_OK) {
	fprintf(stderr, "IPC_connect: ERROR!!\n");
	exit(-1);
    }
    
    IPC_defineMsg(URG_MSG, IPC_VARIABLE_LENGTH, URG_MSG_FMT);
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


static bool urg_struct_init(urg_struct &urg_st, int num_data)
{
    urg_st.obs_x = (double*)malloc(num_data*sizeof(double));
    if (urg_st.obs_x == NULL) {
	fprintf(stderr, "Error: malloc() for obs_x[]\n");
	return false;
    }
    urg_st.obs_y = (double*)malloc(num_data*sizeof(double));
    if (urg_st.obs_y == NULL) {
	fprintf(stderr, "Error: malloc() for obs_y[]\n");
	return false;
    }
    urg_st.x = 0.0;
    urg_st.y = 0.0;
    urg_st.z = 0.0;
    urg_st.roll  = 0.0;
    urg_st.pitch = 0.0;
    urg_st.yaw   = 0.0;
    urg_st.max_data_size = num_data;
    urg_st.id = 0;
    urg_st.smptime = 0.0;
    urg_st.dist.max = 0.0;
    urg_st.dist.min = 0.0;
    
    return true;
}

void laserscan2urg_struct(sensor_msgs::LaserScan &laser_scan,
			  urg_struct &urg_st)
{
    int i;
    int num_data;
    double ang;
    
    num_data = (int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment);
    urg_st.max_data_size = num_data;
    urg_st.id = 0;
    ang = laser_scan.angle_min;
    for (i=0; i<num_data; i++) {
      urg_st.obs_x[i] = -laser_scan.ranges[i] * sin(ang);
      urg_st.obs_y[i] =  laser_scan.ranges[i] * cos(ang);
      ang += laser_scan.angle_increment;
    }
    urg_st.x = 0.0;
    urg_st.y = 0.0;
    urg_st.z = 0.0;
    urg_st.roll  = 0.0;
    urg_st.pitch = 0.0;
    urg_st.yaw   = 0.0;
    
    urg_st.timestamp = getmicrotimeofdayd();
    urg_st.smptime = laser_scan.scan_time;
    urg_st.dist.max = laser_scan.range_max;
    urg_st.dist.min = laser_scan.range_min;
}

double getmicrotimeofdayd(void)
{
    struct timeval	tv;
    struct timezone	tz;
    
    gettimeofday(&tv, &tz);
    
    return ((double)tv.tv_sec * 1000000.0 + (double)tv.tv_usec);
}
