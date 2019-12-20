
#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "sensor_msgs/LaserScan.h"
#include "acc_logic_ros/MsgACC.h" 
#include <pthread.h>

#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;

#define REQ_CUR 0
#define REQ_ACCEL 1
#define REQ_DECEL 2
#define REQ_STOP 3

string target = "person";
bool detect_target = false;
int req_num = REQ_CUR; //0: current 1: accel 2 : decel 3 : stop
acc_logic_ros::MsgACC msg;
ros::Publisher req_pub;

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<endl;
    if(msg->bounding_boxes[0].Class.compare(target) == 0) {
	cout<<">>>>TARGET (Class):" << msg->bounding_boxes[0].Class <<endl;
        detect_target = true;
    } else {
        detect_target = false;
    }
}

#if 1
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
        int index = 0;;
	float range = msg->ranges[index];
	ROS_INFO("range = %lf :\n",msg->ranges[index]);
	req_num = REQ_CUR;
	
	//if((330<index || index<30) && mRange <= 0.2){
	if(range <= 0.6){
		req_num = REQ_STOP;
 		
		ROS_INFO("range = %lf : Emergency! STOP!\n",msg->ranges[index]);;
	}
}

void *test(void *data)
{
    ros::Rate loop_rate(20);
  while (ros::ok())
  {
    msg.stamp = ros::Time::now();        
    msg.acc_cmd  = req_num;                 

    req_pub.publish(msg);        
	
    loop_rate.sleep();                  

  }
}
#endif

int main(int argc, char **argv)
{
    cout<<"acc_logic_node2" <<endl;
#if 0
    ros::init(argc, argv, "acc_logic_node2");
    ros::NodeHandle nh;
    ros::Subscriber obj_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);

	ros::spin();
#endif
#if 1

    pthread_t thread_t;
    int status;

    cout<<"acc_logic_node2" <<endl;

    ros::init(argc, argv, "acc_logic_node2");
    ros::NodeHandle nh;
    ros::Subscriber obj_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	
    req_pub = nh.advertise<acc_logic_ros::MsgACC>("acc_logic_msg", 100);

	
    if (pthread_create(&thread_t, NULL, test, 0) < 0)
    {
        printf("thread create error:");
        exit(0);
    }


    ros::spin();

    pthread_join(thread_t, (void **)&status);
    printf("Thread End %d\n", status);
#endif

	return 0;
}

