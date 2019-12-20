#include "ros/ros.h"
#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;

string target = "cup";
bool detect_target = false;
bool req_stop = false;

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
   // cout<<"Bouding Boxes (header):" << msg->header <<endl;
   // cout<<"Bouding Boxes (image_header):" << msg->image_header <<endl;
    cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<endl;
    //cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[0].xmin <<endl;
    //cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[0].xmax <<endl;
    //cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[0].ymin <<endl;
    //cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[0].ymax <<endl;
    if(msg->bounding_boxes[0].Class.compare(target) == 0) {
	cout<<">>>>TARGET (Class):" << msg->bounding_boxes[0].Class <<endl;
        detect_target = true;
    } else {
        detect_target = false;
    }
    //cout << "\033[2J\033[1;1H";     // clear terminal
}
#if 0
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    
    static float detect_degree;
    static float detect_range;
    req_stop = false;

    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		//if(degree > 100 && degree < 150)
	if (scan->ranges[i] < 1 && (degree > -100 && degree < -90)) {
		//ROS_INFO("0: [%f, %f]", degree, scan->ranges[i]);
		if(detect_target == true) {
                     req_stop = true;
                }
        }
    }
    if( req_stop == true) {
         ROS_INFO(">>>>>>>>>>>>>>>>>>>>");
         ROS_INFO(">>>>>>STOP: [%f, %f]", detect_degree, detect_range);
         ROS_INFO(">>>>>>>>>>>>>>>>>>>>");
    }
}
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obj_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber obj_sub = nh.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
	
    ros::spin();

	return 0;
}

