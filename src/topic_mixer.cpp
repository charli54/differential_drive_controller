#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>

double leftEffortValue;
double rightEffortValue;

void saveLeftWheelEffort(const std_msgs::Float64::ConstPtr& twist_msg){
    //cout << "Speed data:" << twist_msg->linear.x << "\n";
    //data = twist_msg->linear.x;
    //publishData = 1;

    leftEffortValue = twist_msg->data;
    
}

void saveRightWheelEffort(const std_msgs::Float64::ConstPtr& twist_msg){

    rightEffortValue = twist_msg->data;    
}


int main(int argc, char** argv){
    ros::init(argc, argv, "topic_mixer");
    ros::NodeHandle n;
    ros::Subscriber subLeftControlEffort  = n.subscribe("/left_wheel/control_effort",  1000,  saveLeftWheelEffort);
    ros::Subscriber subRightControlEffort = n.subscribe("/right_wheel/control_effort", 1000, saveRightWheelEffort);

    ros::Publisher pub_velCMD = n.advertise<geometry_msgs::Twist>("/arduino/cmd_vel", 50);

    ros::Rate rate(50);

    while(n.ok()){

	ros::spinOnce(); 

	geometry_msgs::Twist msg;

	msg.linear.x = leftEffortValue;
	msg.linear.y = rightEffortValue;

	pub_velCMD.publish(msg);

	rate.sleep();

    }

    return 0;
}
