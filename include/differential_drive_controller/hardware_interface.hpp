#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace robot_hardware_interface
{
  double data_[2];

  class MyRobot : public hardware_interface::RobotHW
  {
  public:
    MyRobot(ros::NodeHandle& nh);
    ~MyRobot();
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}
    void read(){
      //ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    }

    void write(ros::Time t);
    void getOdometryFromArduino(const geometry_msgs::Twist::ConstPtr& twist_msg);


  private:
    const double WHEEL_SEPARATION = 0.256;

    ros::NodeHandle nh_;
    

    ros::Publisher right_vel_cmd_pub_;
    ros::Publisher left_vel_cmd_pub_;
    ros::Subscriber sub_;

    

    std_msgs::Float64 rightWheelVelocity_cmd;
    std_msgs::Float64 leftWheelVelocity_cmd;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    double lastTime;
    double odometryLastTime;
    //ros::Time current_time;

    int publishData;

    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];

    double x_;
    double y_;
    double th_;

    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
  };
}
#endif