#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <differential_drive_controller/hardware_interface.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;

  	robot_hardware_interface::MyRobot robot(nh);
  	controller_manager::ControllerManager cm(&robot);

  	ros::Rate rate(1.0 / robot.getPeriod().toSec());
  	ros::AsyncSpinner spinner(1);
  	spinner.start();

  while (ros::ok())
  {
     robot.read();
     cm.update(robot.getTime(), robot.getPeriod());
     robot.write(robot.getTime());
     rate.sleep();
  }

  spinner.stop();

  return 0;
}