#include <differential_drive_controller/hardware_interface.hpp>

namespace robot_hardware_interface
{
	MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh)
	{
		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle_a("base_to_roue_gauche", &pos_[0], &vel_[0], &eff_[0]);
		jnt_state_interface.registerHandle(state_handle_a);

		hardware_interface::JointStateHandle state_handle_b("base_to_roue_droite", &pos_[1], &vel_[1], &eff_[1]);
		jnt_state_interface.registerHandle(state_handle_b);

		registerInterface(&jnt_state_interface);

		// connect and register the joint position interface
		hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("base_to_roue_gauche"), &cmd_[0]);
		jnt_pos_interface.registerHandle(pos_handle_a);
		jnt_vel_interface.registerHandle(pos_handle_a);


		hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("base_to_roue_droite"), &cmd_[1]);
		jnt_pos_interface.registerHandle(pos_handle_b);
		jnt_vel_interface.registerHandle(pos_handle_b);

		registerInterface(&jnt_pos_interface);
		registerInterface(&jnt_vel_interface);

		//On souscrit a l'odometrie des roues du ARDUINO
		sub_ = nh.subscribe("ard_odo", 1000, &MyRobot::getOdometryFromArduino, this);

		//On publie la vitesse désirée des roues au PID
		right_vel_cmd_pub_ = nh_.advertise<std_msgs::Float64>("cmd_vel_right", 1000);
    	left_vel_cmd_pub_ = nh_.advertise<std_msgs::Float64>("cmd_vel_left", 1000);

    	//On republie une ODOMETRIE utilisable pas ROS
		odom_pub = nh_.advertise<nav_msgs::Odometry>("test/odom", 50);
    	
    	odometryLastTime = 0;
	}

	MyRobot::~MyRobot(){

	}

	void MyRobot::write(ros::Time t){
		//Update pos_ and vel_
		//ROS_INFO_STREAM("Commands for joints: " << vel_[0] << ", " << vel_[1]);
		double timeinterval = t.toSec() - lastTime;
		double stepIntervalGauche = cmd_[0] * timeinterval;
		double stepIntervalDroite = cmd_[1] * timeinterval;
		vel_[0] = cmd_[0] * 0.03;
		vel_[1] = cmd_[1] * 0.03;
		//pos_[0] += stepIntervalGauche;
		//pos_[1] += stepIntervalDroite;

		rightWheelVelocity_cmd.data = cmd_[0];
		leftWheelVelocity_cmd.data = cmd_[1];

		right_vel_cmd_pub_.publish(rightWheelVelocity_cmd);
		left_vel_cmd_pub_.publish(leftWheelVelocity_cmd);

		lastTime = t.toSec();
	}

	void MyRobot::getOdometryFromArduino(const geometry_msgs::Twist::ConstPtr& twist_msg){
      //cout << "Speed data:" << twist_msg->linear.x << "\n";
      //ROS_INFO("I heard: [%s]", twist_msg->linear.x);

      //GET THE WHEEL SPEED FROM THE ENCODER
		double dL, dR, dth, dxy, v, w;
		ros::Time currentTime = ros::Time::now();
		//vx = twist_msg->linear.x;
		//vy = twist_msg->linear.y;

		double elaspedTime = currentTime.toSec() - odometryLastTime;

		odometryLastTime = currentTime.toSec();

		dL = twist_msg->linear.x * elaspedTime;
		dR = twist_msg->linear.y * elaspedTime;

		dxy = (dL + dR) / 2;
		dth = ((dR - dL) / WHEEL_SEPARATION); // * odomertyMultiplier

		x_ += dxy * cosf(th_);
		y_ += dxy * sinf(th_);
		th_ += dth;

		v = dxy/elaspedTime;
		w = dth/elaspedTime;

		publishData = 1;
		//publishOdometry(2);

		
		odom_quat = tf::createQuaternionMsgFromYaw(th_);
		
		//first, we'll publish the transform over tf
		odom_trans.header.stamp = currentTime;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x_;
	    odom_trans.transform.translation.y = y_;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    odom_broadcaster.sendTransform(odom_trans);

	    odom.header.stamp = currentTime;
	    odom.header.frame_id = "odom";

	    //set the position
	    odom.pose.pose.position.x = x_;
	    odom.pose.pose.position.y = y_;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    odom.child_frame_id = "base_link";
	    odom.twist.twist.linear.x = v;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;
	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = w;

	    odom_pub.publish(odom);
    }
    /*
    void publishOdometry(int d){
      ROS_INFO("I heard: [%d]", d);
    }*/
}