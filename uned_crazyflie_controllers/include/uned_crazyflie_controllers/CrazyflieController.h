#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

class CrazyflieController
{
   public:
	CrazyflieController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_control_signal;

	ros::Subscriber m_sub_eje_x, m_sub_eje_y;

	bool initialize();

	/** called when work is to be done */
	bool iterate();

   protected:
	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);

    double m_joy_x{.0}, m_joy_y{.0};

};
