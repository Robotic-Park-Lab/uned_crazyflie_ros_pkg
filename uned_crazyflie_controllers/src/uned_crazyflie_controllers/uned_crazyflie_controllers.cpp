#include <array>
#include <cstring>
#include <iostream>
#include <ros/console.h>
#include <uned_crazyflie_controllers/CrazyflieController.h>

bool CrazyflieController::initialize()
{
	ROS_INFO("CrazyflieController::inicialize() ok.");

	// m_nh_params.getParam("EXAMPLE", m_serial_example);

	// Publisher: Controller Status
	m_pub_control_signal =
		m_nh.advertise<std_msgs::Float64>(
			"crazyflie_control_signal", 10);

	// Subscriber:
	m_sub_eje_x = m_nh.subscribe(
		"joystick_eje_x", 10, &CrazyflieController::ejexCallback, this);
	m_sub_eje_y = m_nh.subscribe(
		"joystick_eje_y", 10, &CrazyflieController::ejeyCallback, this);

	return true;
}

bool CrazyflieController::iterate()
{
	// Housekeeping -------
	// TO-DO:
    ROS_INFO_THROTTLE(1, "In progress ...");

	return true;
}

void CrazyflieController::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_x = msg->data;
}

void CrazyflieController::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_y = msg->data;
}
