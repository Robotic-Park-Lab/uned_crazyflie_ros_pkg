#pragma once

#include <ros/ros.h>
#include <array>
#include <cstring>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <mav_msgs/default_topics.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <mav_msgs/Actuators.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <uned_crazyflie_controllers/AttitudeRateMixerRefs.h>

class CrazyfliePositionController
{
   public:
	CrazyfliePositionController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_motor_velocity_reference, m_pub_control_signal;

	ros::Subscriber m_sub_eje_x, m_sub_eje_y, m_sub_GT_pose, m_sub_pos_ref;

	bool initialize();

	/** called when work is to be done */
	bool iterate();

   protected:
	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw);
    void positionreferenceCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void readTrajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory_reference_msg);

    double m_joy_x{.0}, m_joy_y{.0}, speed;
    std::string m_controller_type, m_robot_id, m_controller_mode;
    geometry_msgs::Pose m_GT_pose, m_ref_pose, m_error_pose, m_ref_position;
    double w_equilibrio = 2000.0;
    double control_signal[2] = {w_equilibrio, w_equilibrio};
    double Kp{1000.0}, Ti{31.14}, Td{0.0}, Tp{0.01};
    double q[3] = {Kp*(1+(Tp/Ti)+(Td/Tp)), Kp*(-1+(Tp/Ti)+(Td/Tp)), Kp*(Td/Tp)};
    double error_signal[3] = {0.0, 0.0, 0.0};
    // Altitude paremeters
    double Z_T = 1/100;
    double z_error_signal[3] = {0.0, 0.0, 0.0};
    double Z_s[3] = {15000.0, 3500.0, 9000.0};
    //double Z_q[3] = {Z_s[0]+Z_s[1]*Z_T/2.0+Z_s[2]/Z_T, -Z_s[0]+Z_s[1]*Z_T/2.0-2.0*Z_s[2]/Z_T, Z_s[2]/Z_T};
    double Z_q[3] = {915017.5, -1814982.5, 900000};
    double delta_omega[3] = {0.0, 0.0, 0.0};
    double omega = 0.0;
    double we = 14480.0;

};
