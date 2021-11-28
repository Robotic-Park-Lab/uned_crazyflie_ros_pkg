#pragma once

#include <ros/ros.h>
#include <array>
#include <cstring>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <mav_msgs/default_topics.h>
#include <time.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_msgs/Actuators.h>
#include <uned_crazyflie_controllers/AttitudeRefs.h>
#include <uned_crazyflie_controllers/RateMixerRefs.h>

struct pid_s{
   double kp, ki, kd, td;
   int nd;
   double error[2], integral, derivative[2], upperlimit, lowerlimit;
};
struct euler_angles{
   double roll, pitch, yaw;
};

class CrazyflieAttitudeController
{
   public:
	CrazyflieAttitudeController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_control_signal;

	ros::Subscriber m_sub_GT_pose, m_sub_attitude_ref;

	bool initialize();

	bool iterate();

   protected:

    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void attitudeRefsCallback(const uned_crazyflie_controllers::AttitudeRefs::ConstPtr& msg);
    void rateMixerRefsCallback(const double dpitch, const double droll, const double dyaw);
    euler_angles quaternion2euler(geometry_msgs::Quaternion quat);
    double pid_controller(struct pid_s controller, double dt);
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);

    // Controllers
    struct pid_s pitch_controller, roll_controller, yaw_controller;
    // Control Signals
    double pitch_ref, roll_ref, yaw_ref;
    double dpitch_ref, droll_ref, dyaw_ref;
    // Angles
    struct euler_angles rpy_ref, rpy_state;
    std::string m_controller_type, m_robot_id, m_controller_mode, str_id;
    geometry_msgs::Pose GT_pose;
    bool first_pose_received = false;
    bool first_ref_received = false;
    double dt = 0.002;
    const double PI = 3.14159265;
    double kp, ki, kd, td;

    double pitch_dron, pitch_error[3], dpitch[2], Phi_q[3];

    double roll_dron, roll_error[3], droll[2], Theta_q[3];

    double Yaw_q[3], yaw_error_signal[3], dyaw[2];

};
