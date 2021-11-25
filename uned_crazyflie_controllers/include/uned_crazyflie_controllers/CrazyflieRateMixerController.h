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
#include <uned_crazyflie_controllers/RateMixerRefs.h>

class CrazyflieRateMixerController
{
   public:
	CrazyflieRateMixerController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_motor_velocity_reference;

	ros::Subscriber m_sub_GT_pose, m_sub_ratemixer_ref, m_sub_omega;

	bool initialize();

	bool iterate();

   protected:
     struct pid_s{
        double kp, ki, kd, td;
        int nd;
        double error[2], integral, derivative[2], upperlimit, lowerlimit;
     };
     struct euler_angles{
        double roll, pitch, yaw;
     };
     void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
     void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
     void rateMixerRefsCallback(const uned_crazyflie_controllers::RateMixerRefs::ConstPtr& msg);
     void omegaCallback(const std_msgs::Float64::ConstPtr& msg);

     std::string m_controller_type, m_robot_id, m_controller_mode, str_id;
     geometry_msgs::Pose m_GT_pose;
     Eigen::Vector4d ref_rotor_velocities;
     bool first_pose_received = false;
     bool first_ref_received = false;
     double dt = 0.002;
     double kp, ki, kd, td;
     struct euler_angles rpy_state;
     // Controllers
     struct pid_s dpitch_controller, droll_controller, dyaw_controller;

     double omega, dpitch_ref, droll_ref, dyaw_ref, dpitch_dron, droll_dron, dyaw_dron;
     const double PI = 3.14159265;
     double fm = 0.2685;
     double C = (4070.3*fm)*PI/30;
     double pitch_dron[2], roll_dron[2], yaw_dron[2];

     double delta_pitch, delta_roll, delta_yaw;

     double droll_error[3], droll[2], Dtheta_q[3];

     double dyaw_error[3], dyaw[2], Dpsi_q[3];

     euler_angles quaternion2euler(geometry_msgs::Quaternion quat){
         euler_angles rpy;

         // roll (x-axis rotation)
         double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
         double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
         rpy.roll = std::atan2(sinr_cosp, cosr_cosp) * (180 / 3.14159265);

         // pitch (y-axis rotation)
         double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
         if (std::abs(sinp) >= 1)
             rpy.pitch = std::copysign(3.14159265 / 2, sinp) * (180 / 3.14159265);
         else
             rpy.pitch = std::asin(sinp) * (180 / 3.14159265);

         // yaw (z-axis rotation)
         double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
         double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
         rpy.yaw = std::atan2(siny_cosp, cosy_cosp) * (180 / 3.14159265);

         return rpy;
     }
     double pid_controller(struct pid_s controller, double dt){
         double outP = controller.kp * controller.error[0];
         controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
         controller.derivative[0] = (controller.td/(controller.td+controller.nd+dt))*controller.derivative[1]+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
         double out = outP + controller.integral + controller.derivative[0];

         double out_i = out;

         if (out > controller.upperlimit)
             out = controller.upperlimit;
         if (out < controller.lowerlimit)
             out = controller.lowerlimit;

         controller.integral = controller.integral - (out - out_i) * sqrt(controller.kp / controller.ki);

         controller.error[1] = controller.error[0];
         controller.derivative[1] = controller.derivative[0];

         return out;
     }
     struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit){
         struct pid_s controller;

         controller.kp = kp;
         controller.ki = ki;
         controller.kd = kd;
         controller.td = td;
         controller.nd = nd;
         controller.error[0] = 0.0;
         controller.error[1] = 0.0;
         controller.integral = 0.0;
         controller.derivative[0] = 0.0;
         controller.derivative[1] = 0.0;
         controller.upperlimit = upperlimit;
         controller.lowerlimit = lowerlimit;

         ROS_INFO("%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
         return controller;
     }
};
