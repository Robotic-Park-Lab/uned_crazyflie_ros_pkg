#include <uned_crazyflie_controllers/CrazyflieRateMixerController.h>

bool CrazyflieRateMixerController::initialize()
{
	ROS_INFO("CrazyflieRateMixerController::inicialize() ok.");

	// dPitch Controller
	if(m_nh_params.hasParam("dPitchKp") && m_nh_params.hasParam("dPitchKi") && m_nh_params.hasParam("dPitchKd")){
			m_nh_params.getParam("dPitchKp", kp);
			m_nh_params.getParam("dPitchKi", ki);
			m_nh_params.getParam("dPitchKd", kd);
			m_nh_params.getParam("dPitchTd", td);
			dpitch_controller = init_controller("dPitch", kp, ki, kd, td, 100, 0.0, 0.0);
	}else{
			dpitch_controller = init_controller("dPitch", 250, 500, 2.5, 0.01, 100, 0.0, 0.0);
	}
	// dRoll Controller
	if(m_nh_params.hasParam("dRollKp") && m_nh_params.hasParam("dRollKi") && m_nh_params.hasParam("dRollKd")){
			m_nh_params.getParam("dRollKp", kp);
			m_nh_params.getParam("dRollKi", ki);
			m_nh_params.getParam("dRollKd", kd);
			m_nh_params.getParam("dRollTd", td);
			droll_controller = init_controller("dRoll", kp, ki, kd, td, 100, 7720.0, -7720.0);
	}else{
			droll_controller = init_controller("dRoll", 250.0, 500.0, 2.5, 0.01, 100, 7720.0, -7720.0);
	}
	if(m_nh_params.hasParam("Dpsiq1") && m_nh_params.hasParam("Dpsiq2") && m_nh_params.hasParam("Dpsiq3")){
		m_nh_params.getParam("Dpsiq1", Dpsi_q[0]);
		m_nh_params.getParam("Dpsiq2", Dpsi_q[1]);
		m_nh_params.getParam("Dpsiq3", Dpsi_q[2]);
	}
	// Publisher:
	// Actuators
	m_pub_motor_velocity_reference = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);
	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyflieRateMixerController::gtposeCallback, this);
	// Reference
	m_sub_ratemixer_ref = m_nh.subscribe( "ratemixer_controller_ref", 10, &CrazyflieRateMixerController::rateMixerRefsCallback, this);

	m_sub_omega = m_nh.subscribe( "omega_signal", 10, &CrazyflieRateMixerController::omegaCallback, this);

	return true;
}

bool CrazyflieRateMixerController::iterate()
{
	// Feedback:
		rpy_state = quaternion2euler(m_GT_pose.orientation);
	// dPitch controller
	pitch_dron[1] = pitch_dron[0];
	pitch_dron[0] = rpy_state.pitch;
	dpitch_dron = (pitch_dron[0] - pitch_dron[1]) / 0.002;
	dpitch_controller.error[0] = dpitch_ref - dpitch_dron;
	delta_pitch = pid_controller(dpitch_controller, 0.002);

	// DRoll Controller
	roll_dron[1] = roll_dron[0];
	roll_dron[0] = rpy_state.roll;
	droll_dron = (roll_dron[0]-roll_dron[1])/0.002;
	droll_controller.error[0] = droll_ref - droll_dron;
	delta_roll = pid_controller(droll_controller, 0.002);

	// DYaw Controller
	yaw_dron[1] = yaw_dron[0];
	double siny_cosp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.z+m_GT_pose.orientation.x*m_GT_pose.orientation.y);
	double cosy_cosp = 1 - 2 * (m_GT_pose.orientation.y*m_GT_pose.orientation.y + m_GT_pose.orientation.z*m_GT_pose.orientation.z);
	yaw_dron[0] = std::atan2(siny_cosp,cosy_cosp)*180/PI;
	dyaw_dron = (yaw_dron[0]-yaw_dron[1])/0.002;
	{
		// Update error vector
		dyaw_error[2] = dyaw_error[1];
		dyaw_error[1] = dyaw_error[0];
		dyaw_error[0] = dyaw_ref - dyaw_dron;

		// Update signal vector
		dyaw[1] = dyaw[0];
		dyaw[0] = dyaw[1] + Dpsi_q[0]*dyaw_error[0] + Dpsi_q[1]*dyaw_error[1] + Dpsi_q[2]*dyaw_error[2];
	}
	// Control Mixer
	{
		ref_rotor_velocities[0] = (omega - 0.5*delta_pitch - 0.5*delta_roll - dyaw[0])*fm;
		ref_rotor_velocities[1] = (omega + 0.5*delta_pitch - 0.5*delta_roll + dyaw[0])*fm;
		ref_rotor_velocities[2] = (omega + 0.5*delta_pitch + 0.5*delta_roll - dyaw[0])*fm;
		ref_rotor_velocities[3] = (omega - 0.5*delta_pitch + 0.5*delta_roll + dyaw[0])*fm;

		rotorvelocitiesCallback(ref_rotor_velocities);
	}

	return true;
}

void CrazyflieRateMixerController::rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities){
	// A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	// The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	actuator_msg->angular_velocities.clear();
	// for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	for (int i = 0; i < 4; i++)
	   actuator_msg->angular_velocities.push_back(rotor_velocities[i]);
	actuator_msg->header.stamp = ros::Time::now();

	m_pub_motor_velocity_reference.publish(actuator_msg);
}

void CrazyflieRateMixerController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg){
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}

void CrazyflieRateMixerController::rateMixerRefsCallback(const uned_crazyflie_controllers::RateMixerRefs::ConstPtr& msg){
	dpitch_ref = msg->dpitch;
	droll_ref = msg->droll;
	dyaw_ref = msg->dyaw;
}

void CrazyflieRateMixerController::omegaCallback(const std_msgs::Float64::ConstPtr& msg){
	omega = msg->data;
}

euler_angles CrazyflieRateMixerController::quaternion2euler(geometry_msgs::Quaternion quat){
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

double CrazyflieRateMixerController::pid_controller(struct pid_s controller, double dt){
	double outP = controller.kp * controller.error[0];
	controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
	controller.derivative[0] = (controller.td/(controller.td+controller.nd+dt))*controller.derivative[1]+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
	double out = outP + controller.integral + controller.derivative[0];

	if(controller.upperlimit != 0.0){
		double out_i = out;

		if (out > controller.upperlimit)
			out = controller.upperlimit;
		if (out < controller.lowerlimit)
			out = controller.lowerlimit;

		controller.integral = controller.integral - (out - out_i) * sqrt(controller.kp / controller.ki);
	}

	controller.error[1] = controller.error[0];
	controller.derivative[1] = controller.derivative[0];

	return out;
}

struct pid_s CrazyflieRateMixerController::init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit){
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
