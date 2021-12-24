#include <uned_crazyflie_controllers/CrazyflieRateMixerController.h>

bool CrazyflieRateMixerController::initialize(){
	ROS_INFO("CrazyflieRateMixerController::inicialize() ok.");

	// dPitch Controller
	if(m_nh_params.hasParam("dPitchKp") && m_nh_params.hasParam("dPitchKi") && m_nh_params.hasParam("dPitchKd")){
			m_nh_params.getParam("dPitchKp", kp);
			m_nh_params.getParam("dPitchKi", ki);
			m_nh_params.getParam("dPitchKd", kd);
			dpitch_zcontroller = init_zcontroller("dPitch", kp, ki, kd, Ts, 100, 0.0, 0.0);
	}else{
			dpitch_zcontroller = init_zcontroller("dPitch", 250, 500, 2.5, Ts, 100, 0.0, 0.0);
	}
	// dRoll Controller
	if(m_nh_params.hasParam("dRollKp") && m_nh_params.hasParam("dRollKi") && m_nh_params.hasParam("dRollKd")){
			m_nh_params.getParam("dRollKp", kp);
			m_nh_params.getParam("dRollKi", ki);
			m_nh_params.getParam("dRollKd", kd);
			droll_zcontroller = init_zcontroller("dRoll", kp, ki, kd, Ts, 100, 0.0, -0.0);
	}else{
			droll_zcontroller = init_zcontroller("dRoll", 250.0, 500.0, 2.5, Ts, 100, 0.0, -0.0);
	}
	// dYaw Controller
	if(m_nh_params.hasParam("dYawKp") && m_nh_params.hasParam("dYawKi") && m_nh_params.hasParam("dYawKd")){
			m_nh_params.getParam("dYawKp", kp);
			m_nh_params.getParam("dYawKi", ki);
			m_nh_params.getParam("dYawKd", kd);
			dyaw_zcontroller = init_zcontroller("dYaw", kp, ki, kd, Ts, 100, 0.0, -0.0);
	}else{
			dyaw_zcontroller = init_zcontroller("dYaw", 120.0, 16.7, 0.0, Ts, 100, 0.0, -0.0);
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

bool CrazyflieRateMixerController::iterate(){
	// Feedback:
	rpy_state = quaternion2euler(m_GT_pose.orientation);
	// dPitch controller
	pitch_dron[1] = pitch_dron[0];
	pitch_dron[0] = rpy_state.pitch;
	dpitch_dron = (pitch_dron[0] - pitch_dron[1]) / Ts;
	dpitch_zcontroller.error[0] = dpitch_ref - dpitch_dron;
	delta_pitch = pid_zcontroller(dpitch_zcontroller);

	// DRoll Controller
	roll_dron[1] = roll_dron[0];
	roll_dron[0] = rpy_state.roll;
	droll_dron = (roll_dron[0]-roll_dron[1])/Ts;
	droll_zcontroller.error[0] = droll_ref - droll_dron;
	delta_roll = pid_zcontroller(droll_zcontroller);

	// DYaw Controller
	yaw_dron[1] = yaw_dron[0];
	yaw_dron[0] = rpy_state.yaw;
	dyaw_dron = (yaw_dron[0]-yaw_dron[1])/Ts;
	dyaw_zcontroller.error[0] = dyaw_ref - dyaw_dron;
	delta_yaw = pid_zcontroller(dyaw_zcontroller);

	// Control Mixer
	{
		ref_rotor_velocities[0] = (omega - 0.5*delta_pitch - 0.5*delta_roll - delta_yaw)*fm;
		ref_rotor_velocities[1] = (omega + 0.5*delta_pitch - 0.5*delta_roll + delta_yaw)*fm;
		ref_rotor_velocities[2] = (omega + 0.5*delta_pitch + 0.5*delta_roll - delta_yaw)*fm;
		ref_rotor_velocities[3] = (omega - 0.5*delta_pitch + 0.5*delta_roll + delta_yaw)*fm;

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
	rpy.roll = std::atan2(sinr_cosp, cosr_cosp) * (180 / PI);

	// pitch (y-axis rotation)
	double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
	if (std::abs(sinp) >= 1)
	rpy.pitch = std::copysign(PI / 2, sinp) * (180 / PI);
	else
	rpy.pitch = std::asin(sinp) * (180 / PI);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
	double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
	rpy.yaw = std::atan2(siny_cosp, cosy_cosp) * (180 / PI);

	return rpy;
}

double CrazyflieRateMixerController::pid_zcontroller(struct pid_z &controller){
	double out = controller.u + controller.q[0]*controller.error[0] + controller.q[1]*controller.error[1]+controller.q[2]*controller.error[2];

	if(controller.upperlimit != 0.0){
		// double out_i = out;
		if (out > controller.upperlimit)
			out = controller.upperlimit;
		if (out < controller.lowerlimit)
			out = controller.lowerlimit;
	}

	controller.error[2] = controller.error[1];
	controller.error[1] = controller.error[0];
	controller.u = out;

	return out;
}

struct pid_z CrazyflieRateMixerController::init_zcontroller(const char id[], double kp, double ki, double kd, double Ts, int nd, double upperlimit, double lowerlimit){
	struct pid_z controller;

	controller.q[0] = kp+ki*Ts/2+kd/Ts;
	controller.q[1] = -kp+ki*Ts/2-2*kd/Ts;
	controller.q[2] = kd/Ts;
	controller.u = 0;
	controller.Ts = Ts;
	controller.nd = nd;
	controller.error[0] = 0.0;
	controller.error[1] = 0.0;
	controller.error[2] = 0.0;
	controller.upperlimit = upperlimit;
	controller.lowerlimit = lowerlimit;

	ROS_INFO("%s Controller: q[0]: %0.2f \tq[1]: %0.2f \tq[2]: %0.2f", id, controller.q[0], controller.q[1], controller.q[2]);
	return controller;
}
