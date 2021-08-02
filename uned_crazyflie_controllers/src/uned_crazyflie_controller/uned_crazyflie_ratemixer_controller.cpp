#include <uned_crazyflie_controllers/CrazyflieRateMixerController.h>

bool CrazyflieRateMixerController::initialize()
{
	ROS_INFO("CrazyflieRateMixerController::inicialize() ok.");

	if(m_nh_params.hasParam("Dphiq1") && m_nh_params.hasParam("Dphiq2") && m_nh_params.hasParam("Dphiq3")){
		m_nh_params.getParam("Dphiq1", Dphi_q[0]);
		m_nh_params.getParam("Dphiq2", Dphi_q[1]);
		m_nh_params.getParam("Dphiq3", Dphi_q[2]);
	}
	if(m_nh_params.hasParam("Dthetaq1") && m_nh_params.hasParam("Dthetaq2") && m_nh_params.hasParam("Dthetaq3")){
		m_nh_params.getParam("Dthetaq1", Dtheta_q[0]);
		m_nh_params.getParam("Dthetaq2", Dtheta_q[1]);
		m_nh_params.getParam("Dthetaq3", Dtheta_q[2]);
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

	m_sub_dyaw = m_nh.subscribe( "dyaw_controller_ref", 10, &CrazyflieRateMixerController::dyawCallback, this);

	return true;
}

bool CrazyflieRateMixerController::iterate()
{
	// DPitch Controller
	pitch_dron[1] = pitch_dron[0];
	double sinp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.y+m_GT_pose.orientation.y*m_GT_pose.orientation.z);
	if(std::abs(sinp) >= 1)
		pitch_dron[0] = std::copysign(M_PI/2, sinp)*180/PI;
	else
		pitch_dron[0] = std::asin(sinp)*180/PI;
	dpitch_dron = (pitch_dron[0]-pitch_dron[1])/0.002;
	{
		// Update error vector
		dpitch_error[2] = dpitch_error[1];
		dpitch_error[1] = dpitch_error[0];
		dpitch_error[0] = dpitch_ref - dpitch_dron;

		// Update signal vector
		dpitch[1] = dpitch[0];
		dpitch[0] = dpitch[1] + Dphi_q[0]*dpitch_error[0] + Dphi_q[1]*dpitch_error[1] + Dphi_q[2]*dpitch_error[2];
	}

	// DRoll Controller
	roll_dron[1] = roll_dron[0];
	double sinr_cosp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.x+m_GT_pose.orientation.y*m_GT_pose.orientation.z);
	double cosr_cosp = 1 - 2 * (m_GT_pose.orientation.x*m_GT_pose.orientation.x+m_GT_pose.orientation.y*m_GT_pose.orientation.y);
	roll_dron[0] = std::atan2(sinr_cosp,cosr_cosp)*180/PI;
	droll_dron = (roll_dron[0]-roll_dron[1])/0.002;
	{
		// Update error vector
		droll_error[2] = droll_error[1];
		droll_error[1] = droll_error[0];
		droll_error[0] = droll_ref - droll_dron;

		// Update signal vector
		droll[1] = droll[0];
		droll[0] = droll[1] + Dtheta_q[0]*droll_error[0] + Dtheta_q[1]*droll_error[1] + Dtheta_q[2]*droll_error[2];
	}
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
		ref_rotor_velocities[0] = (omega - 0.5*dpitch[0] - 0.5*droll[0] - dyaw[0])*0.056+C;
		ref_rotor_velocities[1] = (omega + 0.5*dpitch[0] - 0.5*droll[0] + dyaw[0])*0.056+C;
		ref_rotor_velocities[2] = (omega + 0.5*dpitch[0] + 0.5*droll[0] - dyaw[0])*0.056+C;
		ref_rotor_velocities[3] = (omega - 0.5*dpitch[0] + 0.5*droll[0] + dyaw[0])*0.056+C;

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


void CrazyflieRateMixerController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}

void CrazyflieRateMixerController::rateMixerRefsCallback(const uned_crazyflie_controllers::RateMixerRefs::ConstPtr& msg)
{
	dpitch_ref = msg->dpitch;
	droll_ref = msg->droll;
}

void CrazyflieRateMixerController::omegaCallback(const std_msgs::Float64::ConstPtr& msg)
{
	omega = msg->data;
}

void CrazyflieRateMixerController::dyawCallback(const std_msgs::Float64::ConstPtr& msg)
{
	dyaw_ref = msg->data;
}
