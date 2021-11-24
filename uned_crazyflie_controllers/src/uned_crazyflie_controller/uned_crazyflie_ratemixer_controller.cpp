#include <uned_crazyflie_controllers/CrazyflieRateMixerController.h>

bool CrazyflieRateMixerController::initialize()
{
	ROS_INFO("CrazyflieRateMixerController::inicialize() ok.");

	// dPitch Controller
	dpitch_controller = init_controller("dPitch", 250.0, 500.0, 2.5, 0.01, 100, 720.0, -720.0);
	// dRoll Controller
	droll_controller = init_controller("dRoll", 250.0, 500.0, 2.5, 0.01, 100, 720.0, -720.0);
	// dYaw Controller
	dyaw_controller = init_controller("dYaw", 120.0, 16.7, 0.0, 0.0, 100, 400.0, -400.0);

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
	if (first_ref_received && first_pose_received) {
		// Feedback:
		rpy_state = quaternion2euler(m_GT_pose.orientation);

		// Rate Controller
		// dPitch controller
		pitch_dron[1] = pitch_dron[0];
		pitch_dron[0] = rpy_state.pitch;
		dpitch_dron = (pitch_dron[0] - pitch_dron[1]) / dt;
		dpitch_controller.error[0] = dpitch_ref - dpitch_dron;
		delta_pitch = pid_controller(dpitch_controller, dt);
		// dRoll controller
		roll_dron[1] = roll_dron[0];
		roll_dron[0] = rpy_state.roll;
		droll_dron = (roll_dron[0] - roll_dron[1]) / dt;
		droll_controller.error[0] = droll_ref - droll_dron;
		delta_roll = pid_controller(droll_controller, dt);
		// dYaw controller
		yaw_dron[1] = yaw_dron[0];
		yaw_dron[0] = rpy_state.yaw;
		dyaw_dron = (yaw_dron[0] - yaw_dron[1]) / dt;
		dyaw_controller.error[0] = dyaw_ref - dyaw_dron;
		delta_yaw = pid_controller(dyaw_controller, dt);

		// Control Mixer
		ref_rotor_velocities[0] = ((omega*2 + 0.5*delta_pitch - 0.5*delta_roll + delta_yaw)+4070.0)*fm*PI/30;
		ref_rotor_velocities[1] = ((omega*2 - 0.5*delta_pitch - 0.5*delta_roll - delta_yaw)+4070.0)*fm*PI/30;
		ref_rotor_velocities[2] = ((omega*2 - 0.5*delta_pitch + 0.5*delta_roll + delta_yaw)+4070.0)*fm*PI/30;
		ref_rotor_velocities[3] = ((omega*2 + 0.5*delta_pitch + 0.5*delta_roll - delta_yaw)+4070.0)*fm*PI/30;

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
	if (!first_pose_received)
			first_pose_received = true;
}

void CrazyflieRateMixerController::rateMixerRefsCallback(const uned_crazyflie_controllers::RateMixerRefs::ConstPtr& msg)
{
	dpitch_ref = msg->dpitch;
	droll_ref = msg->droll;
	if (!first_ref_received)
			first_ref_received = true;
}

void CrazyflieRateMixerController::omegaCallback(const std_msgs::Float64::ConstPtr& msg)
{
	omega = msg->data;
}

void CrazyflieRateMixerController::dyawCallback(const std_msgs::Float64::ConstPtr& msg)
{
	dyaw_ref = msg->data;
}
