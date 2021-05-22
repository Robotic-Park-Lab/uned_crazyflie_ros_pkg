#include <uned_crazyflie_controllers/CrazyflieRateMixerController.h>

bool CrazyflieRateMixerController::initialize()
{
	ROS_INFO("CrazyflieRateMixerController::inicialize() ok.");

	// Publisher:
	// Actuators
	m_pub_motor_velocity_reference = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);

	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyflieRateMixerController::gtposeCallback, this);
	// Reference
	m_sub_ratemixer_ref = m_nh.subscribe( "ratemixer_controller_ref", 10, &CrazyflieRateMixerController::rateMixerRefsCallback, this);

	return true;
}

bool CrazyflieRateMixerController::iterate()
{
	// Housekeeping -------
	// TO-DO:
  ROS_INFO_THROTTLE(1, "In progress ...");
	ROS_INFO_THROTTLE(0.1, "Omega: %f \tDPitch: %f \tDRoll: %f \tDYaw; %f", omega, dpitch_ref, droll_ref, dyaw_ref);

	// Control Mixer
	{
		Eigen::Vector4d ref_rotor_velocities;
		ref_rotor_velocities[0] = omega;
		ref_rotor_velocities[1] = omega;
		ref_rotor_velocities[2] = omega;
		ref_rotor_velocities[3] = omega;

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
	omega = msg->omega;
	dpitch_ref = msg->dpitch;
	droll_ref = msg->droll;
	dyaw_ref = msg->dyaw;
}
