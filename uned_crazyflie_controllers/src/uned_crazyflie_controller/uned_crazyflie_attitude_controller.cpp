#include <uned_crazyflie_controllers/CrazyfliePositionController.h>

bool CrazyfliePositionController::initialize()
{
	ROS_INFO("CrazyfliePositionController::inicialize() ok.");

	// Publisher:
	// Actuators
	m_pub_motor_velocity_reference = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed_attitude", 10);

	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyfliePositionController::gtposeCallback, this);
	// Reference

	return true;
}

bool CrazyfliePositionController::iterate()
{
	// Housekeeping -------
	// TO-DO:
  ROS_INFO_THROTTLE(1, "In progress ...");
	m_ref_pose.position.x = 0;
	m_ref_pose.position.y = 0;
	m_ref_pose.position.z = 1;
	m_ref_pose.orientation.x = 0;
	m_ref_pose.orientation.y = 0;
	m_ref_pose.orientation.z = 0;
	m_ref_pose.orientation.w = 1;

	ROS_INFO_STREAM_THROTTLE(1, "GT Pose:\n" << m_GT_pose);
	ROS_INFO_STREAM_THROTTLE(1, "REF Pose:\n" << m_ref_pose);

	return true;
}

void CrazyfliePositionController::rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities){
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


void CrazyfliePositionController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}
