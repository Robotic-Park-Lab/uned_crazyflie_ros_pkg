
#include <uned_crazyflie_controllers/CrazyflieController.h>

bool CrazyflieController::initialize()
{
	ROS_INFO("CrazyflieController::inicialize() ok.");

	m_nh_params.getParam("CONTROLLER_TYPE", m_controller_type);
	m_nh_params.getParam("ROBOT_ID", m_robot_id);
	m_nh_params.getParam("CONTROLLER_MODE", m_controller_mode);

	// Publisher: Controller Status
	m_pub_control_signal = m_nh.advertise<std_msgs::Float64>("crazyflie_control_signal", 10);
	// Actuators
	m_pub_motor_velocity_reference = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);

	// Subscriber:
	m_sub_eje_x = m_nh.subscribe( "joystick_eje_x", 10, &CrazyflieController::ejexCallback, this);
	m_sub_eje_y = m_nh.subscribe( "joystick_eje_y", 10, &CrazyflieController::ejeyCallback, this);
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyflieController::gtposeCallback, this);
	// Reference

	return true;
}

bool CrazyflieController::iterate()
{
	// Housekeeping -------
	// TO-DO:
    ROS_INFO_THROTTLE(1, "In progress ...");
	ROS_WARN_STREAM_THROTTLE(1, "GT Pose:\n" << m_GT_pose);


	Eigen::Vector4d ref_rotor_velocities;
	if(step>200)
		step = 0;
	speed = 2250+(-100+step);
	step = step+1;
	ref_rotor_velocities[0] = speed;
	ref_rotor_velocities[1] = speed;
	ref_rotor_velocities[2] = speed;
	ref_rotor_velocities[3] = speed;

	rotorvelocitiesCallback(ref_rotor_velocities);
	return true;
}

void CrazyflieController::rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities){
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

void CrazyflieController::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_x = msg->data;
}

void CrazyflieController::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_y = msg->data;
}

void CrazyflieController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}

void CrazyflieController::readTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg)
{

}
