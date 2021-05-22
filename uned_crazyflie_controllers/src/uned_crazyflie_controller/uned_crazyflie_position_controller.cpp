
#include <uned_crazyflie_controllers/CrazyfliePositionController.h>

bool CrazyfliePositionController::initialize()
{
	ROS_INFO("CrazyflieController::inicialize() ok.");

	m_nh_params.getParam("CONTROLLER_TYPE", m_controller_type);
	m_nh_params.getParam("ROBOT_ID", m_robot_id);
	m_nh_params.getParam("CONTROLLER_MODE", m_controller_mode);

	// Publisher: Controller Status
	m_pub_control_signal = m_nh.advertise<std_msgs::Float64>("crazyflie_control_signal", 10);
	// Actuators
	m_pub_motor_velocity_reference = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);
	// Referencias para los controladores PID Attitude y Rate
	//m_pub_attitude_rate_references = m_nh.advertise<Eigen::Vector4d>("attitude_rate_references", 10);

	// Subscriber:
	m_sub_eje_x = m_nh.subscribe( "joystick_eje_x", 10, &CrazyfliePositionController::ejexCallback, this);
	m_sub_eje_y = m_nh.subscribe( "joystick_eje_y", 10, &CrazyfliePositionController::ejeyCallback, this);
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyfliePositionController::gtposeCallback, this);
	// Reference:
	m_sub_pos_ref = m_nh.subscribe( "position_reference", 10, &CrazyfliePositionController::positionreferenceCallback, this);

	m_ref_pose.position.x = 0;
	m_ref_pose.position.y = 0;
	m_ref_pose.position.z = 1;
	m_ref_pose.orientation.x = 0;
	m_ref_pose.orientation.y = 0;
	m_ref_pose.orientation.z = 0;
	m_ref_pose.orientation.w = 1;
	m_ref_position = m_ref_pose;

	return true;
}

bool CrazyfliePositionController::iterate()
{
	// Housekeeping -------
	// TO-DO:
	ROS_INFO_THROTTLE(10,"In progress ...");

	ROS_INFO_STREAM_THROTTLE(1, "GT Pose:\n" << m_GT_pose);
	ROS_INFO_STREAM_THROTTLE(1, "REF Pose:\n" << m_ref_position);
	m_error_pose.position.x = m_ref_pose.position.x - m_GT_pose.position.x;
	m_error_pose.position.y = m_ref_pose.position.y - m_GT_pose.position.y;
	m_error_pose.position.z = m_ref_pose.position.z - m_GT_pose.position.z;

	ROS_INFO_STREAM_THROTTLE(1,"Error Pose:\n" << m_error_pose);

	// Calculo señal de control
	error_signal[2] = error_signal[1];
	error_signal[1] = error_signal[0];
	error_signal[0] = m_error_pose.position.z;
	control_signal[1] = control_signal[0];
	control_signal[0] = control_signal[1]+q[0]*error_signal[0]+q[1]*error_signal[1]+q[2]*error_signal[2];

	if(control_signal[0]>3500)
		control_signal[0] = 3500;
	if(control_signal[0] < 0)
		control_signal[0] = 0;

	control_signal[0] = 0;
	//ROS_INFO_THROTTLE(0.1,"Control signal: %f", control_signal[0]);
	//ROS_INFO_THROTTLE(0.1,"Error signal: %f", error_signal[0]);

	Eigen::Vector4d ref_rotor_velocities;
	ref_rotor_velocities[0] = control_signal[0];
	ref_rotor_velocities[1] = control_signal[0];
	ref_rotor_velocities[2] = control_signal[0];
	ref_rotor_velocities[3] = control_signal[0];

	rotorvelocitiesCallback(ref_rotor_velocities);
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

void CrazyfliePositionController::positionreferenceCallback(const geometry_msgs::Pose::ConstPtr& msg){
	m_ref_position.position = msg->position;
	m_ref_position.orientation = msg->orientation;
}

void CrazyfliePositionController::ejexCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_x = msg->data;
}

void CrazyfliePositionController::ejeyCallback(const std_msgs::Float64::ConstPtr& msg)
{
	m_joy_y = msg->data;
}

void CrazyfliePositionController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}

void CrazyfliePositionController::readTrajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory_reference_msg)
{

}
