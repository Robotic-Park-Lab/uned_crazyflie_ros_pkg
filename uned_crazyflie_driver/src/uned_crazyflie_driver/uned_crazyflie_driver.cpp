#include <uned_crazyflie_driver/CrazyflieDriver.h>

bool CrazyflieDriverSim::initialize()
{
	ROS_INFO("CrazyflieDriverSim::inicialize() ok.");

  // Publisher:
  m_pub_cmdcontrol = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);
	m_pub_control_signal = m_nh.advertise<uned_crazyflie_controllers::AttitudeRefs>("attitude_controller_ref", 10);
	m_pub_dyaw = m_nh.advertise<std_msgs::Float64>("dyaw_controller_ref", 10);
	m_pub_omega = m_nh.advertise<std_msgs::Float64>("omega_signal", 10);

	// Subscriber:
	// Reference
	m_sub_onboard = m_nh.subscribe( "onboard_cmd", 10, &CrazyflieDriverSim::onboardCallback, this);

	m_sub_cmd_motors = m_nh.subscribe( "cmd_control", 10, &CrazyflieDriverSim::cmdcontrolCallback, this);

  return true;
}

bool CrazyflieDriverSim::iterate()
{
  if(motors){
    ref_rotor_velocities[0] = m_cmd_motors.data[0];
    ref_rotor_velocities[1] = m_cmd_motors.data[1];
    ref_rotor_velocities[2] = m_cmd_motors.data[2];
    ref_rotor_velocities[3] = m_cmd_motors.data[3];
    rotorvelocitiesCallback(ref_rotor_velocities);
  }
	if(onboard){
		std_msgs::Float64 msg_omega;
		msg_omega.data = thrust;
		m_pub_omega.publish(msg_omega);
		std_msgs::Float64 msg_dyaw;
		msg_dyaw.data = 0.0;
		m_pub_dyaw.publish(msg_dyaw);
	}
	return true;
}

void CrazyflieDriverSim::onboardCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  thrust = msg->data[0];
  roll = msg->data[1];
  pitch = msg->data[2];
  yaw = msg->data[3];
  onboard = true;
	motors = false;
	attitudeRateMixerRefsCallback(thrust, pitch, roll, yaw);
  ROS_INFO("Onboard Control: Thrust: %0.2f \tRoll: %0.2f \tPitch: %0.2f \tYaw: %0.2f", thrust, roll, pitch, yaw);
}

void CrazyflieDriverSim::cmdcontrolCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  m_cmd_motors.data = msg->data;
  motors = true;
	onboard = false;
  ROS_INFO("Motors Control: %0.2f \t%0.2f \t%0.2f \t%0.2f", m_cmd_motors.data[0], m_cmd_motors.data[1], m_cmd_motors.data[2], m_cmd_motors.data[3]);
}

void CrazyflieDriverSim::rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities){
	// A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	// The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	actuator_msg->angular_velocities.clear();
	// for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	for (int i = 0; i < 4; i++)
	   actuator_msg->angular_velocities.push_back(rotor_velocities[i]);
	actuator_msg->header.stamp = ros::Time::now();

	m_pub_cmdcontrol.publish(actuator_msg);
}

void CrazyflieDriverSim::attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw)
{
	uned_crazyflie_controllers::AttitudeRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.pitch = pitch;
	ref_msg.roll = roll;

	m_pub_control_signal.publish(ref_msg);
}
