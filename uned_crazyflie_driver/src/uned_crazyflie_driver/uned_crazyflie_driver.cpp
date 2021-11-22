#include <uned_crazyflie_driver/CrazyflieDriver.h>

bool CrazyflieDriverSim::initialize()
{
	ROS_INFO("CrazyflieDriverSim::inicialize() ok.");

  // Publisher:
  m_pub_cmdcontrol = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);
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
    // TO-DO. Insert Attitude & Rate Controller
  }
	return true;
}

void CrazyflieDriverSim::onboardCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  thrust = msg->data[0];
  roll = msg->data[1];
  pitch = msg->data[2];
  yaw = msg->data[3];
  onboard = true;
  ROS_INFO("Onboard Control: Thrust: %0.2f \tRoll: %0.2f \tPitch: %0.2f \tYaw: %0.2f", thrust, roll, pitch, yaw);
}

void CrazyflieDriverSim::cmdcontrolCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  m_cmd_motors.data = msg->data;
  motors = true;
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
