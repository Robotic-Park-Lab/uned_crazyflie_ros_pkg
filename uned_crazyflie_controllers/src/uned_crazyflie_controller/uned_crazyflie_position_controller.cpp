
#include <uned_crazyflie_controllers/CrazyfliePositionController.h>

bool CrazyfliePositionController::initialize()
{
	ROS_INFO("CrazyfliePositionController::inicialize() ok.");

	m_nh_params.getParam("CONTROLLER_TYPE", m_controller_type);
	m_nh_params.getParam("ROBOT_ID", m_robot_id);
	m_nh_params.getParam("CONTROLLER_MODE", m_controller_mode);

	// Publisher:
	// Actuators
	m_pub_motor_velocity_reference = m_nh.advertise<mav_msgs::Actuators>("command/motor_speed", 10);
	// Referencias para los controladores PID Attitude y Rate
	m_pub_control_signal = m_nh.advertise<uned_crazyflie_controllers::AttitudeRateMixerRefs>("attitude_rate_references", 10);

	// Subscriber:
	m_sub_eje_x = m_nh.subscribe( "joystick_eje_x", 10, &CrazyfliePositionController::ejexCallback, this);
	m_sub_eje_y = m_nh.subscribe( "joystick_eje_y", 10, &CrazyfliePositionController::ejeyCallback, this);
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyfliePositionController::gtposeCallback, this);
	// Reference:
	m_sub_pos_ref = m_nh.subscribe( "position_reference", 10, &CrazyfliePositionController::positionreferenceCallback, this);

	m_ref_pose.position.x = 1;
	m_ref_pose.position.y = 1;
	m_ref_pose.position.z = 1;
	m_ref_pose.orientation.x = 0;
	m_ref_pose.orientation.y = 0;
	m_ref_pose.orientation.z = 0;
	m_ref_pose.orientation.w = 1;
	m_ref_position = m_ref_pose;

	ROS_INFO("In progress ...");
	ROS_INFO("PID(Z) Parameters: \t%f \t%f \t%f", Z_q[0], Z_q[1], Z_q[2]);

	return true;
}

bool CrazyfliePositionController::iterate()
{
	// Altitude Controller
	{
		// Update error vector
		z_error_signal[2] = z_error_signal[1];
		z_error_signal[1] = z_error_signal[0];
		z_error_signal[0] = m_ref_pose.position.z - m_GT_pose.position.z;

		// Update signal vector
		delta_omega[1] = delta_omega[0];
		delta_omega[0] = delta_omega[1] + Z_q[0]*z_error_signal[0] + Z_q[1]*z_error_signal[1] + Z_q[2]*z_error_signal[2];

		// Saturation
		if(delta_omega[0]>15000)
			delta_omega[0] = 15000;
		if(delta_omega[0]<-15000)
			delta_omega[0] = -15000;

		// Output signal
		omega = (delta_omega[0]+(we-4070.3)/0.2685)*0.0509;
		// ROS_INFO_THROTTLE(0.2, "Z_error: %f ; \tZ_deltaOmega: %f ; \tOmega: %f->%f", z_error_signal[0], delta_omega[0], omega/0.0509, omega);

	}
	// X-Y Controller
	{
		// Position
		// Convert quaternion to yw
		double siny_cosp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.z+m_GT_pose.orientation.x*m_GT_pose.orientation.y);
		double cosy_cosp = 1 - 2 * (m_GT_pose.orientation.y*m_GT_pose.orientation.y + m_GT_pose.orientation.z*m_GT_pose.orientation.z);
		double yaw = std::atan2(siny_cosp,cosy_cosp);

		// Update local error
		x_error_signal[2] = x_error_signal[1];
		x_error_signal[1] = x_error_signal[0];
		x_error_signal[0] = (m_ref_pose.position.x - m_GT_pose.position.x)*cos(yaw)+(m_ref_pose.position.y - m_GT_pose.position.y)*sin(yaw);
		y_error_signal[2] = y_error_signal[1];
		y_error_signal[1] = y_error_signal[0];
		y_error_signal[0] = -(m_ref_pose.position.x - m_GT_pose.position.x)*sin(yaw)+(m_ref_pose.position.y - m_GT_pose.position.y)*cos(yaw);

		// Update signal vector
		uc[1] = uc[0];
		uc[0] = uc[1] + X_q[0]*x_error_signal[0] + X_q[1]*x_error_signal[1] + X_q[2]*x_error_signal[2];
		vc[1] = vc[0];
		vc[0] = vc[1] + Y_q[0]*y_error_signal[0] + Y_q[1]*y_error_signal[1] + Y_q[2]*y_error_signal[2];

		// Speed
		u_feedback[1] = u_feedback[0];
		u_feedback[0] = m_GT_pose.position.x;
		double u_signal = (u_feedback[0]-u_feedback[1])/0.01;
		v_feedback[1] = v_feedback[0];
		v_feedback[0] = m_GT_pose.position.y;
		double v_signal = (v_feedback[0]-v_feedback[1])/0.01;

		// Update error
		u_error_signal[2] = u_error_signal[1];
		u_error_signal[1] = u_error_signal[0];
		u_error_signal[0] = uc[0]-u_signal;
		v_error_signal[2] = v_error_signal[1];
		v_error_signal[1] = v_error_signal[0];
		v_error_signal[0] = vc[0]-v_signal;

		// Update signal vector
		pitch_ref[1] = pitch_ref[0];
		pitch_ref[0] = pitch_ref[1] + U_q[0]*u_error_signal[0] + U_q[1]*u_error_signal[1] + U_q[2]*u_error_signal[2];
		roll_ref[1] = roll_ref[0];
		roll_ref[0] = roll_ref[1] + V_q[0]*v_error_signal[0] + V_q[1]*v_error_signal[1] + V_q[2]*v_error_signal[2];

		// Saturation
		if(pitch_ref[0]>30)
			pitch_ref[0] = 30;
		if(pitch_ref[0]<-30)
			pitch_ref[0] = -30;
		if(roll_ref[0]>30)
			roll_ref[0] = 30;
		if(roll_ref[0]<-30)
			roll_ref[0] = -30;
	}

	
	ROS_INFO_STREAM_THROTTLE(1, "GT Pose:\n" << m_GT_pose);
	ROS_INFO_STREAM_THROTTLE(1, "REF Pose:\n" << m_ref_position);
	/*
	m_error_pose.position.x = m_ref_pose.position.x - m_GT_pose.position.x;
	m_error_pose.position.y = m_ref_pose.position.y - m_GT_pose.position.y;

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

	//ROS_INFO_THROTTLE(0.1,"Control signal: %f", control_signal[0]);
	//ROS_INFO_THROTTLE(0.1,"Error signal: %f", error_signal[0]);

	Eigen::Vector4d ref_rotor_velocities;
	ref_rotor_velocities[0] = control_signal[0];
	ref_rotor_velocities[1] = control_signal[0];
	ref_rotor_velocities[2] = control_signal[0];
	ref_rotor_velocities[3] = control_signal[0];

	rotorvelocitiesCallback(ref_rotor_velocities);
	*/

	attitudeRateMixerRefsCallback(omega, pitch_ref[0], roll_ref[0], 0.0);
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

void CrazyfliePositionController::rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities)
{
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

void CrazyfliePositionController::attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw)
{
	uned_crazyflie_controllers::AttitudeRateMixerRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.omega = omega;
	ref_msg.pitch = pitch;
	ref_msg.roll = roll;
	ref_msg.dyaw = dyaw;

	m_pub_control_signal.publish(ref_msg);
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
