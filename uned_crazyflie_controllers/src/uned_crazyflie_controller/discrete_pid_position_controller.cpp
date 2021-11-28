
#include <uned_crazyflie_controllers/CrazyfliePositionController.h>

bool CrazyfliePositionController::initialize()
{
	ROS_INFO("CrazyfliePositionController::inicialize() ok.");

	// Lectura de parámetros
	m_nh_params.getParam("CONTROLLER_TYPE", m_controller_type);
	m_nh_params.getParam("ROBOT_ID", m_robot_id);
	m_nh_params.getParam("CONTROLLER_MODE", m_controller_mode);
	m_nh_params.getParam("X_POS", m_x_init);
	m_nh_params.getParam("Y_POS", m_y_init);
	m_nh_params.getParam("Z_POS", m_z_init);


	if(m_nh_params.hasParam("Zq1") && m_nh_params.hasParam("Zq2") && m_nh_params.hasParam("Zq3")){
		m_nh_params.getParam("Zq1", Z_q[0]);
		m_nh_params.getParam("Zq2", Z_q[1]);
		m_nh_params.getParam("Zq3", Z_q[2]);
	}
	if(m_nh_params.hasParam("Xq1") && m_nh_params.hasParam("Xq2") && m_nh_params.hasParam("Xq3")){
		m_nh_params.getParam("Xq1", X_q[0]);
		m_nh_params.getParam("Xq2", X_q[1]);
		m_nh_params.getParam("Xq3", X_q[2]);
	}
	if(m_nh_params.hasParam("Yq1") && m_nh_params.hasParam("Yq2") && m_nh_params.hasParam("Yq3")){
		m_nh_params.getParam("Yq1", Y_q[0]);
		m_nh_params.getParam("Yq2", Y_q[1]);
		m_nh_params.getParam("Yq3", Y_q[2]);
	}
	if(m_nh_params.hasParam("Uq1") && m_nh_params.hasParam("Uq2") && m_nh_params.hasParam("Uq3")){
		m_nh_params.getParam("Uq1", U_q[0]);
		m_nh_params.getParam("Uq2", U_q[1]);
		m_nh_params.getParam("Uq3", U_q[2]);
	}
	if(m_nh_params.hasParam("Vq1") && m_nh_params.hasParam("Vq2") && m_nh_params.hasParam("Vq3")){
		m_nh_params.getParam("Vq1", V_q[0]);
		m_nh_params.getParam("Vq2", V_q[1]);
		m_nh_params.getParam("Vq3", V_q[2]);
	}
	// Publisher:
	// Referencias para los controladores PID Attitude y Rate
	m_pub_control_signal = m_nh.advertise<uned_crazyflie_controllers::AttitudeRefs>("attitude_controller_ref", 10);

	m_pub_omega = m_nh.advertise<std_msgs::Float64>("omega_signal", 10);

	m_pub_dyaw = m_nh.advertise<std_msgs::Float64>("dyaw_controller_ref", 10);
	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyfliePositionController::gtposeCallback, this);
	// Reference:
	m_sub_pos_ref = m_nh.subscribe( "goal_pose", 10, &CrazyfliePositionController::positionreferenceCallback, this);

	u_feedback[0] = m_x_init;
	v_feedback[0] = m_y_init;

	ref_pose.position.x = m_x_init;
	ref_pose.position.y = m_y_init;
	ref_pose.position.z = m_z_init;
	ref_pose.orientation.x = 0;
	ref_pose.orientation.y = 0;
	ref_pose.orientation.z = 0;
	ref_pose.orientation.w = 1;

	ROS_INFO("In progress ...");
	ROS_INFO_STREAM("Goal Pose: " << ref_pose);
	ROS_INFO("Altitude PID(Z) Parameters: \t%f \t%f \t%f", Z_q[0], Z_q[1], Z_q[2]);
	ROS_INFO("X PID(Z) Parameters: \t%f \t%f \t%f", X_q[0], X_q[1], X_q[2]);
	ROS_INFO("U PID(Z) Parameters: \t%f \t%f \t%f", U_q[0], U_q[1], U_q[2]);
	ROS_INFO("Y PID(Z) Parameters: \t%f \t%f \t%f", Y_q[0], Y_q[1], Y_q[2]);
	ROS_INFO("V PID(Z) Parameters: \t%f \t%f \t%f", V_q[0], V_q[1], V_q[2]);

	return true;
}

bool CrazyfliePositionController::iterate()
{
	// Feedback:
	rpy_state = quaternion2euler(GT_pose.orientation);
	rpy_ref = quaternion2euler(ref_pose.orientation);
	// Altitude Controller
	{
		// Update error vector
		z_error_signal[2] = z_error_signal[1];
		z_error_signal[1] = z_error_signal[0];
		z_error_signal[0] = ref_pose.position.z - GT_pose.position.z;

		// Update signal vector
		delta_omega[1] = delta_omega[0];
		delta_omega[0] = delta_omega[1] + Z_q[0]*z_error_signal[0] + Z_q[1]*z_error_signal[1] + Z_q[2]*z_error_signal[2];

		// Saturation
		if(delta_omega[0]>15000)
			delta_omega[0] = 15000;
		if(delta_omega[0]<-15000)
			delta_omega[0] = -15000;

		// Output signal
		omega = delta_omega[0]+(we-4070.3)/0.2685;
	}

	yaw = rpy_state.yaw;
	{
		// Position
		// Update local error
		x_error_signal[2] = x_error_signal[1];
		x_error_signal[1] = x_error_signal[0];
		x_error_signal[0] = (ref_pose.position.x - GT_pose.position.x)*cos(yaw)+(ref_pose.position.y - GT_pose.position.y)*sin(yaw);
		y_error_signal[2] = y_error_signal[1];
		y_error_signal[1] = y_error_signal[0];
		y_error_signal[0] = -(ref_pose.position.x - GT_pose.position.x)*sin(yaw)+(ref_pose.position.y - GT_pose.position.y)*cos(yaw);

		// Update signal vector
		uc[1] = uc[0];
		uc[0] = uc[1] + X_q[0]*x_error_signal[0] + X_q[1]*x_error_signal[1] + X_q[2]*x_error_signal[2];
		vc[1] = vc[0];
		vc[0] = vc[1] + Y_q[0]*y_error_signal[0] + Y_q[1]*y_error_signal[1] + Y_q[2]*y_error_signal[2];

		// Speed
		u_feedback[1] = u_feedback[0];
		u_feedback[0] = GT_pose.position.x;
		u_signal = (u_feedback[0]-u_feedback[1])/0.01;
		v_feedback[1] = v_feedback[0];
		v_feedback[0] = GT_pose.position.y;
		v_signal = (v_feedback[0]-v_feedback[1])/0.01;

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

	attitudeRateMixerRefsCallback(omega, pitch_ref[0], roll_ref[0], rpy_ref.yaw);

	return true;
}

euler_angles CrazyfliePositionController::quaternion2euler(geometry_msgs::Quaternion quat) {
		euler_angles rpy;

		// roll (x-axis rotation)
		double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
		double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
		rpy.roll = std::atan2(sinr_cosp, cosr_cosp) * (180 / 3.14159265);

		// pitch (y-axis rotation)
		double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
		if (std::abs(sinp) >= 1)
				rpy.pitch = std::copysign(3.14159265 / 2, sinp) * (180 / 3.14159265);
		else
				rpy.pitch = std::asin(sinp) * (180 / 3.14159265);

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
		double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
		rpy.yaw = std::atan2(siny_cosp, cosy_cosp) * (180 / 3.14159265);

		return rpy;
}

void CrazyfliePositionController::attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double yaw){
	uned_crazyflie_controllers::AttitudeRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.pitch = pitch;
	ref_msg.roll = roll;
	ref_msg.yaw = yaw;

	m_pub_control_signal.publish(ref_msg);

	std_msgs::Float64 msg_omega;
	msg_omega.data = omega;
	m_pub_omega.publish(msg_omega);
}

void CrazyfliePositionController::positionreferenceCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	ref_pose.position = msg->position;
	ref_pose.orientation = msg->orientation;
	ROS_INFO("New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
}

void CrazyfliePositionController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	GT_pose.position = msg->position;
	GT_pose.orientation = msg->orientation;
}
