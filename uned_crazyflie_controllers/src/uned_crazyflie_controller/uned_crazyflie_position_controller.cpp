
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
	if(m_nh_params.hasParam("Yawq1") && m_nh_params.hasParam("Yawq2") && m_nh_params.hasParam("Yawq3")){
		m_nh_params.getParam("Yawq1", Yaw_q[0]);
		m_nh_params.getParam("Yawq2", Yaw_q[1]);
		m_nh_params.getParam("Yawq3", Yaw_q[2]);
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
	m_sub_pos_ref = m_nh.subscribe( "position_reference", 10, &CrazyfliePositionController::positionreferenceCallback, this);

	u_feedback[0] = m_x_init;
	v_feedback[0] = m_y_init;

	m_ref_pose.position.x = m_x_init;
	m_ref_pose.position.y = m_y_init;
	m_ref_pose.position.z = m_z_init;
	m_ref_pose.orientation.x = 0;
	m_ref_pose.orientation.y = 0;
	m_ref_pose.orientation.z = 0;
	m_ref_pose.orientation.w = 1;

	ROS_INFO("In progress ...");
	ROS_INFO_STREAM("Goal Pose: " << m_ref_pose);
	ROS_INFO("Altitude PID(Z) Parameters: \t%f \t%f \t%f", Z_q[0], Z_q[1], Z_q[2]);
	ROS_INFO("X PID(Z) Parameters: \t%f \t%f \t%f", X_q[0], X_q[1], X_q[2]);
	ROS_INFO("U PID(Z) Parameters: \t%f \t%f \t%f", U_q[0], U_q[1], U_q[2]);
	ROS_INFO("Y PID(Z) Parameters: \t%f \t%f \t%f", Y_q[0], Y_q[1], Y_q[2]);
	ROS_INFO("V PID(Z) Parameters: \t%f \t%f \t%f", V_q[0], V_q[1], V_q[2]);
	ROS_INFO("Yaw PID(Z) Parameters: \t%f \t%f \t%f", Yaw_q[0], Yaw_q[1], Yaw_q[2]);
	ROS_ERROR("Hay que esperar a que gazebo esté listo para lanzar el nodo");

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
		omega = delta_omega[0]+(we-4070.3)/0.2685;
		std_msgs::Float64 msg_omega;
		msg_omega.data = omega;
		m_pub_omega.publish(msg_omega);
	}
	// X-Y Controller
	// Convert quaternion to yw
	double siny_cosp_ref = 2 * (m_ref_pose.orientation.w*m_ref_pose.orientation.z+m_ref_pose.orientation.x*m_ref_pose.orientation.y);
	double cosy_cosp_ref = 1 - 2 * (m_ref_pose.orientation.y*m_ref_pose.orientation.y + m_ref_pose.orientation.z*m_ref_pose.orientation.z);
	double yaw_ref = std::atan2(siny_cosp_ref,cosy_cosp_ref);
	double siny_cosp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.z+m_GT_pose.orientation.x*m_GT_pose.orientation.y);
	double cosy_cosp = 1 - 2 * (m_GT_pose.orientation.y*m_GT_pose.orientation.y + m_GT_pose.orientation.z*m_GT_pose.orientation.z);
	double yaw = std::atan2(siny_cosp,cosy_cosp);
	{
		// Position
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
	// Yaw Controller
	{
		// Update error
		yaw_error_signal[2] = yaw_error_signal[1];
		yaw_error_signal[1] = yaw_error_signal[0];
		yaw_error_signal[0] = (yaw_ref-yaw)*180/3.14159265;

		// Update signal vector
		dyaw_ref[1] = dyaw_ref[0];
		dyaw_ref[0] = dyaw_ref[1] + Yaw_q[0]*yaw_error_signal[0] + Yaw_q[1]*yaw_error_signal[1] + Yaw_q[2]*yaw_error_signal[2];
		std_msgs::Float64 msg_dyaw;
		msg_dyaw.data = dyaw_ref[0];
		m_pub_dyaw.publish(msg_dyaw);
	}

	attitudeRateMixerRefsCallback(omega, pitch_ref[0], roll_ref[0], dyaw_ref[0]);

	return true;
}

void CrazyfliePositionController::attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw)
{
	uned_crazyflie_controllers::AttitudeRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.pitch = pitch;
	ref_msg.roll = roll;

	m_pub_control_signal.publish(ref_msg);
}

void CrazyfliePositionController::positionreferenceCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_ref_pose.position = msg->position;
	m_ref_pose.orientation = msg->orientation;
	ROS_INFO("New Pose: x: %f \ty: %f \tz: %f", m_ref_pose.position.x, m_ref_pose.position.y, m_ref_pose.position.z);
}

void CrazyfliePositionController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}
