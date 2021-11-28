#include <uned_crazyflie_controllers/CrazyflieAttitudeController.h>

bool CrazyflieAttitudeController::initialize()
{
	ROS_INFO("CrazyflieAttitudeController::inicialize() ok.");

	// Lectura de parámetros
	if(m_nh_params.hasParam("Phiq1") && m_nh_params.hasParam("Phiq2") && m_nh_params.hasParam("Phiq3")){
		m_nh_params.getParam("Phiq1", Phi_q[0]);
		m_nh_params.getParam("Phiq2", Phi_q[1]);
		m_nh_params.getParam("Phiq3", Phi_q[2]);
	}
	if(m_nh_params.hasParam("Thetaq1") && m_nh_params.hasParam("Thetaq2") && m_nh_params.hasParam("Thetaq3")){
		m_nh_params.getParam("Thetaq1", Theta_q[0]);
		m_nh_params.getParam("Thetaq2", Theta_q[1]);
		m_nh_params.getParam("Thetaq3", Theta_q[2]);
	}
	if(m_nh_params.hasParam("Yawq1") && m_nh_params.hasParam("Yawq2") && m_nh_params.hasParam("Yawq3")){
		m_nh_params.getParam("Yawq1", Yaw_q[0]);
		m_nh_params.getParam("Yawq2", Yaw_q[1]);
		m_nh_params.getParam("Yawq3", Yaw_q[2]);
	}

	// Publisher:
	m_pub_control_signal = m_nh.advertise<uned_crazyflie_controllers::RateMixerRefs>("ratemixer_controller_ref", 10);
	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyflieAttitudeController::gtposeCallback, this);
	// Reference
	m_sub_attitude_ref = m_nh.subscribe( "attitude_controller_ref", 10, &CrazyflieAttitudeController::attitudeRefsCallback, this);

	return true;
}

bool CrazyflieAttitudeController::iterate()
{
	// Feedback:
	rpy_state = quaternion2euler(GT_pose.orientation);

	// Pitch Controller
	{
		// Update error vector
		pitch_error[2] = pitch_error[1];
		pitch_error[1] = pitch_error[0];
		pitch_error[0] = pitch_ref - rpy_state.pitch;

		// Update signal vector
		dpitch[1] = dpitch[0];
		dpitch[0] = dpitch[1] + Phi_q[0]*pitch_error[0] + Phi_q[1]*pitch_error[1] + Phi_q[2]*pitch_error[2];

		// Saturation
		if(dpitch[0]>180.0)
			dpitch[0] = 180.0;
		if(dpitch[0]<-180.0)
			dpitch[0] = -180.0;
	}
	// Roll Controller
	{
		// Update error vector
		roll_error[2] = roll_error[1];
		roll_error[1] = roll_error[0];
		roll_error[0] = roll_ref - rpy_state.roll;

		// Update signal vector
		droll[1] = droll[0];
		droll[0] = droll[1] + Theta_q[0]*roll_error[0] + Theta_q[1]*roll_error[1] + Theta_q[2]*roll_error[2];

		// Saturation
		if(droll[0]>314.1593)
			droll[0] = 314.1593;
		if(droll[0]<-314.1593)
			droll[0] = -314.1593;
	}
	// Yaw Controller
	{
		// Update error
		yaw_error_signal[2] = yaw_error_signal[1];
		yaw_error_signal[1] = yaw_error_signal[0];
		yaw_error_signal[0] = (yaw_ref-rpy_state.yaw);

		// Update signal vector
		dyaw[1] = dyaw[0];
		dyaw[0] = dyaw[1] + Yaw_q[0]*yaw_error_signal[0] + Yaw_q[1]*yaw_error_signal[1] + Yaw_q[2]*yaw_error_signal[2];
	}

	rateMixerRefsCallback(dpitch[0],droll[0],dyaw[0]);

	return true;
}

void CrazyflieAttitudeController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	GT_pose.position = msg->position;
	GT_pose.orientation = msg->orientation;
}

void CrazyflieAttitudeController::attitudeRefsCallback(const uned_crazyflie_controllers::AttitudeRefs::ConstPtr& msg)
{
	pitch_ref = msg->pitch;
	roll_ref = msg->roll;
	yaw_ref = msg->yaw;
}
void CrazyflieAttitudeController::rateMixerRefsCallback(const double dpitch, const double droll, const double dyaw)
{
	uned_crazyflie_controllers::RateMixerRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.dpitch = dpitch;
	ref_msg.droll = droll;
	ref_msg.dyaw = dyaw;

	m_pub_control_signal.publish(ref_msg);
}

euler_angles CrazyflieAttitudeController::quaternion2euler(geometry_msgs::Quaternion quat){
	euler_angles rpy;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
	double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
	rpy.roll = std::atan2(sinr_cosp, cosr_cosp) * (180 / PI);

	// pitch (y-axis rotation)
	double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
	if (std::abs(sinp) >= 1)
	rpy.pitch = std::copysign(PI / 2, sinp) * (180 / PI);
	else
	rpy.pitch = std::asin(sinp) * (180 / PI);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
	double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
	rpy.yaw = std::atan2(siny_cosp, cosy_cosp) * (180 / PI);

	return rpy;
}
