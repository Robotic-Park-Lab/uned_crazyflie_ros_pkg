#include <uned_crazyflie_controllers/CrazyflieAttitudeController.h>

bool CrazyflieAttitudeController::initialize()
{
	ROS_INFO("CrazyflieAttitudeController::inicialize() ok.");

	// Publisher:
	m_pub_control_signal = m_nh.advertise<uned_crazyflie_controllers::RateMixerRefs>("ratemixer_controller_ref", 10);

	// Subscriber:
	// Crazyflie Pose
	m_sub_GT_pose = m_nh.subscribe( "ground_truth/pose", 10, &CrazyflieAttitudeController::gtposeCallback, this);
	// Reference
	m_sub_attitude_ref = m_nh.subscribe( "attitude_controller_ref", 10, &CrazyflieAttitudeController::attitudeRefsCallback, this);
	ROS_INFO_THROTTLE(1, "In revision ...");
	return true;
}

bool CrazyflieAttitudeController::iterate()
{

	// Roll
	double sinr_cosp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.x+m_GT_pose.orientation.y*m_GT_pose.orientation.z);
	double cosr_cosp = 1 - 2 * (m_GT_pose.orientation.x*m_GT_pose.orientation.x+m_GT_pose.orientation.y*m_GT_pose.orientation.y);
	roll_dron = std::atan2(sinr_cosp,cosr_cosp)*180/3.14159265;
	// Pitch
	double sinp = 2 * (m_GT_pose.orientation.w*m_GT_pose.orientation.y+m_GT_pose.orientation.y*m_GT_pose.orientation.z);
	if(std::abs(sinp) >= 1)
		pitch_dron = std::copysign(M_PI/2, sinp)*180/3.14159265;
	else
		pitch_dron = std::asin(sinp)*180/3.14159265;

	// Pitch Controller
	{
		// Update error vector
		pitch_error[2] = pitch_error[1];
		pitch_error[1] = pitch_error[0];
		pitch_error[0] = pitch_ref - pitch_dron;

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
		roll_error[0] = roll_ref - roll_dron;

		// Update signal vector
		droll[1] = droll[0];
		droll[0] = droll[1] + Theta_q[0]*roll_error[0] + Theta_q[1]*roll_error[1] + Theta_q[2]*roll_error[2];

		// Saturation
		if(droll[0]>314.1593)
			droll[0] = 314.1593;
		if(droll[0]<-314.1593)
			droll[0] = -314.1593;
	}

	rateMixerRefsCallback(omega,dpitch[0],droll[0],dyaw);

	return true;
}

void CrazyflieAttitudeController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
}

void CrazyflieAttitudeController::attitudeRefsCallback(const uned_crazyflie_controllers::AttitudeRefs::ConstPtr& msg)
{
	omega = msg->omega;
	pitch_ref = msg->pitch;
	roll_ref = msg->roll;
	dyaw = msg->dyaw;
}
void CrazyflieAttitudeController::rateMixerRefsCallback(const double omega, const double dpitch, const double droll, const double dyaw)
{
	uned_crazyflie_controllers::RateMixerRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.omega = omega;
	ref_msg.dpitch = dpitch;
	ref_msg.droll = droll;
	ref_msg.dyaw = dyaw;

	m_pub_control_signal.publish(ref_msg);
}
