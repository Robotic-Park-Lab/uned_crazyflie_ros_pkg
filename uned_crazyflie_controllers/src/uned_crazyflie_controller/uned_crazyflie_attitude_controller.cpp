#include <uned_crazyflie_controllers/CrazyflieAttitudeController.h>

bool CrazyflieAttitudeController::initialize()
{
	ROS_INFO("CrazyflieAttitudeController::inicialize() ok.");

	// Lectura de parámetros
	//Pitch
	pitch_controller = init_controller("Pitch", 6.0, 3.0, 0.0, 0.0, 100, 50.0, -50.0);
	// Roll
	roll_controller = init_controller("Roll", 6.0, 3.0, 0.0, 0.0, 100, 50.0, -50.0);

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

	if (first_ref_received && first_pose_received) {
			ROS_INFO_ONCE("AttitudeRateController::iterate(). Running ...");
			// Feedback:
			rpy_state = quaternion2euler(m_GT_pose.orientation);

			// Attitude Controller
			// Pitch controller
			pitch_controller.error[0] = (pitch_ref - rpy_state.pitch);
			dpitch_ref = pid_controller(pitch_controller, dt);
			// Roll controller
			roll_controller.error[0] = (roll_ref - rpy_state.roll);
			droll_ref = pid_controller(roll_controller, dt);

			rateMixerRefsCallback(dpitch_ref,droll_ref, 0.0);
	}
	else {
			ROS_INFO_ONCE("AttitudeRateController::iterate(). Waiting reference & feedback orientation");
	}

	return true;
}

void CrazyflieAttitudeController::gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	m_GT_pose.position = msg->position;
	m_GT_pose.orientation = msg->orientation;
	if (!first_pose_received)
			first_pose_received = true;
}

void CrazyflieAttitudeController::attitudeRefsCallback(const uned_crazyflie_controllers::AttitudeRefs::ConstPtr& msg)
{
	pitch_ref = msg->pitch;
	roll_ref = msg->roll;
	if (!first_ref_received)
			first_ref_received = true;
}
void CrazyflieAttitudeController::rateMixerRefsCallback(const double dpitch, const double droll, const double dyaw)
{
	uned_crazyflie_controllers::RateMixerRefs ref_msg;

	ref_msg.timestamp = ros::Time::now().toSec();
	ref_msg.dpitch = dpitch;
	ref_msg.droll = droll;

	m_pub_control_signal.publish(ref_msg);
}
