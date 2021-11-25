#include <uned_crazyflie_controllers/CrazyflieAttitudeController.h>

bool CrazyflieAttitudeController::initialize()
{
	ROS_INFO("CrazyflieAttitudeController::inicialize() ok.");

	// Lectura de parámetros
	// Pitch Controller
	if(m_nh_params.hasParam("PitchKp") && m_nh_params.hasParam("PitchKi") && m_nh_params.hasParam("PitchKd")){
			m_nh_params.getParam("PitchKp", kp);
			m_nh_params.getParam("PitchKi", ki);
			m_nh_params.getParam("PitchKd", kd);
			m_nh_params.getParam("PitchTd", td);
			pitch_controller = init_controller("Pitch", kp, ki, kd, td, 100, 720.0, -720.0);
	}else{
			pitch_controller = init_controller("Pitch", 6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0);
	}
	// Roll Controller
	if(m_nh_params.hasParam("RollKp") && m_nh_params.hasParam("RollKi") && m_nh_params.hasParam("RollKd")){
			m_nh_params.getParam("RollKp", kp);
			m_nh_params.getParam("RollKi", ki);
			m_nh_params.getParam("RollKd", kd);
			m_nh_params.getParam("RollTd", td);
			roll_controller = init_controller("Roll", kp, ki, kd, td, 100, 720.0, -720.0);
	}else{
			roll_controller = init_controller("Roll", 6.0, 3.0, 0.0, 0.0, 100, 720.0, -720.0);
	}
	// Yaw Controller
	if(m_nh_params.hasParam("YawKp") && m_nh_params.hasParam("YawKi") && m_nh_params.hasParam("YawKd")){
			m_nh_params.getParam("YawKp", kp);
			m_nh_params.getParam("YawKi", ki);
			m_nh_params.getParam("YawKd", kd);
			m_nh_params.getParam("YawTd", td);
			yaw_controller = init_controller("Yaw", kp, ki, kd, td, 100, 400.0, -400.0);
	}else{
			yaw_controller = init_controller("Yaw", 6.0, 1.0, 0.349, 0.05816, 100, 400.0, -400.0);
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
			// Yaw controller
			yaw_controller.error[0] = (yaw_ref - rpy_state.yaw);
			dyaw_ref = pid_controller(yaw_controller, dt);

			rateMixerRefsCallback(dpitch_ref,droll_ref, dyaw_ref);
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
	yaw_ref = msg->yaw;
	if (!first_ref_received)
			first_ref_received = true;
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
