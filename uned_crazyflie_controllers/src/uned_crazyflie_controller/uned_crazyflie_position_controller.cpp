
#include <uned_crazyflie_controllers/CrazyfliePositionController.h>

bool CrazyfliePositionController::initialize()
{
	ROS_INFO("CrazyfliePositionController::inicialize() ok.");

	// Lectura de parámetros
	    ROS_INFO("TO-DO: Read Params.");
	m_nh_params.getParam("CONTROLLER_TYPE", m_controller_type);
	m_nh_params.getParam("ROBOT_ID", m_robot_id);
	m_nh_params.getParam("CONTROLLER_MODE", m_controller_mode);
	m_nh_params.getParam("X_POS", m_x_init);
	m_nh_params.getParam("Y_POS", m_y_init);
	m_nh_params.getParam("Z_POS", m_z_init);

	// Z Controller
	str_id = "Z";
	z_controller = init_controller(str_id.c_str(), 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);
	// W Controller
	str_id = "W";
	w_controller = init_controller(str_id.c_str(), 25.0, 15.0, 0.0, 0.0, 100, 1160.0, -640.0);
	// X Controller
	str_id = "X";
	x_controller = init_controller(str_id.c_str(), 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);
	// U Controller
	str_id = "U";
	u_controller = init_controller(str_id.c_str(), 25.0, 2.0, 0.0, 0.0, 100, 30.0, -30.0);
	// Y Controller
	str_id = "Y";
	y_controller = init_controller(str_id.c_str(), 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);
	// V Controller
	str_id = "V";
	v_controller = init_controller(str_id.c_str(), 25.0, 2.0, 0.0, 0.0, 100, 30.0, -30.0);
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

	return true;
}

bool CrazyfliePositionController::iterate()
{
	if (first_pose_received && first_ref_received) {
		// Z Controller
		z_controller.error[0] = ref_pose.position.z - GT_pose.position.z;
		w_ref = pid_controller(z_controller, dt);

		// W Controller
		w_feedback[1] = w_feedback[0];
		w_feedback[0] = GT_pose.position.z;
		w_signal = (w_feedback[0] - w_feedback[1]) / dt;
		w_controller.error[0] = w_ref - w_signal;
		thrust = pid_controller(w_controller, dt);

		thrust = thrust * 1000 + 36000;

		// Convert quaternion to yw
		rpy_ref = quaternion2euler(ref_pose.orientation);
		rpy_state = quaternion2euler(GT_pose.orientation);

		x_global_error = ref_pose.position.x - GT_pose.position.x;
		y_global_error = ref_pose.position.y - GT_pose.position.y;
		// X Controller
		x_controller.error[0] = x_global_error * cos(rpy_state.yaw) + y_global_error * sin(rpy_state.yaw);
		u_ref = pid_controller(x_controller, dt);
		// Y Controller
		y_controller.error[0] = -x_global_error * sin(rpy_state.yaw) + y_global_error * cos(rpy_state.yaw);
		v_ref = pid_controller(y_controller, dt);

		// Speed
		u_feedback[1] = u_feedback[0];
		u_feedback[0] = GT_pose.position.x;
		u_signal = (u_feedback[0] - u_feedback[1]) / dt;
		v_feedback[1] = v_feedback[0];
		v_feedback[0] = GT_pose.position.y;
		v_signal = (v_feedback[0] - v_feedback[1]) / dt;

		// U Controller
		u_controller.error[0] = u_ref - u_signal;
		pitch = pid_controller(u_controller, dt);

		// V Controller
		v_controller.error[0] = v_ref - v_signal;
		roll = pid_controller(v_controller, dt);

		attitudeRateMixerRefsCallback(thrust, pitch, roll, rpy_ref.yaw);

	}

	return true;
}

void CrazyfliePositionController::attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double yaw)
{
	std_msgs::Float64MultiArray msg_cmd;

	msg_cmd.data = { thrust, roll, pitch, rpy_ref.yaw };

	m_pub_control_signal.publish(msg_cmd);
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
