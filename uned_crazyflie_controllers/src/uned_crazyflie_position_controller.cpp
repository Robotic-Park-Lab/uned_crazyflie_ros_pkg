#include "uned_crazyflie_controllers/CrazyfliePositionController.hpp"
using std::placeholders::_1;

bool PositionController::initialize(){
  RCLCPP_INFO(this->get_logger(),"PositionController::inicialize() ok.");

  // Lectura de parámetros
	this->get_parameter("CONTROLLER_TYPE", m_controller_type);
	this->get_parameter("ROBOT_ID", m_robot_id);
	this->get_parameter("CONTROLLER_MODE", m_controller_mode);
	this->get_parameter("X_POS", m_x_init);
	this->get_parameter("Y_POS", m_y_init);
	this->get_parameter("Z_POS", m_z_init);

  if(this->has_parameter("Zq1") && this->has_parameter("Zq2") && this->has_parameter("Zq3")){
		this->get_parameter("Zq1", Z_q[0]);
		this->get_parameter("Zq2", Z_q[1]);
		this->get_parameter("Zq3", Z_q[2]);
    RCLCPP_INFO(this->get_logger(),"Altitude PID(Z) Parameters: \t%f \t%f \t%f", Z_q[0], Z_q[1], Z_q[2]);
	}
  if(this->has_parameter("Xq1") && this->has_parameter("Xq2") && this->has_parameter("Xq3")){
		this->get_parameter("Xq1", X_q[0]);
		this->get_parameter("Xq2", X_q[1]);
		this->get_parameter("Xq3", X_q[2]);
    RCLCPP_INFO(this->get_logger(),"X PID(Z) Parameters: \t%f \t%f \t%f", X_q[0], X_q[1], X_q[2]);
	}
  if(this->has_parameter("Uq1") && this->has_parameter("Uq2") && this->has_parameter("Uq3")){
		this->get_parameter("Uq1", U_q[0]);
		this->get_parameter("Uq2", U_q[1]);
		this->get_parameter("Uq3", U_q[2]);
    RCLCPP_INFO(this->get_logger(),"U PID(Z) Parameters: \t%f \t%f \t%f", U_q[0], U_q[1], U_q[2]);
	}
	if(this->has_parameter("Yq1") && this->has_parameter("Yq2") && this->has_parameter("Yq3")){
		this->get_parameter("Yq1", Y_q[0]);
		this->get_parameter("Yq2", Y_q[1]);
		this->get_parameter("Yq3", Y_q[2]);
    RCLCPP_INFO(this->get_logger(),"Y PID(Z) Parameters: \t%f \t%f \t%f", Y_q[0], Y_q[1], Y_q[2]);
	}
	if(this->has_parameter("Vq1") && this->has_parameter("Vq2") && this->has_parameter("Vq3")){
		this->get_parameter("Vq1", V_q[0]);
		this->get_parameter("Vq2", V_q[1]);
		this->get_parameter("Vq3", V_q[2]);
    RCLCPP_INFO(this->get_logger(),"V PID(Z) Parameters: \t%f \t%f \t%f", V_q[0], V_q[1], V_q[2]);
	}
	if(this->has_parameter("Yawq1") && this->has_parameter("Yawq2") && this->has_parameter("Yawq3")){
		this->get_parameter("Yawq1", Yaw_q[0]);
		this->get_parameter("Yawq2", Yaw_q[1]);
		this->get_parameter("Yawq3", Yaw_q[2]);
    RCLCPP_INFO(this->get_logger(),"Yaw PID(Z) Parameters: \t%f \t%f \t%f", Yaw_q[0], Yaw_q[1], Yaw_q[2]);
	}

  // Publisher:
	// Referencias para los controladores PID Attitude y Rate
  pub_omega = this->create_publisher<std_msgs::msg::Float64>("omega_signal",10);
  pub_dyaw = this->create_publisher<std_msgs::msg::Float64>("dyaw_controller_ref", 10);
  pub_control_signal = this->create_publisher<std_msgs::msg::Float64MultiArray>("attitude_controller_ref", 10);

  // Subscriber:
	// Crazyflie Pose
  GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("ground_truth/pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
	// Reference:
  ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("position_reference", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

  // Init values
  u_feedback[0] = m_x_init;
	v_feedback[0] = m_y_init;

  ref_pose.position.x = m_x_init;
	ref_pose.position.y = m_y_init;
	ref_pose.position.z = m_z_init;
	ref_pose.orientation.x = 0;
	ref_pose.orientation.y = 0;
	ref_pose.orientation.z = 0;
	ref_pose.orientation.w = 1;
  RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);

  return true;
}

bool PositionController::iterate(){
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
		std_msgs::msg::Float64 msg_omega;
		msg_omega.data = omega;
		pub_omega->publish(msg_omega);
	}

  // X-Y Controller
	// Convert quaternion to yw
	double siny_cosp_ref = 2 * (ref_pose.orientation.w*ref_pose.orientation.z+ref_pose.orientation.x*ref_pose.orientation.y);
	double cosy_cosp_ref = 1 - 2 * (ref_pose.orientation.y*ref_pose.orientation.y + ref_pose.orientation.z*ref_pose.orientation.z);
	double yaw_ref = std::atan2(siny_cosp_ref,cosy_cosp_ref);
	double siny_cosp = 2 * (GT_pose.orientation.w*GT_pose.orientation.z+GT_pose.orientation.x*GT_pose.orientation.y);
	double cosy_cosp = 1 - 2 * (GT_pose.orientation.y*GT_pose.orientation.y + GT_pose.orientation.z*GT_pose.orientation.z);
	double yaw = std::atan2(siny_cosp,cosy_cosp);
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
		double u_signal = (u_feedback[0]-u_feedback[1])/0.01;
		v_feedback[1] = v_feedback[0];
		v_feedback[0] = GT_pose.position.y;
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

    this->attitudeRateMixerRefsCallback(pitch_ref[0], roll_ref[0]);
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
		std_msgs::msg::Float64 msg_dyaw;
		msg_dyaw.data = dyaw_ref[0];
		pub_dyaw->publish(msg_dyaw);
	}

  return true;
}

int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);

    auto crazyflie_position_controller = std::make_shared<PositionController>();

    rclcpp::Rate loop_rate(10);

    crazyflie_position_controller->initialize();

    while (rclcpp::ok()){
      rclcpp::spin_some(crazyflie_position_controller);

      crazyflie_position_controller->iterate();

      loop_rate.sleep();
    }

    return 0;
  } catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}

}
