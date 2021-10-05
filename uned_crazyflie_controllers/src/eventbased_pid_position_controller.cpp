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
  auto pub_omega_ = this->create_publisher<std_msgs::msg::Float64>("omega_signal",10);
  auto pub_dyaw = this->create_publisher<std_msgs::msg::Float64>("dyaw_controller_ref", 10);
  auto pub_control_signal = this->create_publisher<std_msgs::msg::Float64MultiArray>("attitude_controller_ref", 10);

  // Subscriber:
	// Crazyflie Pose
  GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("ground_truth/pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
	// Reference:
  ref_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("position_reference", 10, std::bind(&PositionController::positionreferenceCallback, this, _1));

  // Init values
  u_feedback[0] = m_x_init;
	v_feedback[0] = m_y_init;

  //ref_pose.position.x = m_x_init;
	//ref_pose.position.y = m_y_init;
	//ref_pose.position.z = m_z_init;
  ref_pose.position.x = 0;
	ref_pose.position.y = 1;
	ref_pose.position.z = 2;
	ref_pose.orientation.x = 0;
	ref_pose.orientation.y = 0;
	ref_pose.orientation.z = 0;
	ref_pose.orientation.w = 1;
  RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);

  return true;
}

bool PositionController::iterate(){
  RCLCPP_INFO(this->get_logger(),"TO-DO: Altitude Controller.");
  // Altitude Controller
	if(0){
    /*
    z_error[0] = ref_pose.position.z - GT_pose.position.z;

    outP = Kp*z_error[0];
    Ki = Kp/Ti;
    z_i[0] = z_i[1] + Ki*dt*z_error[1];
    Kd = Kp*Td;
    z_d[0] = (Td/(Td+Nd+dt))*z_d[1]+(Kd*Nd/(Td+Nd*dt))*(z_error[0]-z_error[1]);
    out = outP+z_i[0]+z_d[0];
    outr = out;
    if(outr>15000){
      outr = 150000;
    }
    if(outr<-20000){
      outr = -20000;
    }
    // Antiwindup
    z_i[0] = z_i[0] + dt*(outr-out)/sqrt(Ti);

    // Update vectors
    z_error[1] = z_error[0];
    z_i[1] = z_i[0];
    z_d[1] = z_d[0];
    */
		// Update error vector
		z_error_signal[1] = z_error_signal[0];

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
    omega = 2600;
		auto msg_omega = std_msgs::msg::Float64();
		msg_omega.data = omega;
        pub_omega_->publish(msg_omega);

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
      crazyflie_position_controller->iterate();
      rclcpp::spin_some(crazyflie_position_controller);
      loop_rate.sleep();
    }
    return 0;
  } catch (std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
	}
}
