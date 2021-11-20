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
	/*
	if(this->has_parameter("Zq1") && this->has_parameter("Zq2") && this->has_parameter("Zq3")){
		this->get_parameter("Zq1", Z_q[0]);
		this->get_parameter("Zq2", Z_q[1]);
		this->get_parameter("Zq3", Z_q[2]);
		RCLCPP_INFO(this->get_logger(),"Altitude PID(Z) Parameters: \t%f \t%f \t%f", Z_q[0], Z_q[1], Z_q[2]);
	}
	*/

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

	return true;
}

int main(int argc, char ** argv){
	try{
		rclcpp::init(argc, argv);
		auto crazyflie_position_controller = std::make_shared<PositionController>();
		rclcpp::Rate loop_rate(100);
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
