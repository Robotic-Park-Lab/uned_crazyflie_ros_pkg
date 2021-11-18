#include "uned_crazyflie_controllers/CrazyflieAttitudeRateController.hpp"

using std::placeholders::_1;

bool AttitudeRateController::initialize(){
    RCLCPP_INFO(this->get_logger(),"AttitudeRateController::inicialize() ok.");

    // Lectura de parámetros
    RCLCPP_INFO(this->get_logger(),"TO-DO: Read Params.");
    m_controller_type = "PID";
    m_robot_id = "dron_test";
    m_controller_mode = "close loop";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s, \tMode: %s", m_controller_type, m_robot_id, m_controller_mode);

    // Z Controller
    init_controller("Pitch", pitch_controller, 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);

    // Publisher:
    // pub_cmd_ = this->create_publisher<uned_crazyflie_config::msg::Cmdsignal>("cf_cmd_control", 10);

    // Subscriber:
    // Crazyflie Pose
    // GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("ground_truth/pose", 10, std::bind(&PositionController::gtposeCallback, this, _1));
    // GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&AttitudeRateController::gtposeCallback, this, _1));

    return true;
}


bool AttitudeRateController::iterate(){
    RCLCPP_WARN(this->get_logger(), "Attitude & Rate Controller in progress ...");


  return true;
}


int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto crazyflie_attituderate_controller = std::make_shared<AttitudeRateController>();
    rclcpp::Rate loop_rate(250);
    crazyflie_attituderate_controller->initialize();

    while (rclcpp::ok()){
      rclcpp::spin_some(crazyflie_attituderate_controller);
      crazyflie_attituderate_controller->iterate();
      loop_rate.sleep();
    }
    return 0;
  } catch (std::exception &e){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s",e.what());
    }
}
