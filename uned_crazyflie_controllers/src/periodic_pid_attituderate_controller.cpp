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

    // Pitch Controller
    init_controller("Pitch", pitch_controller, 2.0, 0.5, 0.0, 0.0, 100, 1.0, -1.0);
    // Yaw Controller
    init_controller("Yaw", yaw_controller, 6.0, 1.0, 0.3499, 0.0583, 100, 20.0, -20.0);
    // Publisher:
    // pub_cmd_ = this->create_publisher<uned_crazyflie_config::msg::Cmdsignal>("cf_cmd_control", 10);

    // Subscriber:
    // Crazyflie Pose {Real: /pose; Sim: /ground_truth/pose}
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&AttitudeRateController::gtposeCallback, this, _1));

    return true;
}


bool AttitudeRateController::iterate(){
    RCLCPP_WARN(this->get_logger(), "Attitude & Rate Controller in progress ...");
    
    // Feedback:
    rpy_state = quaternion2euler(GT_pose.orientation);

    // Attitude Controller
    // Pitch controller
    dpitch_ref = pid_controller(pitch_controller, 0.002);
    // Roll controller
    droll_ref = pid_controller(roll_controller, 0.002);
    // Yaw controller
    yaw_controller.error[0] = (rpy_state.yaw - rpy_state.yaw);
    dyaw_ref = pid_controller(yaw_controller, dt);

    // Rate Controller
    // dPitch controller
    delta_pitch = pid_controller(dpitch_controller, 0.002);
    // dRoll controller
    delta_roll = pid_controller(droll_controller, 0.002);
    // dYaw controller
    delta_yaw = pid_controller(dyaw_controller, 0.002);

    // Mixer Controller

  return true;
}


int main(int argc, char ** argv){
  try{
    rclcpp::init(argc, argv);
    auto crazyflie_attituderate_controller = std::make_shared<AttitudeRateController>();
    rclcpp::Rate loop_rate(500);
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
