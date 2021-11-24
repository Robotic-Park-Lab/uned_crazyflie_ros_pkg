#include "uned_crazyflie_controllers/CrazyflieAttitudeRateController.hpp"

using std::placeholders::_1;

bool AttitudeRateController::initialize(){
    RCLCPP_INFO(this->get_logger(),"AttitudeRateController::inicialize() ok.");

    // Lectura de parámetros
    RCLCPP_INFO(this->get_logger(),"TO-DO: Read Params.");
    m_controller_type = "PID";
    m_robot_id = "dron01";
    m_controller_mode = "close loop";
    RCLCPP_INFO(this->get_logger(),"Controller Type: %s, \tRobot id: %s, \tMode: %s", m_controller_type.c_str(), m_robot_id.c_str(), m_controller_mode.c_str());

    // Pitch Controller
    str_id = "Pitch";
    init_controller(str_id.c_str(), pitch_controller, 6.0, 3.0, 0.0, 0.0, 100, 50.0, -50.0);
    // Roll Controller
    str_id = "Roll";
    init_controller(str_id.c_str(), roll_controller, 6.0, 3.0, 0.0, 0.0, 100, 50.0, -50.0);
    // Yaw Controller
    str_id = "Yaw";
    init_controller(str_id.c_str(), yaw_controller, 6.0, 1.0, 0.3499, 0.0583, 100, 20.0, -20.0);
    // dPitch Controller
    str_id = "dPitch";
    init_controller(str_id.c_str(), dpitch_controller, 250.0, 500.0, 2.5, 0.01, 100, 720.0, -720.0);
    // dRoll Controller
    str_id = "dRoll";
    init_controller(str_id.c_str(), droll_controller, 250.0, 500.0, 2.5, 0.01, 100, 720.0, -720.0);
    // dYaw Controller
    str_id = "dYaw";
    init_controller(str_id.c_str(), dyaw_controller, 120.0, 16.7, 0.0, 0.0, 100, 400.0, -400.0);

    // Publisher:
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_control", 10);

    // Subscriber:
    // Crazyflie Pose {Real: /pose; Sim: /ground_truth/pose}
    GT_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("pose", 10, std::bind(&AttitudeRateController::gtposeCallback, this, _1));
    // Reference from Off-board Controller
    ref_cmd_ = this->create_subscription<uned_crazyflie_config::msg::Cmdsignal>("onboard_cmd", 10, std::bind(&AttitudeRateController::refcmdCallback, this, _1));

    return true;
}


bool AttitudeRateController::iterate(){
    RCLCPP_INFO_ONCE(this->get_logger(), "AttitudeRateController::iterate(). ok.");
    if (first_ref_received && first_pose_received) {
        RCLCPP_INFO_ONCE(this->get_logger(), "AttitudeRateController::iterate(). Running ...");
        // Feedback:
        rpy_state = quaternion2euler(GT_pose.orientation);

        // Attitude Controller
        // Pitch controller
        pitch_controller.error[0] = (ref_cmd.pitch - rpy_state.pitch);
        dpitch_ref = pid_controller(pitch_controller, dt);
        // Roll controller
        roll_controller.error[0] = (ref_cmd.roll - rpy_state.roll);
        droll_ref = pid_controller(roll_controller, dt);
        // Yaw controller
        yaw_controller.error[0] = (ref_cmd.yaw - rpy_state.yaw);
        dyaw_ref = pid_controller(yaw_controller, dt);

        // Rate Controller
        // dPitch controller
        dpitch_feedback[1] = dpitch_feedback[0];
        dpitch_feedback[0] = rpy_state.pitch;
        dpitch_signal = (dpitch_feedback[0] - dpitch_feedback[1]) / dt;
        dpitch_controller.error[0] = dpitch_ref - dpitch_signal;
        delta_pitch = pid_controller(dpitch_controller, dt);
        // dRoll controller
        droll_feedback[1] = droll_feedback[0];
        droll_feedback[0] = rpy_state.roll;
        droll_signal = (droll_feedback[0] - droll_feedback[1]) / dt;
        droll_controller.error[0] = droll_ref - droll_signal;
        delta_roll = pid_controller(droll_controller, dt);
        // dYaw controller
        dyaw_feedback[1] = dyaw_feedback[0];
        dyaw_feedback[0] = rpy_state.yaw;
        dyaw_signal = (dyaw_feedback[0] - dyaw_feedback[1]) / dt;
        dyaw_controller.error[0] = dyaw_ref - dyaw_signal;
        delta_yaw = pid_controller(dyaw_controller, dt);

        // Mixer Controller
        motors[0] = ref_cmd.thrust - 0.5 * delta_pitch - 0.5 * delta_roll - delta_yaw;
        motors[1] = ref_cmd.thrust + 0.5 * delta_pitch - 0.5 * delta_roll + delta_yaw;
        motors[2] = ref_cmd.thrust + 0.5 * delta_pitch + 0.5 * delta_roll - delta_yaw;
        motors[3] = ref_cmd.thrust - 0.5 * delta_pitch + 0.5 * delta_roll + delta_yaw;

        // Publish Control CMD
        auto msg_cmd = std_msgs::msg::Float64MultiArray();
        for (int i = 0; i < 4; i++)
            msg_cmd.data[i] = motors[i];
        pub_cmd_->publish(msg_cmd);
    }
    else {
        RCLCPP_INFO_ONCE(this->get_logger(), "AttitudeRateController::iterate(). Waiting reference & feedback orientation");
    }

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
